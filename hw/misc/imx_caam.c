/*
 * i.MX CAAM job rings and cryptographic descriptors.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "hw/misc/imx_caam.h"
#include "hw/core/irq.h"
#include "crypto/hash.h"
#include "crypto/cipher.h"
#include "exec/cpu-common.h"
#include "migration/vmstate.h"
#include "qemu/guest-random.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "system/address-spaces.h"
#include <gnutls/crypto.h>
#include <gnutls/x509.h>

static uint32_t caam_ld32(hwaddr address)
{
    return address_space_ldl_le(&address_space_memory, address,
                                 MEMTXATTRS_UNSPECIFIED, NULL);
}
static uint64_t caam_pointer(uint32_t *desc, unsigned *index, bool wide)
{
    uint64_t value = desc[(*index)++];
    if (wide) {
        value = value << 32 | desc[(*index)++];
    }
    return value;
}

static bool caam_dma(hwaddr pointer, uint8_t *bytes, unsigned length,
                     bool sg, bool write)
{
    if (!sg) {
        if (write) { cpu_physical_memory_write(pointer, bytes, length); }
        else { cpu_physical_memory_read(pointer, bytes, length); }
        return true;
    }
    unsigned done = 0;
    for (unsigned j = 0; j < 128 && done < length; j++) {
        uint64_t address = (uint64_t)caam_ld32(pointer) << 32 | caam_ld32(pointer + 4);
        uint32_t flags = caam_ld32(pointer + 8);
        unsigned offset = caam_ld32(pointer + 12) & 0x1fff;
        if (flags & BIT(31)) {
            pointer = address;
            continue;
        }
        unsigned count = MIN(flags & 0x3fffffff, length - done);
        if (!count) { return false; }
        if (write) { cpu_physical_memory_write(address + offset, bytes + done, count); }
        else { cpu_physical_memory_read(address + offset, bytes + done, count); }
        done += count;
        pointer += 16;
        if (flags & BIT(30)) { break; }
    }
    return done == length;
}

static void caam_ghash(uint8_t state[16], const uint8_t h[16],
                       const uint8_t *data, unsigned length)
{
    uint64_t h0 = ldq_be_p(h), h1 = ldq_be_p(h + 8);

    while (length) {
        uint8_t block[16] = {0};
        unsigned count = MIN(length, sizeof(block));
        memcpy(block, data, count);
        uint64_t x0 = ldq_be_p(state) ^ ldq_be_p(block);
        uint64_t x1 = ldq_be_p(state + 8) ^ ldq_be_p(block + 8);
        uint64_t z0 = 0, z1 = 0, v0 = h0, v1 = h1;
        for (unsigned bit = 0; bit < 128; bit++) {
            uint64_t mask = -(uint64_t)(bit < 64 ?
                            (x0 >> (63 - bit)) & 1 :
                            (x1 >> (127 - bit)) & 1);
            z0 ^= v0 & mask;
            z1 ^= v1 & mask;
            bool carry = v1 & 1;
            v1 = (v1 >> 1) | (v0 << 63);
            v0 = (v0 >> 1) ^ (carry ? UINT64_C(0xe100000000000000) : 0);
        }
        stq_be_p(state, z0);
        stq_be_p(state + 8, z1);
        data += count;
        length -= count;
    }
}

/* The CAAM GCM context contains MAC, Yi, Y0 and the two bit counts. */
static bool caam_gcm(QCryptoCipher *aes, uint32_t op, uint8_t context[64],
                      const uint8_t *nonce, unsigned nonce_length,
                      const uint8_t *aad, unsigned aad_length,
                      const uint8_t *input, uint8_t *output, unsigned length)
{
    uint8_t zero[16] = {0}, h[16], block[16];
    unsigned state = (op >> 2) & 3;
    bool encrypt = op & 1;

    if (qcrypto_cipher_encrypt(aes, zero, h, 16, NULL) < 0) {
        return false;
    }
    if (state & 1) {
        memset(context, 0, 64);
        if (nonce_length == 12) {
            memcpy(context + 32, nonce, 12);
            context[47] = 1;
        } else {
            caam_ghash(context + 32, h, nonce, nonce_length);
            stq_be_p(block, 0);
            stq_be_p(block + 8, (uint64_t)nonce_length * 8);
            caam_ghash(context + 32, h, block, 16);
        }
        memcpy(context + 16, context + 32, 16);
    }
    caam_ghash(context, h, aad, aad_length);
    stq_be_p(context + 48, ldq_be_p(context + 48) + (uint64_t)aad_length * 8);
    for (unsigned done = 0; done < length; done += 16) {
        unsigned count = MIN(length - done, 16);
        stl_be_p(context + 28, ldl_be_p(context + 28) + 1);
        if (qcrypto_cipher_encrypt(aes, context + 16, block, 16, NULL) < 0) {
            return false;
        }
        for (unsigned i = 0; i < count; i++) {
            output[done + i] = input[done + i] ^ block[i];
        }
        caam_ghash(context, h, (encrypt ? output : input) + done, count);
    }
    stq_be_p(context + 56, ldq_be_p(context + 56) + (uint64_t)length * 8);
    if (state & 2) {
        caam_ghash(context, h, context + 48, 16);
        if (qcrypto_cipher_encrypt(aes, context + 32, block, 16, NULL) < 0) {
            return false;
        }
        for (unsigned i = 0; i < 16; i++) {
            context[i] ^= block[i];
        }
    }
    return true;
}

static void caam_sha_block(uint32_t h[8], const uint8_t block[64], bool sha1)
{
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
        0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
        0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
        0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
        0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
    };
    uint32_t w[80], a = h[0], b = h[1], c = h[2], d = h[3];
    uint32_t e = h[4], f = h[5], g = h[6], z = h[7];
    for (unsigned i = 0; i < 16; i++) {
        w[i] = ldl_be_p(block + 4 * i);
    }
    for (unsigned i = 0; i < (sha1 ? 80 : 64); i++) {
        if (sha1) {
            if (i >= 16) {
                w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
            }
            uint32_t fn = i < 20 ? (b & c) | (~b & d) :
                          i < 40 || i >= 60 ? b ^ c ^ d :
                          (b & c) | (b & d) | (c & d);
            uint32_t round = i < 20 ? 0x5a827999 : i < 40 ? 0x6ed9eba1 :
                             i < 60 ? 0x8f1bbcdc : 0xca62c1d6;
            uint32_t t = rol32(a, 5) + fn + e + round + w[i];
            e = d; d = c; c = rol32(b, 30); b = a; a = t;
        } else {
            if (i >= 16) {
                uint32_t s0 = ror32(w[i - 15], 7) ^ ror32(w[i - 15], 18) ^
                              (w[i - 15] >> 3);
                uint32_t s1 = ror32(w[i - 2], 17) ^ ror32(w[i - 2], 19) ^
                              (w[i - 2] >> 10);
                w[i] = w[i - 16] + s0 + w[i - 7] + s1;
            }
            uint32_t s0 = ror32(a, 2) ^ ror32(a, 13) ^ ror32(a, 22);
            uint32_t s1 = ror32(e, 6) ^ ror32(e, 11) ^ ror32(e, 25);
            uint32_t t1 = z + s1 + ((e & f) ^ (~e & g)) + k[i] + w[i];
            uint32_t t2 = s0 + ((a & b) ^ (a & c) ^ (b & c));
            z = g; g = f; f = e; e = d + t1;
            d = c; c = b; b = a; a = t1 + t2;
        }
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    if (!sha1) { h[5] += f; h[6] += g; h[7] += z; }
}

static bool caam_sha(uint32_t op, uint8_t context[64],
                      const uint8_t *data, unsigned length)
{
    static const uint32_t sha256_iv[8] = {
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
    };
    bool sha1 = ((op >> 16) & 0xff) == 0x41;
    unsigned words = sha1 ? 5 : 8, state = (op >> 2) & 3;
    uint32_t h[8] = {0};
    uint64_t count = 0;
    if (state & 1) {
        if (sha1) {
            h[0] = 0x67452301; h[1] = 0xefcdab89; h[2] = 0x98badcfe;
            h[3] = 0x10325476; h[4] = 0xc3d2e1f0;
        } else {
            memcpy(h, sha256_iv, sizeof(h));
        }
    } else {
        for (unsigned i = 0; i < words; i++) { h[i] = ldl_be_p(context + i * 4); }
        count = ldq_be_p(context + words * 4);
    }
    count += length;
    while (length >= 64) {
        caam_sha_block(h, data, sha1);
        data += 64;
        length -= 64;
    }
    if (state & 2) {
        uint8_t tail[128] = {0};
        unsigned size = length < 56 ? 64 : 128;
        memcpy(tail, data, length);
        tail[length] = 0x80;
        stq_be_p(tail + size - 8, count * 8);
        caam_sha_block(h, tail, sha1);
        if (size == 128) { caam_sha_block(h, tail + 64, sha1); }
    } else if (length) {
        return false; /* Streaming updates must end at a block boundary. */
    }
    for (unsigned i = 0; i < words; i++) { stl_be_p(context + i * 4, h[i]); }
    stq_be_p(context + words * 4, count);
    return true;
}

typedef struct CAAMChunk {
    hwaddr address;
    unsigned length;
    bool sg;
} CAAMChunk;

static bool caam_input(CAAMChunk *chunks, unsigned count, uint8_t *bytes)
{
    for (unsigned i = 0; i < count; i++) {
        if (!caam_dma(chunks[i].address, bytes, chunks[i].length, chunks[i].sg, false)) {
            return false;
        }
        bytes += chunks[i].length;
    }
    return true;
}

static unsigned caam_black_size(unsigned size, bool ccm)
{
    return ccm ? ROUND_UP(size, 8) + 12 : ROUND_UP(size, 16);
}

/* Black keys are volatile, authenticated job-key wrappers. Blob export below
 * provides persistence independently of the current job-key encryption key. */
static bool caam_black(IMXCAAMState *s, bool encrypt, bool ccm,
                        uint8_t *black, uint8_t *plain, unsigned size)
{
    unsigned padded = ROUND_UP(size, ccm ? 8 : 16);
    g_autofree uint8_t *data = g_malloc0(padded);
    if (encrypt) { memcpy(data, plain, size); }
    int ret;
    if (ccm) {
        uint8_t nonce[13] = {0};
        gnutls_datum_t key = {s->jdkey, sizeof(s->jdkey)};
        gnutls_aead_cipher_hd_t cipher;
        if (gnutls_aead_cipher_init(&cipher, GNUTLS_CIPHER_AES_128_CCM, &key) < 0) {
            return false;
        }
        if (encrypt) { qemu_guest_getrandom_nofail(black, 6); }
        memcpy(nonce, black, 6);
        size_t result = encrypt ? padded + 6 : padded;
        ret = encrypt ?
            gnutls_aead_cipher_encrypt(cipher, nonce, sizeof(nonce), NULL, 0, 6,
                                       data, padded, black + 6, &result) :
            gnutls_aead_cipher_decrypt(cipher, nonce, sizeof(nonce), NULL, 0, 6,
                                       black + 6, padded + 6, data, &result);
        gnutls_aead_cipher_deinit(cipher);
    } else {
        g_autoptr(QCryptoCipher) cipher = qcrypto_cipher_new(
            QCRYPTO_CIPHER_ALGO_AES_128, QCRYPTO_CIPHER_MODE_ECB,
            s->jdkey, sizeof(s->jdkey), NULL);
        if (!cipher) { return false; }
        ret = encrypt ? qcrypto_cipher_encrypt(cipher, data, black, padded, NULL) :
                        qcrypto_cipher_decrypt(cipher, black, data, padded, NULL);
    }
    if (ret < 0) { return false; }
    if (!encrypt) { memcpy(plain, data, size); }
    return true;
}

static bool caam_mpi(uint8_t *dest, unsigned size, const gnutls_datum_t *mpi)
{
    unsigned skip = mpi->size > size ? mpi->size - size : 0;
    for (unsigned i = 0; i < skip; i++) {
        if (mpi->data[i]) { return false; }
    }
    memset(dest, 0, size);
    memcpy(dest + size - (mpi->size - skip), mpi->data + skip, mpi->size - skip);
    return true;
}

static uint32_t caam_ecc_keygen(IMXCAAMState *s, uint32_t *desc,
                                 unsigned length, unsigned start, bool wide)
{
    static const gnutls_ecc_curve_t curves[] = {
        GNUTLS_ECC_CURVE_SECP192R1, GNUTLS_ECC_CURVE_SECP224R1,
        GNUTLS_ECC_CURVE_SECP256R1, GNUTLS_ECC_CURVE_SECP384R1,
        GNUTLS_ECC_CURVE_SECP521R1,
    };
    static const unsigned sizes[] = {24, 28, 32, 48, 66};
    unsigned curve = (desc[1] >> 7) & 0x3f, index = 2;
    uint32_t op = desc[start];
    if (curve >= ARRAY_SIZE(curves) || start != 4 + 2 * wide ||
        start + 1 != length || !(desc[1] & BIT(25))) {
        return 0x40000004;
    }
    hwaddr secret = caam_pointer(desc, &index, wide);
    hwaddr public = caam_pointer(desc, &index, wide);
    gnutls_x509_privkey_t key;
    gnutls_datum_t x = {0}, y = {0}, d = {0};
    gnutls_ecc_curve_t selected;
    uint8_t plain[66], xy[132], black[84];
    uint32_t status = 0x40000004;
    unsigned size = sizes[curve];
    if (gnutls_x509_privkey_init(&key) < 0) { return status; }
    if (gnutls_x509_privkey_generate(key, GNUTLS_PK_ECDSA,
            GNUTLS_CURVE_TO_BITS(curves[curve]), 0) < 0 ||
        gnutls_x509_privkey_export_ecc_raw(key, &selected, &x, &y, &d) < 0 ||
        !caam_mpi(plain, size, &d) || !caam_mpi(xy, size, &x) ||
        !caam_mpi(xy + size, size, &y)) {
        goto out;
    }
    if (op & BIT(2)) {
        bool ccm = op & BIT(4);
        if (!caam_black(s, true, ccm, black, plain, size)) { goto out; }
        cpu_physical_memory_write(secret, black, caam_black_size(size, ccm));
    } else {
        cpu_physical_memory_write(secret, plain, size);
    }
    cpu_physical_memory_write(public, xy, size * 2);
    status = 0;
out:
    gnutls_free(x.data); gnutls_free(y.data); gnutls_free(d.data);
    gnutls_x509_privkey_deinit(key);
    return status;
}

static uint32_t caam_mp_key(IMXCAAMState *s, uint32_t *desc,
                             unsigned length, unsigned start, bool wide)
{
    static const gnutls_ecc_curve_t curves[] = {
        GNUTLS_ECC_CURVE_SECP256R1, GNUTLS_ECC_CURVE_SECP384R1,
        GNUTLS_ECC_CURVE_SECP521R1,
    };
    static const unsigned sizes[] = {32, 48, 66};
    static const QCryptoHashAlgo hashes[] = {
        QCRYPTO_HASH_ALGO_SHA256, QCRYPTO_HASH_ALGO_SHA384,
        QCRYPTO_HASH_ALGO_SHA512,
    };
    unsigned curve = (desc[1] >> 17) & 15, index = 2;
    uint32_t op = desc[start];
    if (curve < 3 || curve > 5 || start != 4 + wide || length != start + 1) {
        return 0x40000004;
    }
    hwaddr pointer = caam_pointer(desc, &index, wide);
    unsigned count = desc[index], size = sizes[curve - 3];
    if ((op >> 24 & 7) == 7) {
        /* Emulated MP derivation from the unfused zero OTPMK and passphrase.
         * Keep the resulting scalar stable across resets and regeneration. */
        if (!count || count > 4096) { return 0x40000004; }
        g_autofree uint8_t *material = g_malloc0(32 + count);
        g_autofree uint8_t *digest = NULL;
        size_t digest_length = 0;
        cpu_physical_memory_read(pointer, material + 32, count);
        if (qcrypto_hash_bytes(hashes[curve - 3], material, 32 + count,
                                &digest, &digest_length, NULL) < 0) {
            return 0x40000004;
        }
        /* A positive scalar strictly below the selected curve's order. */
        digest[0] &= 0x7f;
        digest[digest_length - 1] |= 1;
        memset(s->mpkey, 0, sizeof(s->mpkey));
        memcpy(s->mpkey + size - digest_length, digest, digest_length);
        s->mpcurve = curve;
        s->registers[0xc / 4] = (s->registers[0xc / 4] & 0x0fffffff) |
                                curve << 28;
        return 0;
    }
    if ((op >> 24 & 7) != 6 || s->mpcurve != curve || count < size * 2) {
        return 0x40000004;
    }
    gnutls_x509_privkey_t key;
    gnutls_datum_t d = {s->mpkey, size}, x = {0}, y = {0};
    gnutls_ecc_curve_t selected;
    uint8_t xy[132];
    uint32_t status = 0x40000004;
    if (gnutls_x509_privkey_init(&key) < 0) { return status; }
    if (gnutls_x509_privkey_import_ecc_raw(key, curves[curve - 3],
                                          NULL, NULL, &d) >= 0 &&
        gnutls_x509_privkey_export_ecc_raw(key, &selected, &x, &y, NULL) >= 0 &&
        caam_mpi(xy, size, &x) && caam_mpi(xy + size, size, &y) &&
        caam_dma(pointer, xy, size * 2, desc[1] & BIT(31), true)) {
        status = 0;
    }
    gnutls_free(x.data); gnutls_free(y.data);
    gnutls_x509_privkey_deinit(key);
    return status;
}

static uint32_t caam_blob(IMXCAAMState *s, uint32_t op,
                           const uint8_t *modifier, unsigned modifier_length,
                           CAAMChunk *chunks, unsigned input_chunks,
                           unsigned input_length, hwaddr output,
                           unsigned output_length, bool output_sg)
{
    bool encapsulate = ((op >> 24) & 7) == 7, ccm = op & BIT(8);
    unsigned size = encapsulate ? input_length : output_length;
    if (!(op & BIT(2)) || size > 4096 || !size || modifier_length != 16 ||
        (encapsulate ? output_length != size + 48 : input_length != size + 48)) {
        return 0x2000000a;
    }
    unsigned black_size = caam_black_size(size, ccm);
    g_autofree uint8_t *black = g_malloc(black_size);
    g_autofree uint8_t *plain = g_malloc(size);
    g_autofree uint8_t *blob = g_malloc(size + 48);
    uint8_t material[48] = {0};
    memcpy(material + 32, modifier, 16);
    g_autofree uint8_t *derived = NULL;
    size_t derived_length = 0;
    if (qcrypto_hash_bytes(QCRYPTO_HASH_ALGO_SHA256, material, sizeof(material),
                           &derived, &derived_length, NULL) < 0) {
        return 0x40000004;
    }
    gnutls_datum_t key = {derived, derived_length};
    gnutls_aead_cipher_hd_t cipher;
    if (gnutls_aead_cipher_init(&cipher, GNUTLS_CIPHER_AES_256_GCM, &key) < 0) {
        return 0x40000004;
    }
    uint32_t status = 0x2000000a;
    if (encapsulate) {
        if (input_chunks != 1 || !caam_dma(chunks[0].address, black, black_size,
                                           chunks[0].sg, false) ||
            !caam_black(s, false, ccm, black, plain, size)) { goto out; }
        qemu_guest_getrandom_nofail(blob, 32);
        size_t count = size + 16;
        if (gnutls_aead_cipher_encrypt(cipher, blob, 12, blob, 32, 16,
                                       plain, size, blob + 32, &count) < 0 ||
            !caam_dma(output, blob, size + 48, output_sg, true)) { goto out; }
    } else {
        size_t count = size;
        if (!caam_input(chunks, input_chunks, blob) ||
            gnutls_aead_cipher_decrypt(cipher, blob, 12, blob, 32, 16,
                                       blob + 32, size + 16, plain, &count) < 0 ||
            !caam_black(s, true, ccm, black, plain, size) ||
            !caam_dma(output, black, black_size, output_sg, true)) { goto out; }
    }
    status = 0;
out:
    gnutls_aead_cipher_deinit(cipher);
    return status;
}

static uint32_t caam_execute(IMXCAAMState *s, hwaddr address)
{
    uint32_t desc[128], op = 0;
    hwaddr output = 0;
    hwaddr saved_context = 0, nonce = 0, aad = 0;
    uint8_t key[64] = {0}, context[64] = {0}, hash_context[64] = {0};
    uint8_t modifier[64] = {0};
    unsigned modifier_length = 0;
    CAAMChunk chunks[32];
    unsigned input_chunks = 0;
    unsigned key_length = 0, context_length = 0, context_offset = 0;
    unsigned nonce_length = 0, aad_length = 0;
    unsigned input_length = 0, output_length = 0;
    bool output_sg = false, context_sg = false;
    bool nonce_sg = false, aad_sg = false;
    bool wide = s->registers[4 / 4] & BIT(16);
    unsigned length = caam_ld32(address) & 0x7f;
    if (!length || length > 64) {
        return 0x40000004; /* invalid descriptor length */
    }
    for (unsigned i = 0; i < length; i++) {
        desc[i] = caam_ld32(address + 4 * i);
    }
    bool shared = desc[0] & BIT(12);
    if (shared) {
        unsigned index = 1;
        if (index + wide >= length) {
            return 0x40000004;
        }
        hwaddr pointer = caam_pointer(desc, &index, wide);
        uint32_t header = caam_ld32(pointer);
        unsigned words = header & 0x3f;
        if ((header >> 27) != 0x17 || !words || index > length) {
            return 0x40000004;
        }
        /* Job sequence pointers are established before the shared program. */
        unsigned parameters = length - index;
        memmove(desc + 1, desc + index, parameters * 4);
        for (unsigned i = 1; i < words; i++) {
            desc[parameters + i] = caam_ld32(pointer + i * 4);
        }
        length = parameters + words;
    }
    unsigned start = shared ? 1 : MAX(1, (desc[0] >> 16) & 0x3f);
    if (!shared && start < length &&
        (desc[start] & 0xfeffffff) == 0x86140000) {
        return caam_mp_key(s, desc, length, start, wide);
    }
    if (!shared && start < length && (desc[start] & 0xffff0002) == 0x80140002) {
        return caam_ecc_keygen(s, desc, length, start, wide);
    }
    for (unsigned i = start; i < length;) {
        uint32_t command = desc[i++];
        unsigned type = command >> 27;
        switch (type) {
        case 0x03: /* imported hash/cipher context needs streaming state */
            return 0x40000004;
        case 0x05: /* sequence data uses the job's input pointer */
        case 0x0b: /* sequence result uses its output pointer */
        case 0x15: /* VARSEQINLEN = SEQINLEN, no immediate operands */
            break;
        case 0x00: /* key pointer / immediate key */
        case 0x02: { /* context load */
            if (type == 0 && ((command >> 25) & 3) == 2) {
                return 0x40000004; /* Class-2 keyed hashing is unsupported. */
            }
            unsigned count = command & (type == 0 ? 0x3ff : 0xff);
            bool cipher_key = type == 0 && ((command >> 25) & 3) == 1;
            bool cipher_iv = type == 2 && ((command >> 25) & 3) == 1 &&
                             ((command >> 16) & 0x7f) == 0x20;
            bool hash_state = type == 2 && ((command >> 25) & 3) == 2 &&
                              ((command >> 16) & 0x7f) == 0x20;
            bool blob_modifier = type == 2 && ((command >> 25) & 3) == 2 &&
                                 ((command >> 16) & 0x7f) == 0x40;
            uint8_t bytes[64] = {0};
            if ((cipher_key || cipher_iv || hash_state || blob_modifier) && count > sizeof(bytes)) {
                return 0x40000004;
            }
            if (command & BIT(23)) {
                if (i + (count + 3) / 4 > length) { return 0x40000004; }
                for (unsigned j = 0; j < count && j < sizeof(bytes); j++) {
                    bytes[j] = desc[i + j / 4] >> ((j % 4) * 8);
                }
                i += (count + 3) / 4;
            } else if (i + wide < length) {
                hwaddr pointer = caam_pointer(desc, &i, wide);
                if (cipher_key || cipher_iv || hash_state || blob_modifier) {
                    cpu_physical_memory_read(pointer, bytes, count);
                }
            } else {
                return 0x40000004;
            }
            if (cipher_key) {
                if (command & BIT(22)) { return 0x40000004; }
                key_length = count;
                memcpy(key, bytes, count);
            } else if (cipher_iv) {
                unsigned offset = (command >> 8) & 255;
                if (offset > 64 || count > 64 - offset) { return 0x40000004; }
                memcpy(context + offset, bytes, count);
            } else if (hash_state) {
                unsigned offset = (command >> 8) & 255;
                if (offset > 64 || count > 64 - offset) { return 0x40000004; }
                memcpy(hash_context + offset, bytes, count);
            } else if (blob_modifier) {
                memcpy(modifier, bytes, count);
                modifier_length = count;
            }
            break;
        }
        case 0x04: case 0x1e: {
            unsigned count = command & 0xffff;
            hwaddr pointer = 0;
            if (command & BIT(23)) {
                if (count) { return 0x40000004; }
            } else {
                if (i + wide >= length) { return 0x40000004; }
                pointer = caam_pointer(desc, &i, wide);
            }
            if (command & BIT(22)) {
                if (i == length) { return 0x40000004; }
                count = desc[i++];
            }
            unsigned datatype = type == 4 ? (command >> 16) & 0x38 : 0x10;
            if (datatype == 0x20) {
                nonce = pointer;
                nonce_length = count;
                nonce_sg = command & BIT(24);
            } else if (datatype == 0x30) {
                aad = pointer;
                aad_length = count;
                aad_sg = command & BIT(24);
            } else {
                if (type == 0x1e) {
                    input_length = 0;
                    input_chunks = 0;
                }
                if (input_chunks == ARRAY_SIZE(chunks) ||
                    count > 64 * 1024 * 1024 - input_length) {
                    return 0x40000004;
                }
                chunks[input_chunks++] = (CAAMChunk){pointer, count, command & BIT(24)};
                input_length += count;
            }
            break;
        }
        case 0x0a: case 0x0c: case 0x1f: {
            if (i + wide >= length) { return 0x40000004; }
            hwaddr destination = caam_pointer(desc, &i, wide);
            unsigned count = type == 0x0a ? command & 0xff : command & 0xffff;
            if (command & BIT(22)) {
                if (i == length) { return 0x40000004; }
                count = desc[i++];
            }
            if (type == 0x0a && ((command >> 25) & 3) == 1 &&
                ((command >> 16) & 0x7f) == 0x20) {
                context_offset = (command >> 8) & 255;
                if (context_offset > 64 || count > 64 - context_offset) {
                    return 0x40000004;
                }
                saved_context = destination;
                context_length = count;
                context_sg = command & BIT(24);
            } else {
                output = destination;
                output_length = count;
                output_sg = command & BIT(24);
            }
            break;
        }
        case 0x10: op = command; break;
        case 0x14: break; /* local completion barrier */
        default:
            qemu_log_mask(LOG_UNIMP, "CAAM descriptor command %#x\n", command);
            return 0x40000004;
        }
        if (i > length) { return 0x40000004; }
    }
    if ((op & 0x00ffffff) == 0x0d0002 && !input_length && output_length == 32) {
        /* Stable master-key verification value for the unfused zero OTPMK. */
        static const uint8_t otpmk[32];
        g_autofree uint8_t *digest = NULL;
        size_t digest_length = 0;
        if (qcrypto_hash_bytes(QCRYPTO_HASH_ALGO_SHA256, otpmk, sizeof(otpmk),
                                &digest, &digest_length, NULL) < 0) {
            return 0x40000004;
        }
        if (!caam_dma(output, digest, digest_length, output_sg, true)) { return 0x40000004; }
        return 0;
    }
    if ((op & 0xff0000) == 0x0d0000) {
        return caam_blob(s, op, modifier, modifier_length, chunks, input_chunks,
                          input_length, output, output_length, output_sg);
    }
    if (((op >> 16) & 0xff) == 0x50) {
        if (output_length > 1024 * 1024) { return 0x40000004; }
        if (output && output_length) {
            g_autofree uint8_t *random = g_malloc(output_length);
            qemu_guest_getrandom_nofail(random, output_length);
            if (!caam_dma(output, random, output_length, output_sg, true)) { return 0x40000004; }
        }
        s->registers[0x6c0 / 4] = 0x40000033;
        return 0;
    }
    unsigned algorithm = (op >> 16) & 0xff;
    if (algorithm == 0x10) {
        if ((key_length != 16 && key_length != 24 && key_length != 32) ||
            input_length != output_length ||
            (input_length && !output) ||
            input_length > 16 * 1024 * 1024 ||
            nonce_length > 1024 || aad_length > 16 * 1024 * 1024) {
            return 0x40000004;
        }
        QCryptoCipherMode mode;
        switch (op & 0x1fe0) {
        case 0: mode = QCRYPTO_CIPHER_MODE_CTR; break;
        case 0x200: mode = QCRYPTO_CIPHER_MODE_CBC; break;
        case 0x400: mode = QCRYPTO_CIPHER_MODE_ECB; break;
        case 0x900: mode = QCRYPTO_CIPHER_MODE_ECB; break;
        default: return 0x40000004;
        }
        QCryptoCipherAlgo alg = key_length == 16 ? QCRYPTO_CIPHER_ALGO_AES_128 :
                               key_length == 24 ? QCRYPTO_CIPHER_ALGO_AES_192 :
                               QCRYPTO_CIPHER_ALGO_AES_256;
        g_autoptr(QCryptoCipher) cipher = qcrypto_cipher_new(alg, mode, key, key_length, NULL);
        if (!cipher || (mode != QCRYPTO_CIPHER_MODE_ECB &&
            qcrypto_cipher_setiv(cipher, context, 16, NULL) < 0)) {
            return 0x40000004;
        }
        g_autofree uint8_t *plain = g_malloc(input_length);
        g_autofree uint8_t *result = g_malloc(output_length);
        if (!caam_input(chunks, input_chunks, plain)) { return 0x40000004; }
        int error;
        if ((op & 0x1fe0) == 0x900) {
            g_autofree uint8_t *nonce_data = g_malloc(nonce_length);
            g_autofree uint8_t *aad_data = g_malloc(aad_length);
            if (!caam_dma(nonce, nonce_data, nonce_length, nonce_sg, false) ||
                !caam_dma(aad, aad_data, aad_length, aad_sg, false)) {
                return 0x40000004;
            }
            error = caam_gcm(cipher, op, context, nonce_data, nonce_length,
                             aad_data, aad_length, plain, result, input_length) ? 0 : -1;
        } else {
            error = (op & 1) ?
                qcrypto_cipher_encrypt(cipher, plain, result, input_length, NULL) :
                qcrypto_cipher_decrypt(cipher, plain, result, input_length, NULL);
        }
        if (error < 0) { return 0x40000004; }
        if (!caam_dma(output, result, output_length, output_sg, true)) { return 0x40000004; }
        if (mode == QCRYPTO_CIPHER_MODE_CBC && input_length >= 16) {
            memcpy(context, ((op & 1) ? result : plain) + input_length - 16, 16);
        }
        if (saved_context && !caam_dma(saved_context, context + context_offset,
                                       context_length, context_sg, true)) {
            return 0x40000004;
        }
        return 0;
    }
    if (algorithm == 0x41 || algorithm == 0x43) {
        if ((op & 0x1ff0) || input_length > 64 * 1024 * 1024) {
            return 0x40000004;
        }
        g_autofree uint8_t *data = g_malloc(input_length);
        if (!caam_input(chunks, input_chunks, data) ||
            !caam_sha(op, hash_context, data, input_length)) {
            return 0x40000004;
        }
        if (output_length > sizeof(hash_context) ||
            !caam_dma(output, hash_context, output_length, output_sg, true)) {
            return 0x40000004;
        }
        return 0;
    }
    qemu_log_mask(LOG_UNIMP, "CAAM operation %#x\n", op);
    return 0x40000004;
}

static void caam_irq(IMXCAAMState *s, unsigned ring)
{
    uint32_t *regs = s->registers + (ring + 1) * 0x1000 / 4;
    qemu_set_irq(s->irq[ring], (regs[0x4c / 4] & 1) && !(regs[0x54 / 4] & 1));
}
static uint64_t caam_read(void *opaque, hwaddr offset, unsigned size)
{
    IMXCAAMState *s = opaque;
    if (offset >= 0x1000 && offset < 0x5000) {
        unsigned ring = offset / 0x1000 - 1;
        uint32_t *regs = s->registers + (ring + 1) * 0x1000 / 4;
        switch (offset & 0xfff) {
        case 0x14: return regs[0xc / 4] - s->pending[ring];
        case 0x3c: return s->pending[ring];
        /* Hardware indices are byte offsets into the DMA rings. */
        case 0x5c:
            return s->input_index[ring] * (s->registers[1] & BIT(16) ? 8 : 4);
        case 0x64:
            return s->output_index[ring] * (s->registers[1] & BIT(16) ? 12 : 8);
        case 0x6c: return 0;
        }
        if ((offset & 0xfff) >= 0xf00) {
            return s->registers[(offset & 0xfff) / 4];
        }
    }
    return s->registers[offset / 4];
}
static void caam_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IMXCAAMState *s = opaque;
    if (offset >= 0x1000 && offset < 0x5000) {
        unsigned ring = offset / 0x1000 - 1;
        uint32_t *regs = s->registers + (ring + 1) * 0x1000 / 4;
        bool wide = s->registers[1] & BIT(16);
        switch (offset & 0xfff) {
        case 0x6c:
            /* Flush/park preserves indices. A second RESET while halted
             * resets them; resume through JRINT must preserve them too. */
            if ((value & 1) && (regs[0x4c / 4] & 0xc) == 8) {
                s->input_index[ring] = s->output_index[ring] = s->pending[ring] = 0;
                regs[0x4c / 4] = 0;
            } else if (value & 3) {
                regs[0x4c / 4] = (regs[0x4c / 4] & ~0xc) | 8;
            }
            caam_irq(s, ring);
            return;
        case 0x4c:
            regs[0x4c / 4] &= ~value;
            caam_irq(s, ring);
            return;
        case 0x34:
            s->pending[ring] -= MIN(value, s->pending[ring]);
            return;
        case 0x1c:
            for (unsigned j = 0; j < MIN(value, 64); j++) {
                if (!regs[3] || !regs[0x2c / 4] || s->pending[ring] >= regs[0x2c / 4]) { break; }
                hwaddr inbase = (uint64_t)regs[0] << 32 | regs[1];
                hwaddr outbase = (uint64_t)regs[8] << 32 | regs[9];
                hwaddr in = inbase + s->input_index[ring] * (wide ? 8 : 4);
                uint64_t desc = caam_ld32(in);
                if (wide) { desc |= (uint64_t)caam_ld32(in + 4) << 32; }
                uint32_t status = caam_execute(s, desc);
                uint8_t result[12];
                if (wide) { stq_le_p(result, desc); } else { stl_le_p(result, desc); }
                stl_le_p(result + (wide ? 8 : 4), status);
                cpu_physical_memory_write(outbase + s->output_index[ring] * (wide ? 12 : 8),
                                           result, wide ? 12 : 8);
                s->input_index[ring] = (s->input_index[ring] + 1) % regs[3];
                s->output_index[ring] = (s->output_index[ring] + 1) % regs[0x2c / 4];
                s->pending[ring]++;
                regs[0x4c / 4] |= 1;
            }
            caam_irq(s, ring);
            return;
        }
    }
    s->registers[offset / 4] = value;
}
static const MemoryRegionOps caam_ops = {
    .read = caam_read, .write = caam_write, .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static void caam_reset(DeviceState *dev)
{
    IMXCAAMState *s = IMX_CAAM(dev);
    memset(s->registers, 0, sizeof(s->registers));
    memset(s->input_index, 0, sizeof(s->input_index));
    memset(s->output_index, 0, sizeof(s->output_index));
    memset(s->pending, 0, sizeof(s->pending));
    memset(s->mpkey, 0, sizeof(s->mpkey));
    s->mpcurve = 0;
    s->registers[0x6c0 / 4] = 0x40000033;
    s->registers[0xfa8 / 4] = 2 << 8; /* two RNG4 state handles */
    s->registers[0xfd4 / 4] = 6;      /* DECO and TRNG idle */
    s->registers[0xfd8 / 4] = (8 << 16) | (4 << 12);
    s->registers[0xfdc / 4] = 0x020105;
    s->registers[0xfe4 / 4] = 9 << 24;
    s->registers[0xfec / 4] = 4 << 16;
    s->registers[0xff0 / 4] = 0x31000000;
    s->registers[0xff4 / 4] = 0x10011011;
    qemu_guest_getrandom_nofail(s->jdkey, sizeof(s->jdkey));
    for (unsigned i = 0; i < 4; i++) { caam_irq(s, i); }
}
static const VMStateDescription caam_vmstate = {
    .name = TYPE_IMX_CAAM, .version_id = 1, .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(registers, IMXCAAMState, 0x10000 / 4),
        VMSTATE_UINT32_ARRAY(input_index, IMXCAAMState, 4),
        VMSTATE_UINT32_ARRAY(output_index, IMXCAAMState, 4),
        VMSTATE_UINT32_ARRAY(pending, IMXCAAMState, 4),
        VMSTATE_UINT8_ARRAY(jdkey, IMXCAAMState, 16),
        VMSTATE_UINT8_ARRAY(mpkey, IMXCAAMState, 66),
        VMSTATE_UINT8(mpcurve, IMXCAAMState),
        VMSTATE_END_OF_LIST()
    },
};
static void caam_init(Object *obj)
{
    IMXCAAMState *s = IMX_CAAM(obj);
    memory_region_init_io(&s->memory, obj, &caam_ops, s, TYPE_IMX_CAAM, 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->memory);
    for (unsigned i = 0; i < 4; i++) { sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]); }
}
static void caam_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->vmsd = &caam_vmstate;
    device_class_set_legacy_reset(dc, caam_reset);
}
static const TypeInfo caam_info = {
    .name = TYPE_IMX_CAAM, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IMXCAAMState), .instance_init = caam_init,
    .class_init = caam_class_init,
};
static void caam_register(void) { type_register_static(&caam_info); }
type_init(caam_register)
