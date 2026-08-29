/*
 * Copyright (C) 2014 - Linaro
 * Author: Rob Herring <rob.herring@linaro.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "helper.h"
#include "kvm-consts.h"
#include "qemu/main-loop.h"
#include "exec/cpu-common.h"
#include "system/runstate.h"
#include "internals.h"
#include "arm-powerctl.h"
#include "target/arm/multiprocessing.h"
#include "target/arm/trace.h"

#define MTK_SIP_VCOREFS_CONTROL         0x82000506
#define MTK_SIP_VCOREFS_INIT            0
#define MTK_SIP_VCOREFS_START           1
#define MTK_SIP_VCOREFS_GET_OPP_TYPE    2
#define MTK_SIP_VCOREFS_GET_FW_TYPE     3
#define MTK_SIP_VCOREFS_GET_VCORE_UV    4
#define MTK_SIP_VCOREFS_GET_DRAM_FREQ   5
#define MTK_SIP_VCOREFS_GET_FREQ_COUNT  7
#define MTK_SIP_VCOREFS_FB_ACTION       8
#define MTK_SIP_VCOREFS_RESUME          21
#define MTK_SIP_VCOREFS_QOS_MODE        32

/* OP-TEE SMC ABI used by the MT8113 firmware device tree. */
#define MTK_OPTEE_CALLS_COUNT            0xbf00ff00
#define MTK_OPTEE_CALLS_UID              0xbf00ff01
#define MTK_OPTEE_CALLS_REVISION         0xbf00ff03
#define MTK_OPTEE_GET_OS_UUID            0xb2000000
#define MTK_OPTEE_GET_OS_REVISION        0xb2000001
#define MTK_OPTEE_CALL_WITH_ARG          0x32000004
#define MTK_OPTEE_GET_SHM_CONFIG         0xb2000007
#define MTK_OPTEE_EXCHANGE_CAPABILITIES  0xb2000009
#define MTK_OPTEE_DISABLE_SHM_CACHE      0xb200000a
#define MTK_OPTEE_ENABLE_SHM_CACHE       0xb200000b

#define MTK_OPTEE_SHM_BASE               0x43400000ULL
#define MTK_OPTEE_SHM_SIZE               0x00100000ULL
#define MTK_OPTEE_RAM_BASE               0x40000000ULL
#define MTK_OPTEE_RAM_SIZE               0x40000000ULL
#define MTK_OPTEE_SESSION_FBE            1
#define MTK_OPTEE_SESSION_ENUM           2
#define MTK_OPTEE_SESSION_KREE_CONSOLE   3

#define MTK_OPTEE_RETURN_OK              0
#define MTK_OPTEE_RETURN_EBADADDR        4
#define MTK_OPTEE_RETURN_EBADCMD         5
#define MTK_OPTEE_RETURN_ENOTAVAIL       7

#define MTK_OPTEE_MSG_OPEN_SESSION       0
#define MTK_OPTEE_MSG_INVOKE_COMMAND     1
#define MTK_OPTEE_MSG_CLOSE_SESSION      2
#define MTK_OPTEE_MSG_CANCEL             3
#define MTK_OPTEE_MSG_REGISTER_SHM       4
#define MTK_OPTEE_MSG_UNREGISTER_SHM     5

#define MTK_OPTEE_ATTR_TYPE_MASK         0xff
#define MTK_OPTEE_ATTR_TMEM_INPUT        0x9
#define MTK_OPTEE_ATTR_TMEM_OUTPUT       0xa
#define MTK_OPTEE_ATTR_TMEM_INOUT        0xb
#define MTK_OPTEE_ATTR_RMEM_INPUT        0x5
#define MTK_OPTEE_ATTR_RMEM_OUTPUT       0x6
#define MTK_OPTEE_ATTR_RMEM_INOUT        0x7

#define MTK_TEEC_SUCCESS                 0
#define MTK_TEEC_ERROR_BAD_PARAMETERS    0xffff0006
#define MTK_TEEC_ERROR_ITEM_NOT_FOUND    0xffff0008
#define MTK_TEEC_ORIGIN_TEE              3
#define MTK_TEEC_ORIGIN_TRUSTED_APP      4

#define MTK_OPTEE_MSG_ARG_SIZE           32
#define MTK_OPTEE_MSG_PARAM_SIZE         32
#define MTK_OPTEE_MAX_PARAMS             8
#define MTK_OPTEE_MAX_KEY_SIZE           4096
#define MTK_OPTEE_MAX_REGISTRATIONS      32
#define MTK_OPTEE_PAGE_MASK              (~0xfffULL)

typedef struct MtkOpteeRegistration {
    bool used;
    uint64_t ref;
    uint64_t addr;
    uint64_t size;
} MtkOpteeRegistration;

static MtkOpteeRegistration mtk_optee_registrations[
    MTK_OPTEE_MAX_REGISTRATIONS];

static const uint8_t mtk_optee_fbe_uuid[16] = {
    0xa6, 0xf7, 0x22, 0xd3, 0xfa, 0xec, 0x49, 0x06,
    0xbf, 0xc4, 0xe8, 0x93, 0x83, 0x73, 0xfa, 0xab,
};

static const uint8_t mtk_optee_kree_console_uuid[16] = {
    0x91, 0x89, 0xc1, 0xcb, 0x92, 0x29, 0x51, 0x82,
    0x47, 0x32, 0xcf, 0x9a, 0x63, 0x84, 0x08, 0xc5,
};

static const uint8_t mtk_optee_enum_uuid[16] = {
    0x70, 0x11, 0xa6, 0x88, 0xdd, 0xde, 0x40, 0x53,
    0xa5, 0xa9, 0x7b, 0x3c, 0x4d, 0xdf, 0x13, 0xb8,
};

static bool mtk_optee_ram_range(uint64_t addr, uint64_t size)
{
    return addr >= MTK_OPTEE_RAM_BASE &&
           size <= MTK_OPTEE_RAM_SIZE &&
           addr - MTK_OPTEE_RAM_BASE <= MTK_OPTEE_RAM_SIZE - size;
}

static bool mtk_optee_read(uint64_t addr, void *buf, size_t size)
{
    hwaddr mapped_size = size;
    void *mapped;

    if (!mtk_optee_ram_range(addr, size)) {
        return false;
    }
    mapped = cpu_physical_memory_map(addr, &mapped_size, false);
    if (!mapped || mapped_size != size) {
        if (mapped) {
            cpu_physical_memory_unmap(mapped, mapped_size, false, 0);
        }
        return false;
    }
    memcpy(buf, mapped, size);
    cpu_physical_memory_unmap(mapped, mapped_size, false, 0);
    return true;
}

static bool mtk_optee_write(uint64_t addr, const void *buf, size_t size)
{
    hwaddr mapped_size = size;
    void *mapped;

    if (!mtk_optee_ram_range(addr, size)) {
        return false;
    }
    mapped = cpu_physical_memory_map(addr, &mapped_size, true);
    if (!mapped || mapped_size != size) {
        if (mapped) {
            cpu_physical_memory_unmap(mapped, mapped_size, true, 0);
        }
        return false;
    }
    memcpy(mapped, buf, size);
    cpu_physical_memory_unmap(mapped, mapped_size, true, size);
    return true;
}

static MtkOpteeRegistration *mtk_optee_find_registration(uint64_t ref)
{
    for (int n = 0; n < ARRAY_SIZE(mtk_optee_registrations); n++) {
        if (mtk_optee_registrations[n].used &&
            mtk_optee_registrations[n].ref == ref) {
            return &mtk_optee_registrations[n];
        }
    }
    return NULL;
}

static bool mtk_optee_register_shm(const uint8_t *param)
{
    uint64_t page_list = ldq_le_p(param + 8);
    uint64_t size = ldq_le_p(param + 16);
    uint64_t ref = ldq_le_p(param + 24);
    uint64_t page_addr;
    MtkOpteeRegistration *reg = mtk_optee_find_registration(ref);

    if (!size || !ref ||
        !mtk_optee_read(page_list & MTK_OPTEE_PAGE_MASK,
                        &page_addr, sizeof(page_addr))) {
        return false;
    }
    page_addr = le64_to_cpu(page_addr) & MTK_OPTEE_PAGE_MASK;
    page_addr += page_list & ~MTK_OPTEE_PAGE_MASK;
    if (!mtk_optee_ram_range(page_addr, size)) {
        return false;
    }

    if (!reg) {
        for (int n = 0; n < ARRAY_SIZE(mtk_optee_registrations); n++) {
            if (!mtk_optee_registrations[n].used) {
                reg = &mtk_optee_registrations[n];
                break;
            }
        }
    }
    if (!reg) {
        return false;
    }
    *reg = (MtkOpteeRegistration) {
        .used = true,
        .ref = ref,
        .addr = page_addr,
        .size = size,
    };
    return true;
}

static void mtk_optee_unregister_shm(const uint8_t *param)
{
    MtkOpteeRegistration *reg =
        mtk_optee_find_registration(ldq_le_p(param + 24));

    if (reg) {
        reg->used = false;
    }
}

static bool mtk_optee_resolve_memref(const uint8_t *param, uint64_t *addr,
                                     uint64_t *size)
{
    uint32_t type = ldq_le_p(param) & MTK_OPTEE_ATTR_TYPE_MASK;

    *size = ldq_le_p(param + 16);
    if (type == MTK_OPTEE_ATTR_TMEM_INPUT ||
        type == MTK_OPTEE_ATTR_TMEM_OUTPUT ||
        type == MTK_OPTEE_ATTR_TMEM_INOUT) {
        *addr = ldq_le_p(param + 8);
        return mtk_optee_ram_range(*addr, *size);
    }
    if (type == MTK_OPTEE_ATTR_RMEM_INPUT ||
        type == MTK_OPTEE_ATTR_RMEM_OUTPUT ||
        type == MTK_OPTEE_ATTR_RMEM_INOUT) {
        uint64_t offset = ldq_le_p(param + 8);
        MtkOpteeRegistration *reg =
            mtk_optee_find_registration(ldq_le_p(param + 24));

        if (!reg || offset > reg->size || *size > reg->size - offset) {
            return false;
        }
        *addr = reg->addr + offset;
        return mtk_optee_ram_range(*addr, *size);
    }
    return false;
}

static bool mtk_optee_mix_mem(uint64_t *hash, uint64_t addr, uint64_t size,
                              uint32_t index)
{
    uint8_t data[256];

    *hash ^= index;
    *hash *= 1099511628211ULL;
    for (int n = 0; n < 8; n++) {
        *hash ^= size >> (n * 8);
        *hash *= 1099511628211ULL;
    }

    while (size) {
        size_t chunk = MIN(size, sizeof(data));

        if (!mtk_optee_read(addr, data, chunk)) {
            return false;
        }
        for (size_t n = 0; n < chunk; n++) {
            *hash ^= data[n];
            *hash *= 1099511628211ULL;
        }
        addr += chunk;
        size -= chunk;
    }
    return true;
}

static bool mtk_optee_fbe_get_key(uint8_t *msg, uint32_t num_params)
{
    uint64_t hash = 1469598103934665603ULL;
    uint8_t *output = NULL;
    uint64_t output_addr = 0;
    uint64_t output_size = 0;

    for (uint32_t n = 0; n < num_params; n++) {
        uint8_t *param = msg + MTK_OPTEE_MSG_ARG_SIZE +
                         n * MTK_OPTEE_MSG_PARAM_SIZE;
        uint32_t type = ldq_le_p(param) & MTK_OPTEE_ATTR_TYPE_MASK;
        uint64_t addr;
        uint64_t size;

        if (type == MTK_OPTEE_ATTR_TMEM_INPUT ||
            type == MTK_OPTEE_ATTR_TMEM_INOUT ||
            type == MTK_OPTEE_ATTR_RMEM_INPUT ||
            type == MTK_OPTEE_ATTR_RMEM_INOUT) {
            if (!mtk_optee_resolve_memref(param, &addr, &size)) {
                return false;
            }
            if (!mtk_optee_mix_mem(&hash, addr, size, n)) {
                return false;
            }
        }
        if (type == MTK_OPTEE_ATTR_TMEM_OUTPUT ||
            type == MTK_OPTEE_ATTR_TMEM_INOUT ||
            type == MTK_OPTEE_ATTR_RMEM_OUTPUT ||
            type == MTK_OPTEE_ATTR_RMEM_INOUT) {
            if (!mtk_optee_resolve_memref(param, &addr, &size)) {
                return false;
            }
            output = param;
            output_addr = addr;
            output_size = size;
        }
    }

    if (!output || !output_size || output_size > MTK_OPTEE_MAX_KEY_SIZE ||
        !mtk_optee_ram_range(output_addr, output_size)) {
        return false;
    }

    uint8_t *key = g_malloc(output_size);
    uint64_t state = hash ^ 0x4642452d4b455931ULL;

    for (uint64_t n = 0; n < output_size; n++) {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        key[n] = (state * 2685821657736338717ULL) >> 56;
    }
    bool ok = mtk_optee_write(output_addr, key, output_size);

    g_free(key);
    if (ok) {
        stq_le_p(output + 16, output_size);
    }
    return ok;
}

static uint32_t mtk_optee_call_with_arg(uint64_t addr)
{
    uint8_t header[MTK_OPTEE_MSG_ARG_SIZE];
    uint8_t *msg;
    uint32_t cmd;
    uint32_t num_params;
    size_t size;

    if (!mtk_optee_read(addr, header, sizeof(header))) {
        return MTK_OPTEE_RETURN_EBADADDR;
    }
    cmd = ldl_le_p(header);
    num_params = ldl_le_p(header + 28);
    if (num_params > MTK_OPTEE_MAX_PARAMS) {
        return MTK_OPTEE_RETURN_EBADCMD;
    }
    size = MTK_OPTEE_MSG_ARG_SIZE +
           num_params * MTK_OPTEE_MSG_PARAM_SIZE;
    msg = g_malloc(size);
    if (!mtk_optee_read(addr, msg, size)) {
        g_free(msg);
        return MTK_OPTEE_RETURN_EBADADDR;
    }

    stl_le_p(msg + 20, MTK_TEEC_SUCCESS);
    stl_le_p(msg + 24, MTK_TEEC_ORIGIN_TRUSTED_APP);
    switch (cmd) {
    case MTK_OPTEE_MSG_OPEN_SESSION:
        if (num_params < 2) {
            stl_le_p(msg + 20, MTK_TEEC_ERROR_ITEM_NOT_FOUND);
            stl_le_p(msg + 24, MTK_TEEC_ORIGIN_TEE);
        } else if (!memcmp(msg + MTK_OPTEE_MSG_ARG_SIZE + 8,
                           mtk_optee_fbe_uuid,
                           sizeof(mtk_optee_fbe_uuid))) {
            stl_le_p(msg + 8, MTK_OPTEE_SESSION_FBE);
        } else if (!memcmp(msg + MTK_OPTEE_MSG_ARG_SIZE + 8,
                           mtk_optee_enum_uuid,
                           sizeof(mtk_optee_enum_uuid))) {
            stl_le_p(msg + 8, MTK_OPTEE_SESSION_ENUM);
        } else if (!memcmp(msg + MTK_OPTEE_MSG_ARG_SIZE + 8,
                           mtk_optee_kree_console_uuid,
                           sizeof(mtk_optee_kree_console_uuid))) {
            stl_le_p(msg + 8, MTK_OPTEE_SESSION_KREE_CONSOLE);
        } else {
            stl_le_p(msg + 20, MTK_TEEC_ERROR_ITEM_NOT_FOUND);
            stl_le_p(msg + 24, MTK_TEEC_ORIGIN_TEE);
        }
        break;
    case MTK_OPTEE_MSG_INVOKE_COMMAND:
        if (ldl_le_p(msg + 8) == MTK_OPTEE_SESSION_ENUM &&
            ldl_le_p(msg + 4) <= 1) {
            if (num_params) {
                stq_le_p(msg + MTK_OPTEE_MSG_ARG_SIZE + 16, 0);
            }
        } else if (ldl_le_p(msg + 8) == MTK_OPTEE_SESSION_KREE_CONSOLE &&
                   ldl_le_p(msg + 4) == 0) {
            /* The guest supplied the registered console ring successfully. */
        } else if (ldl_le_p(msg + 8) != MTK_OPTEE_SESSION_FBE ||
                   ldl_le_p(msg + 4) != 1 ||
                   !mtk_optee_fbe_get_key(msg, num_params)) {
            stl_le_p(msg + 20, MTK_TEEC_ERROR_BAD_PARAMETERS);
        }
        break;
    case MTK_OPTEE_MSG_CLOSE_SESSION:
    case MTK_OPTEE_MSG_CANCEL:
        break;
    case MTK_OPTEE_MSG_REGISTER_SHM:
        if (num_params != 1 ||
            !mtk_optee_register_shm(msg + MTK_OPTEE_MSG_ARG_SIZE)) {
            stl_le_p(msg + 20, MTK_TEEC_ERROR_BAD_PARAMETERS);
        }
        break;
    case MTK_OPTEE_MSG_UNREGISTER_SHM:
        if (num_params == 1) {
            mtk_optee_unregister_shm(msg + MTK_OPTEE_MSG_ARG_SIZE);
        }
        break;
    default:
        g_free(msg);
        return MTK_OPTEE_RETURN_EBADCMD;
    }

    uint32_t msg_ret = ldl_le_p(msg + 20);
    bool ok = mtk_optee_write(addr, msg, size);
    uint32_t smc_ret = ok ? MTK_OPTEE_RETURN_OK :
                            MTK_OPTEE_RETURN_EBADADDR;

    trace_arm_mtk_optee_message(
        addr, cmd, num_params, msg_ret, smc_ret,
        num_params ? ldl_le_p(msg + MTK_OPTEE_MSG_ARG_SIZE + 8) : 0,
        num_params ? ldl_le_p(msg + MTK_OPTEE_MSG_ARG_SIZE + 12) : 0,
        num_params ? ldl_le_p(msg + MTK_OPTEE_MSG_ARG_SIZE + 16) : 0,
        num_params ? ldl_le_p(msg + MTK_OPTEE_MSG_ARG_SIZE + 20) : 0);
    g_free(msg);
    return smc_ret;
}

static void mtk_optee_set_results(CPUARMState *env, uint32_t a0, uint32_t a1,
                                  uint32_t a2, uint32_t a3)
{
    uint32_t result[] = { a0, a1, a2, a3 };

    for (int n = 0; n < ARRAY_SIZE(result); n++) {
        if (is_a64(env)) {
            env->xregs[n] = result[n];
        } else {
            env->regs[n] = result[n];
        }
    }
}

static bool arm_handle_mtk_optee(ARMCPU *cpu, uint64_t param[4])
{
    CPUARMState *env = &cpu->env;

    if (!cpu->mtk_optee_fbe) {
        return false;
    }

    switch (param[0]) {
    case MTK_OPTEE_CALLS_COUNT:
        mtk_optee_set_results(env, 8, 0, 0, 0);
        return true;
    case MTK_OPTEE_CALLS_UID:
        mtk_optee_set_results(env, 0x384fb3e0, 0xe7f811e3,
                              0xaf630002, 0xa5d5c51b);
        return true;
    case MTK_OPTEE_CALLS_REVISION:
        mtk_optee_set_results(env, 2, 0, 0, 0);
        return true;
    case MTK_OPTEE_GET_OS_UUID:
        mtk_optee_set_results(env, 0x486178e0, 0xe7f811e3,
                              0xbc5e0002, 0xa5d5c51b);
        return true;
    case MTK_OPTEE_GET_OS_REVISION:
        mtk_optee_set_results(env, 3, 19, 0, 0);
        return true;
    case MTK_OPTEE_GET_SHM_CONFIG:
        mtk_optee_set_results(env, MTK_OPTEE_RETURN_OK,
                              MTK_OPTEE_SHM_BASE, MTK_OPTEE_SHM_SIZE, 1);
        return true;
    case MTK_OPTEE_EXCHANGE_CAPABILITIES:
        mtk_optee_set_results(env, MTK_OPTEE_RETURN_OK, 7, 0, 0);
        return true;
    case MTK_OPTEE_DISABLE_SHM_CACHE:
        mtk_optee_set_results(env, MTK_OPTEE_RETURN_ENOTAVAIL, 0, 0, 0);
        return true;
    case MTK_OPTEE_ENABLE_SHM_CACHE:
        mtk_optee_set_results(env, MTK_OPTEE_RETURN_OK, 0, 0, 0);
        return true;
    case MTK_OPTEE_CALL_WITH_ARG:
        mtk_optee_set_results(env,
                              mtk_optee_call_with_arg(param[1] << 32 |
                                                      param[2]),
                              0, 0, 0);
        return true;
    default:
        return false;
    }
}

static int32_t arm_handle_mtk_vcorefs(uint64_t command, uint64_t index,
                                      uint32_t *result)
{
    static const uint32_t vcore_uv[] = {
        800000, 750000, 700000, 650000,
    };
    static const uint32_t dram_khz[] = {
        1200000, 800000, 400000,
    };

    *result = 0;
    switch (command) {
    case MTK_SIP_VCOREFS_INIT:
    case MTK_SIP_VCOREFS_START:
    case MTK_SIP_VCOREFS_GET_OPP_TYPE:
    case MTK_SIP_VCOREFS_GET_FW_TYPE:
    case MTK_SIP_VCOREFS_FB_ACTION:
    case MTK_SIP_VCOREFS_RESUME:
    case MTK_SIP_VCOREFS_QOS_MODE:
        return 0;
    case MTK_SIP_VCOREFS_GET_VCORE_UV:
        if (index >= ARRAY_SIZE(vcore_uv)) {
            return QEMU_PSCI_RET_INVALID_PARAMS;
        }
        *result = vcore_uv[index];
        return 0;
    case MTK_SIP_VCOREFS_GET_DRAM_FREQ:
        if (index >= ARRAY_SIZE(dram_khz)) {
            return QEMU_PSCI_RET_INVALID_PARAMS;
        }
        *result = dram_khz[index];
        return 0;
    case MTK_SIP_VCOREFS_GET_FREQ_COUNT:
        *result = ARRAY_SIZE(dram_khz);
        return 0;
    default:
        return QEMU_PSCI_RET_NOT_SUPPORTED;
    }
}

bool arm_is_psci_call(ARMCPU *cpu, int excp_type)
{
    /*
     * Return true if the exception type matches the configured PSCI conduit.
     * This is called before the SMC/HVC instruction is executed, to decide
     * whether we should treat it as a PSCI call or with the architecturally
     * defined behaviour for an SMC or HVC (which might be UNDEF or trap
     * to EL2 or to EL3).
     */

    switch (excp_type) {
    case EXCP_HVC:
        if (cpu->psci_conduit != QEMU_PSCI_CONDUIT_HVC) {
            return false;
        }
        break;
    case EXCP_SMC:
        if (cpu->psci_conduit != QEMU_PSCI_CONDUIT_SMC) {
            return false;
        }
        break;
    default:
        return false;
    }

    return true;
}

void arm_handle_psci_call(ARMCPU *cpu)
{
    /*
     * This function partially implements the logic for dispatching Power State
     * Coordination Interface (PSCI) calls (as described in ARM DEN 0022D.b),
     * to the extent required for bringing up and taking down secondary cores,
     * and for handling reset and poweroff requests.
     * Additional information about the calling convention used is available in
     * the document 'SMC Calling Convention' (ARM DEN 0028)
     */
    CPUARMState *env = &cpu->env;
    uint64_t param[4];
    uint64_t context_id, mpidr;
    uint64_t entry;
    uint32_t result = 0;
    int32_t ret = 0;
    int i;

    for (i = 0; i < 4; i++) {
        /*
         * All PSCI functions take explicit 32-bit or native int sized
         * arguments so we can simply zero-extend all arguments regardless
         * of which exact function we are about to call.
         */
        param[i] = is_a64(env) ? env->xregs[i] : env->regs[i];
    }
    trace_arm_psci_call(param[0], param[1], param[2], param[3],
                        arm_cpu_mp_affinity(cpu));

    if ((param[0] & QEMU_PSCI_0_2_64BIT) && !is_a64(env)) {
        ret = QEMU_PSCI_RET_NOT_SUPPORTED;
        goto err;
    }

    switch (param[0]) {
        CPUState *target_cpu_state;
        ARMCPU *target_cpu;

    case QEMU_PSCI_0_2_FN_PSCI_VERSION:
        ret = QEMU_PSCI_VERSION_1_1;
        break;
    case MTK_OPTEE_CALLS_COUNT:
    case MTK_OPTEE_CALLS_UID:
    case MTK_OPTEE_CALLS_REVISION:
    case MTK_OPTEE_GET_OS_UUID:
    case MTK_OPTEE_GET_OS_REVISION:
    case MTK_OPTEE_CALL_WITH_ARG:
    case MTK_OPTEE_GET_SHM_CONFIG:
    case MTK_OPTEE_EXCHANGE_CAPABILITIES:
    case MTK_OPTEE_DISABLE_SHM_CACHE:
    case MTK_OPTEE_ENABLE_SHM_CACHE:
        if (arm_handle_mtk_optee(cpu, param)) {
            return;
        }
        ret = QEMU_PSCI_RET_NOT_SUPPORTED;
        break;
    case MTK_SIP_VCOREFS_CONTROL:
        if (cpu->mtk_sip_vcorefs) {
            ret = arm_handle_mtk_vcorefs(param[1], param[2], &result);
            if (is_a64(env)) {
                env->xregs[1] = result;
            } else {
                env->regs[1] = result;
            }
        } else {
            ret = QEMU_PSCI_RET_NOT_SUPPORTED;
        }
        break;
    case QEMU_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
        ret = QEMU_PSCI_0_2_RET_TOS_MIGRATION_NOT_REQUIRED; /* No trusted OS */
        break;
    case QEMU_PSCI_0_2_FN_AFFINITY_INFO:
    case QEMU_PSCI_0_2_FN64_AFFINITY_INFO:
        mpidr = param[1];

        switch (param[2]) {
        case 0:
            target_cpu_state = arm_get_cpu_by_id(mpidr);
            if (!target_cpu_state) {
                ret = QEMU_PSCI_RET_INVALID_PARAMS;
                break;
            }
            target_cpu = ARM_CPU(target_cpu_state);

            g_assert(bql_locked());
            ret = target_cpu->power_state;
            break;
        default:
            /* Everything above affinity level 0 is always on. */
            ret = 0;
        }
        break;
    case QEMU_PSCI_0_2_FN_SYSTEM_RESET:
        qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        /* QEMU reset and shutdown are async requests, but PSCI
         * mandates that we never return from the reset/shutdown
         * call, so power the CPU off now so it doesn't execute
         * anything further.
         */
        goto cpu_off;
    case QEMU_PSCI_0_2_FN_SYSTEM_OFF:
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
        goto cpu_off;
    case QEMU_PSCI_0_1_FN_CPU_ON:
    case QEMU_PSCI_0_2_FN_CPU_ON:
    case QEMU_PSCI_0_2_FN64_CPU_ON:
    {
        /* The PSCI spec mandates that newly brought up CPUs start
         * in the highest exception level which exists and is enabled
         * on the calling CPU. Since the QEMU PSCI implementation is
         * acting as a "fake EL3" or "fake EL2" firmware, this for us
         * means that we want to start at the highest NS exception level
         * that we are providing to the guest.
         * The execution mode should be that which is currently in use
         * by the same exception level on the calling CPU.
         * The CPU should be started with the context_id value
         * in x0 (if AArch64) or r0 (if AArch32).
         */
        int target_el = arm_feature(env, ARM_FEATURE_EL2) ? 2 : 1;
        bool target_aarch64 = arm_el_is_aa64(env, target_el);

        mpidr = param[1];
        entry = param[2];
        context_id = param[3];
        ret = arm_set_cpu_on(mpidr, entry, context_id,
                             target_el, target_aarch64);
        break;
    }
    case QEMU_PSCI_0_1_FN_CPU_OFF:
    case QEMU_PSCI_0_2_FN_CPU_OFF:
        goto cpu_off;
    case QEMU_PSCI_0_1_FN_CPU_SUSPEND:
    case QEMU_PSCI_0_2_FN_CPU_SUSPEND:
    case QEMU_PSCI_0_2_FN64_CPU_SUSPEND:
        if (param[1] & ~(uint64_t)QEMU_PSCI_0_2_POWER_STATE_MASK) {
            ret = QEMU_PSCI_RET_INVALID_PARAMS;
            break;
        }
        if (param[1] & QEMU_PSCI_0_2_POWER_STATE_TYPE_MASK) {
            CPUState *cs = CPU(cpu);
            int target_el = arm_feature(env, ARM_FEATURE_EL2) ? 2 : 1;

            cpu->psci_powerdown_entry = param[2];
            cpu->psci_powerdown_context_id = param[3];
            cpu->psci_powerdown_target_el = target_el;
            cpu->psci_powerdown_target_aa64 =
                arm_el_is_aa64(env, target_el);
            qatomic_set(&cpu->psci_powerdown_pending, true);

            /*
             * The redistributor wake request is level-sensitive.  It may
             * have become asserted after Linux put the GIC CPU interface to
             * sleep but before this PSCI call reached QEMU.  Latch that
             * already-active request now so CPU_SUSPEND cannot lose it.
             */
            if (qatomic_read(&cpu->psci_wakeup_requested)) {
                cpu_interrupt(cs, CPU_INTERRUPT_EXITTB);
            }

            cs->exception_index = EXCP_HLT;
            cs->halted = 1;
            cpu_loop_exit(cs);
        }
        if (is_a64(env)) {
            env->xregs[0] = 0;
        } else {
            env->regs[0] = 0;
        }
        helper_wfi(env, 4);
        break;
    case QEMU_PSCI_1_0_FN_PSCI_FEATURES:
        switch (param[1]) {
        case QEMU_PSCI_0_2_FN_PSCI_VERSION:
        case QEMU_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
        case QEMU_PSCI_0_2_FN_AFFINITY_INFO:
        case QEMU_PSCI_0_2_FN64_AFFINITY_INFO:
        case QEMU_PSCI_0_2_FN_SYSTEM_RESET:
        case QEMU_PSCI_0_2_FN_SYSTEM_OFF:
        case QEMU_PSCI_0_1_FN_CPU_ON:
        case QEMU_PSCI_0_2_FN_CPU_ON:
        case QEMU_PSCI_0_2_FN64_CPU_ON:
        case QEMU_PSCI_0_1_FN_CPU_OFF:
        case QEMU_PSCI_0_2_FN_CPU_OFF:
        case QEMU_PSCI_0_1_FN_CPU_SUSPEND:
        case QEMU_PSCI_0_2_FN_CPU_SUSPEND:
        case QEMU_PSCI_0_2_FN64_CPU_SUSPEND:
        case QEMU_PSCI_1_0_FN_PSCI_FEATURES:
            if (!(param[1] & QEMU_PSCI_0_2_64BIT) || is_a64(env)) {
                ret = 0;
                break;
            }
            /* fallthrough */
        case QEMU_PSCI_0_1_FN_MIGRATE:
        case QEMU_PSCI_0_2_FN_MIGRATE:
        default:
            ret = QEMU_PSCI_RET_NOT_SUPPORTED;
            break;
        }
        break;
    case QEMU_PSCI_0_1_FN_MIGRATE:
    case QEMU_PSCI_0_2_FN_MIGRATE:
    default:
        ret = QEMU_PSCI_RET_NOT_SUPPORTED;
        break;
    }

err:
    if (is_a64(env)) {
        env->xregs[0] = ret;
    } else {
        env->regs[0] = ret;
    }
    return;

cpu_off:
    ret = arm_set_cpu_off(arm_cpu_mp_affinity(cpu));
    /* notreached */
    /* sanity check in case something failed */
    assert(ret == QEMU_ARM_POWERCTL_RET_SUCCESS);
}
