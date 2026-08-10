i.MX50 Tequila (``imx50-tequila``)
==================================

The ``imx50-tequila`` machine models the Amazon Kindle 4 Tequila board,
which uses the Freescale i.MX508 SoC and a Cortex-A8 CPU.  It supports the
legacy Lab126 U-Boot 2009.08 image linked for i.MX50 internal RAM.

The onboard eMMC is connected to eSDHC3 and is the first MMC device seen by
U-Boot.  Supply its user area as an SD drive.  IDME variables are populated in
the volatile eMMC boot partition 1 and do not modify the disk image.

Example::

  qemu-system-arm -M imx50-tequila -m 256M -nographic \
    -bios k4-main.bin \
    -drive file=tequila-emmc.img,if=sd,format=raw

The IDME defaults identify a Tequila board.  They can be overridden with
machine properties, for example::

  qemu-system-arm \
    -M imx50-tequila,idme-serial=B00E000000000001,idme-pcbsn=0031500000000001 \
    -m 256M -nographic -bios k4-main.bin \
    -drive file=tequila-emmc.img,if=sd,format=raw

The available properties are ``idme-serial``, ``idme-accel``, ``idme-mac``,
``idme-sec``, ``idme-pcbsn``, ``idme-bootmode``, and ``idme-postmode``.
