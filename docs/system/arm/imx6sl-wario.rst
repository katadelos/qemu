i.MX6SL Wario (``imx6sl-wario``)
================================

The ``imx6sl-wario`` machine models the Lab126 Wario family built around an
i.MX6 SoloLite. It is intended for the legacy, non-device-tree Lab126 U-Boot
and Linux 3.0.35 software stack.

The machine starts a raw i.MX boot image through the OCRAM IVT, reports ARM
machine ID 4091, and provides 512 MiB of RAM by default. USDHC2 is the Wario
eMMC controller. A disk image therefore needs to be attached with
``if=sd,index=1``. The stock kernel image is stored in the eMMC user area at
byte offset ``0x41000`` in the user area and the root filesystem is DOS
partition 1. IDME data resides in eMMC boot partition 1. The backing image is
laid out as described by :doc:`../devices/emmc`: boot1, boot2, then user area.

Example::

  qemu-system-arm -M imx6sl-wario -m 512M -display none \
    -serial mon:stdio -bios u-boot.bin \
    -drive file=wario-emmc.img,if=sd,index=1,format=raw

The EPDC model supplies its MMIO interface, update/LUT completion and IRQ
behavior. It does not currently render electrophoretic waveforms.
