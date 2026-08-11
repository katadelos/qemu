i2c_ss = ss.source_set()
i2c_ss.add(when: 'CONFIG_I2C', if_true: files('core.c'))
i2c_ss.add(when: 'CONFIG_SMBUS', if_true: files('smbus_slave.c', 'smbus_master.c'))
i2c_ss.add(when: 'CONFIG_ACPI_SMBUS', if_true: files('pm_smbus.c'))
i2c_ss.add(when: 'CONFIG_ACPI_ICH9', if_true: files('smbus_ich9.c'))
i2c_ss.add(when: 'CONFIG_ASPEED_SOC', if_true: files('aspeed_i2c.c'))
i2c_ss.add(when: 'CONFIG_BITBANG_I2C', if_true: files('bitbang_i2c.c'))
i2c_ss.add(when: 'CONFIG_EXYNOS4', if_true: files('exynos4210_i2c.c'))
i2c_ss.add(when: 'CONFIG_IMX_I2C', if_true: files('imx_i2c.c'))
i2c_ss.add(when: 'CONFIG_MTK_I2C', if_true: files('mtk_i2c.c'))
i2c_ss.add(when: 'CONFIG_NEONODE_ZFORCE', if_true: files('zforce.c'))
i2c_ss.add(when: 'CONFIG_IMX50_KOBOTOUCH', if_true: files('kobotouch_msp430.c'))
i2c_ss.add(when: 'CONFIG_IMX6SL_WARIO', if_true: files('max77696.c', 'drv2667.c', 'cyttsp4.c'))
i2c_ss.add(when: 'CONFIG_IMX60_HEISENBERG', if_true: files(
  'bd71815.c',
  'fp9928.c',
))
i2c_ss.add(when: 'CONFIG_IMX6SLL_REX', if_true: files(
  'bd71827.c',
  'fp9929.c',
  'goodix_gt9xx.c',
  'lm3695.c',
  'max14656.c',
))
i2c_ss.add(when: 'CONFIG_MT8113_COLOURSOFT', if_true: files(
  'bd71828.c',
  'fp9935.c',
  'fp9967.c',
  'ft5536g.c',
  'lis2du12.c',
  'max20342.c',
))
i2c_ss.add(when: 'CONFIG_MPC_I2C', if_true: files('mpc_i2c.c'))
i2c_ss.add(when: 'CONFIG_ALLWINNER_I2C', if_true: files('allwinner-i2c.c'))
i2c_ss.add(when: 'CONFIG_NRF51_SOC', if_true: files('microbit_i2c.c'))
i2c_ss.add(when: 'CONFIG_NPCM7XX', if_true: files('npcm7xx_smbus.c'))
i2c_ss.add(when: 'CONFIG_SMBUS_EEPROM', if_true: files('smbus_eeprom.c'))
i2c_ss.add(when: 'CONFIG_ARM_SBCON_I2C', if_true: files('arm_sbcon_i2c.c'))
i2c_ss.add(when: 'CONFIG_OMAP', if_true: files('omap_i2c.c'))
i2c_ss.add(when: 'CONFIG_PPC4XX', if_true: files('ppc4xx_i2c.c'))
i2c_ss.add(when: 'CONFIG_PCA954X', if_true: files('i2c_mux_pca954x.c'))
i2c_ss.add(when: 'CONFIG_PMBUS', if_true: files('pmbus_device.c'))
i2c_ss.add(when: 'CONFIG_BCM2835_I2C', if_true: files('bcm2835_i2c.c'))
system_ss.add_all(when: 'CONFIG_I2C', if_true: i2c_ss)
