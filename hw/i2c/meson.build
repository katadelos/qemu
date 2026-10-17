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
i2c_ss.add(when: 'CONFIG_MT8113_ELIPSA2E', if_true: files('elan-ekth3500.c'))
i2c_ss.add(when: 'CONFIG_MT8113_KOBO', if_true: files('jd9930.c', 'kx122.c'))
i2c_ss.add(when: 'CONFIG_NEONODE_ZFORCE', if_true: files('zforce.c'))
i2c_ss.add(when: 'CONFIG_IMX50_YOSHI', if_true: files('cyttsp.c'))
i2c_ss.add(when: 'CONFIG_KOBO_MSP430', if_true: files('kobotouch_msp430.c'))
i2c_ss.add(when: 'CONFIG_KOBO_NTX_PMIC', if_true: files('forma-pmic.c'))
i2c_ss.add(when: 'CONFIG_CYTTSP5', if_true: files('cyttsp5.c'))
i2c_ss.add(when: 'CONFIG_PARADE_TT', if_true: files('parade-tt7010.c'))
i2c_ss.add(when: 'CONFIG_BMA2X2', if_true: files('bma2x2.c'))
i2c_ss.add(when: 'CONFIG_MMA7660', if_true: files('mma7660.c'))
i2c_ss.add(when: 'CONFIG_MAX77696', if_true: files('max77696.c'))
i2c_ss.add(when: 'CONFIG_DRV2667', if_true: files('drv2667.c'))
i2c_ss.add(when: 'CONFIG_CYTTSP4', if_true: files('cyttsp4.c'))
i2c_ss.add(when: 'CONFIG_MAX77796', if_true: files('max77796.c'))
i2c_ss.add(when: 'CONFIG_BD71815', if_true: files('bd71815.c'))
i2c_ss.add(when: 'CONFIG_FP9928', if_true: files('fp9928.c'))
i2c_ss.add(when: 'CONFIG_ELAN_KTF2K', if_true: files('elan-ktf2k.c'))
i2c_ss.add(when: 'CONFIG_MSG2133', if_true: files('msg2133.c'))
i2c_ss.add(when: 'CONFIG_BD71827', if_true: files('bd71827.c'))
i2c_ss.add(when: 'CONFIG_FP9929', if_true: files('fp9929.c'))
i2c_ss.add(when: 'CONFIG_GOODIX_GT9XX', if_true: files('goodix_gt9xx.c'))
i2c_ss.add(when: 'CONFIG_GOODIX_GT813', if_true: files('goodix_gt813.c'))
i2c_ss.add(when: 'CONFIG_CY8C_SLIDER', if_true: files('cy8c-slider.c'))
i2c_ss.add(when: 'CONFIG_LM3695', if_true: files('lm3695.c'))
i2c_ss.add(when: 'CONFIG_LM3630A', if_true: files('lm3630a.c'))
i2c_ss.add(when: 'CONFIG_LM3697', if_true: files('lm3697.c'))
i2c_ss.add(when: 'CONFIG_AW99703', if_true: files('aw99703.c'))
i2c_ss.add(when: 'CONFIG_TMP103', if_true: files('tmp103.c'))
i2c_ss.add(when: 'CONFIG_MAX14656', if_true: files('max14656.c'))
i2c_ss.add(when: 'CONFIG_MT8113_BELLATRIX4', if_true: files(
  'bd71828.c',
  'fp9935.c',
  'ft5536g.c',
  'lis2du12.c',
))
i2c_ss.add(when: 'CONFIG_MT8110_BELLATRIX', if_true: files(
  'goodix_gtx8.c',
  'fp9966.c',
  'bq25611d.c',
  'kx132.c',
  'ti-opt-sensors.c',
))
i2c_ss.add(when: 'CONFIG_WACOM_EMR', if_true: files('wacom_emr.c'))
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
i2c_ss.add(when: ['CONFIG_MT8113_BELLATRIX4'], if_true: files('fp9967.c', 'max20342.c'))
i2c_ss.add(when: 'CONFIG_MT8115_PA6_CS8', if_true: files('fp9967.c', 'max20342.c', 'sgm66075.c', 'fp9936.c', 'ti-opt-sensors.c', 'focaltech-ft3a81.c'))
i2c_ss.add(when: 'CONFIG_MT8115_PA6_CS8', if_true: files('lsm6dso16is.c'))

i2c_ss.add(when: 'CONFIG_REMARKABLE_PMIC', if_true: files('remarkable-pmic.c'))


i2c_ss.add(when: 'CONFIG_SY7636', if_true: files('sy7636.c'))
system_ss.add_all(when: 'CONFIG_I2C', if_true: i2c_ss)
