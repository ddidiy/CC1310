#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/packages;C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages;C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/bios_6_45_01_29/packages;C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/uia_2_00_05_50/packages
override XDCROOT = c:/ti/xdctools_3_32_00_06_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/packages;C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/tidrivers_cc13xx_cc26xx_2_16_00_08/packages;C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/bios_6_45_01_29/packages;C:/ti/tirtos_cc13xx_cc26xx_2_16_00_08/products/uia_2_00_05_50/packages;c:/ti/xdctools_3_32_00_06_core/packages;..
HOSTOS = Windows
endif
