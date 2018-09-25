/*
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef MX6_TETRA_ANDROID_COMMON_H
#define MX6_TETRA_ANDROID_COMMON_H

#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED

#define CONFIG_BCB_SUPPORT
#define CONFIG_CMD_READ

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_VBUS_DRAW	2

#define CONFIG_G_DNL_VENDOR_NUM		0x18d1
#define CONFIG_G_DNL_PRODUCT_NUM	0x0d02
#define CONFIG_G_DNL_MANUFACTURER	"FSL"

#define CONFIG_USB_FUNCTION_FASTBOOT
#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_FASTBOOT_FLASH

/* For NAND we don't support lock/unlock */
#define CONFIG_FASTBOOT_LOCK
#define FSL_FASTBOOT_FB_DEV "mmc"
#define FASTBOOT_ENCRYPT_LOCK
#define CONFIG_FSL_CAAM_KB
#define CONFIG_CMD_FSL_CAAM_KB
#define CONFIG_SHA1
#define CONFIG_SHA256

#define CONFIG_FSL_FASTBOOT
#define CONFIG_ANDROID_RECOVERY

#if defined CONFIG_SPI_BOOT
#define CONFIG_FASTBOOT_STORAGE_MMC
#elif defined CONFIG_NAND_BOOT
#define CONFIG_FASTBOOT_STORAGE_NAND
#elif defined CONFIG_SYS_BOOT_SATA
#define CONFIG_FASTBOOT_STORAGE_SATA
#else
#define CONFIG_FASTBOOT_STORAGE_MMC
#endif

#define CONFIG_CMD_BOOTA
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_SERIAL_TAG

#undef CONFIG_EXTRA_ENV_SETTINGS
#undef CONFIG_BOOTCOMMAND

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"splashpos=m,m\0"	  \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"fastboot_dev=mmc0\0"	\
	"bootcmd=boota mmc0\0"	\
	"bootargs=console=ttymxc0,115200 androidboot.console=ttymxc0 consoleblank=0 vmalloc=128M init=/init video=mxcfb0:dev=hdmi,1920x1080M@60,bpp=32 video=mxcfb1:off video=mxcfb2:off video=mxcfb3:off androidboot.hardware=freescale cma=448M androidboot.selinux=permissive androidboot.dm_verity=disabled androidboot.storage_type=sd gpt\0"	\

#define CONFIG_FASTBOOT_BUF_ADDR   CONFIG_SYS_LOAD_ADDR
#define CONFIG_FASTBOOT_BUF_SIZE   0x19000000
#endif /* MX6_SABRE_ANDROID_COMMON_H */
