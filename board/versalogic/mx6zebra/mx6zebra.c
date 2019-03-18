/*
 * Copyright (C) 2012-2016 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <linux/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "../common/pfuze.h"
#include <asm/arch/mx6-ddr.h>
#include <usb.h>
#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define UART_PAD_CTRL0  (PAD_CTL_PUS_100K_DOWN |                   \
        PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |                 \
        PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_OD_PAD_CTRL  (PAD_CTL_ODE | PAD_CTL_HYS |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_PUS_100K_UP )

#define RESET_OD_PAD_CTRL  (PAD_CTL_ODE | PAD_CTL_HYS |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_PUS_100K_UP )

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)


//#define I2C_PMIC	1

#define I2C_PAD MUX_PAD_CTRL(I2C_PAD_CTRL)

#define DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {

	MX6_PAD_CSI0_DAT10__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D19__UART1_CTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_D20__UART1_RTS_B | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_NANDF_D4__GPIO2_IO04 | MUX_PAD_CTRL(UART_PAD_CTRL0),
	MX6_PAD_NANDF_D6__GPIO2_IO06 | MUX_PAD_CTRL(UART_PAD_CTRL0),
};

static iomux_v3_cfg_t const uart3_pads[] = {
        MX6_PAD_EIM_D24__UART3_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
        MX6_PAD_EIM_D25__UART3_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};


static iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/*
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	*/
        MX6_PAD_RGMII_RD0__GPIO6_IO25            | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* pin 31 - 1 - (MODE1) all */
        MX6_PAD_RGMII_RD1__GPIO6_IO27            | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* pin 28 - 1 - (MODE2) all */
        MX6_PAD_RGMII_RD2__GPIO6_IO28            | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* pin 27 - 1 - (MODE3) all */
        MX6_PAD_RGMII_RD3__GPIO6_IO29            | MUX_PAD_CTRL(NO_PAD_CTRL),
        /* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
        MX6_PAD_RGMII_RX_CTL__GPIO6_IO24         | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* AR8031 PHY Reset */
        MX6_PAD_ENET_CRS_DV__GPIO1_IO25         | MUX_PAD_CTRL(ENET_OD_PAD_CTRL),
};

static iomux_v3_cfg_t const enet_pads2[] = {
        MX6_PAD_RGMII_RD0__RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD1__RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD2__RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD3__RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL      | MUX_PAD_CTRL(ENET_PAD_CTRL),

};


static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

	/* Reset AR8031 PHY */
	gpio_request(IMX_GPIO_NR(1, 25), "GPIO1_25"); 
        gpio_request(IMX_GPIO_NR(6, 24), "GPIO6_24");
        gpio_request(IMX_GPIO_NR(6, 25), "GPIO6_25");
        gpio_request(IMX_GPIO_NR(6, 27), "GPIO6_27");
        gpio_request(IMX_GPIO_NR(6, 28), "GPIO6_28");
        gpio_request(IMX_GPIO_NR(6, 29), "GPIO6_29");
        gpio_direction_output(IMX_GPIO_NR(1, 25), 0); /* assert PHY rst */

        gpio_direction_output(IMX_GPIO_NR(6, 24), 1);
        gpio_direction_output(IMX_GPIO_NR(6, 25), 1);
        gpio_direction_output(IMX_GPIO_NR(6, 27), 1);
        gpio_direction_output(IMX_GPIO_NR(6, 28), 1);
        gpio_direction_output(IMX_GPIO_NR(6, 29), 1);
        udelay(1000 * 10);

        gpio_set_value(IMX_GPIO_NR(1, 25), 1); /* deassert PHY rst */

        /* Need delay 100ms to exit from reset. */
        udelay(1000 * 100);

        gpio_free(IMX_GPIO_NR(6, 24));
        gpio_free(IMX_GPIO_NR(6, 25));
        gpio_free(IMX_GPIO_NR(6, 27));
        gpio_free(IMX_GPIO_NR(6, 28));
        gpio_free(IMX_GPIO_NR(6, 29));
        udelay(1); /* short delay to avoid contentions */
	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_GPIO_2__SD2_WP          | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_4__SD2_CD_B        | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO_6__SD2_LCTL        | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/*
static iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NANDF_D0__GPIO2_IO00    | MUX_PAD_CTRL(NO_PAD_CTRL), // CD
};
*/
static iomux_v3_cfg_t const usdhc4_pads[] = {
	MX6_PAD_NANDF_ALE__SD4_RESET | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


static iomux_v3_cfg_t const ecspi1_pads[] = {
	/*MX6_PAD_DISP0_DAT15__ECSPI1_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),*/
        MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_EIM_EB2__GPIO2_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL),
        /*MX6_PAD_EIM_EB2__ECSPI1_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),*/
};

static iomux_v3_cfg_t const ecspi3_pads[] = {
	MX6_PAD_DISP0_DAT0__ECSPI3_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__ECSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__ECSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__ECSPI3_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__ECSPI3_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__ECSPI3_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
};
static void setup_spi(void)
{
        u32 reg;
        struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

        // Enable ECSPI1 clocks
        reg = readl(&ccm->CCGR1);
        reg |= MXC_CCM_CCGR1_ECSPI1S_MASK;;
        writel(reg, &ccm->CCGR1);

	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
        imx_iomux_v3_setup_multiple_pads(ecspi3_pads, ARRAY_SIZE(ecspi3_pads));
	gpio_request(IMX_GPIO_NR(2, 30), "SPI GPIO");
        gpio_direction_output(IMX_GPIO_NR(2, 30), 0);
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (IMX_GPIO_NR(2, 30)) : 0;
}
static iomux_v3_cfg_t const rgb_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/*MX6_PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),*/
};

static void enable_rgb(struct display_info_t const *dev)
{
	imx_iomux_v3_setup_multiple_pads(rgb_pads, ARRAY_SIZE(rgb_pads));
	gpio_direction_output(DISP0_PWR_EN, 1);
}


static struct i2c_pads_info i2c_pad_info1 = {
        .scl = {
                .i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL | I2C_PAD,
                .gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | I2C_PAD,
                .gp = IMX_GPIO_NR(3, 21)
        },
        .sda = {
                .i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | I2C_PAD,
                .gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | I2C_PAD,
                .gp = IMX_GPIO_NR(3, 28)
        }
};

static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | I2C_PAD,
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | I2C_PAD,
		.gp = IMX_GPIO_NR(4, 13)
	}
};
static struct i2c_pads_info i2c_pad_info3 = {
        .scl = {
                .i2c_mode = MX6_PAD_GPIO_5__I2C3_SCL | I2C_PAD,
                .gpio_mode = MX6_PAD_GPIO_5__GPIO1_IO05 | I2C_PAD,
                .gp = IMX_GPIO_NR(1, 5)
        },
        .sda = {
                .i2c_mode = MX6_PAD_GPIO_16__I2C3_SDA | I2C_PAD,
                .gpio_mode = MX6_PAD_GPIO_16__GPIO7_IO11 | I2C_PAD,
                .gp = IMX_GPIO_NR(7, 11)
        }
};

iomux_v3_cfg_t const di0_pads[] = {
	MX6_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,	/* DISP0_CLK */
	MX6_PAD_DI0_PIN2__IPU1_DI0_PIN02,		/* DISP0_HSYNC */
	MX6_PAD_DI0_PIN3__IPU1_DI0_PIN03,		/* DISP0_VSYNC */
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
        imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
//	{USDHC3_BASE_ADDR},
	{USDHC4_BASE_ADDR},
};

#define USDHC2_CD_GPIO	IMX_GPIO_NR(1, 4)
//#define USDHC3_CD_GPIO	IMX_GPIO_NR(2, 0)
int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int mmc_get_env_dev(void)
{
        u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
        u32 dev_no;
        u32 bootsel;

        bootsel = (soc_sbmr & 0x000000FF) >> 6 ;

        /* If not boot from sd/mmc, use default value */
        if (bootsel != 1)
                return CONFIG_SYS_MMC_ENV_DEV;

        /* BOOT_CFG2[3] and BOOT_CFG2[4] */
        dev_no = (soc_sbmr & 0x00001800) >> 11;

        /* need ubstract 1 to map to the mmc device id
         * see the comments in board_mmc_init function
         */

        dev_no--;

        return dev_no;
}


int mmc_map_to_kernel_blk(int devno)
{
	return devno + 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR:
		ret = !gpio_get_value(USDHC2_CD_GPIO);
		break;
//	case USDHC3_BASE_ADDR:
//		ret = !gpio_get_value(USDHC3_CD_GPIO);
//		break;
	case USDHC4_BASE_ADDR:
		ret = 1; /* eMMC/uSDHC4 is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int ret;
	int i;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    SD2
	 * mmc1                    SD3
	 * mmc2                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			gpio_direction_input(USDHC2_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
/*
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
			gpio_direction_input(USDHC3_CD_GPIO);
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
*/
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
#else
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x2) {
	case 0x1:
		imx_iomux_v3_setup_multiple_pads(
			usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
/*
	case 0x2:
		imx_iomux_v3_setup_multiple_pads(
			usdhc3_pads, ARRAY_SIZE(usdhc3_pads));
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
*/
	case 0x2:
		imx_iomux_v3_setup_multiple_pads(
			usdhc4_pads, ARRAY_SIZE(usdhc4_pads));
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

int mx6_rgmii_rework(struct phy_device *phydev)
{
	//unsigned short val;

	//added iMX6 Rex KSZ90xx support, taken from Saberlite
	/* enable master mode, force phy to 100Mbps */
	//phy_write(phydev, phy_addr, 0x9, 0x1c00);

	/* min rx data delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x0b, 0x8105);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x0b, 0x8104);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x0c, 0xf0f0);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x0b, 0x104);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	mx6_rgmii_rework(phydev);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#if defined(CONFIG_VIDEO_IPUV3)
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT |
	       IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT;
	writel(reg, &iomux->gpr[2]);
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 640,
		.yres           = 480,
		.pixclock       = 39721,
		.left_margin    = 48,
		.right_margin   = 16,
		.upper_margin   = 33,
		.lower_margin   = 10,
		.hsync_len      = 96,
		.vsync_len      = 2,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "SEIKO-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 29850,
		.left_margin    = 89,
		.right_margin   = 164,
		.upper_margin   = 23,
		.lower_margin   = 10,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

iomux_v3_cfg_t const backlight_pads[] = {
        MX6_PAD_NANDF_CS2__GPIO6_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_backlight(void)
{
        gpio_direction_output(IMX_GPIO_NR(6, 15), 1);
        imx_iomux_v3_setup_multiple_pads(backlight_pads,
                                         ARRAY_SIZE(backlight_pads));
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;


	gpio_direction_output(IMX_GPIO_NR(6, 16), 1); /* LVDS power On */

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	imx_iomux_v3_setup_multiple_pads(di0_pads, ARRAY_SIZE(di0_pads));

        setup_iomux_backlight();
	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

        reg = readl(&iomux->gpr[3]);
        reg &= ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK |
                 IOMUXC_GPR3_HDMI_MUX_CTL_MASK);
        reg |= (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<
                IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET) |
               (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<
                IOMUXC_GPR3_HDMI_MUX_CTL_OFFSET);
        writel(reg, &iomux->gpr[3]);
}
#endif /* CONFIG_VIDEO_IPUV3 */

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}
static void setup_fec(void)
{
	if (is_mx6dqp()) {
		int ret;

		// select ENET MAC0 TX clock from PLL
		imx_iomux_set_gpr_register(1, 23, 1, 1);
		ret = enable_fec_anatop_clock(0, ENET_125MHZ);
		if (ret)
		    printf("Error fec anatop clock settings!\n");
	}
}

int board_eth_init(bd_t *bis)
{
	setup_iomux_enet();
	//setup_pcie();

	return cpu_eth_init(bis);
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)
#define UCTRL_OC_POL           (1 << 8)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	/*MX6_PAD_EIM_D22__USB_OTG_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),*/
	MX6_PAD_GPIO_1__USB_OTG_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL), //NO_PAD_CTRL),
	MX6_PAD_KEY_COL4__USB_OTG_OC | MUX_PAD_CTRL(NO_PAD_CTRL), //Over current
};

static iomux_v3_cfg_t const usb_hc1_pads[] = {
	/*MX6_PAD_EIM_D31__USB_H1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL), */
	MX6_PAD_EIM_D31__GPIO3_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_D30__USB_H1_OC | MUX_PAD_CTRL(NO_PAD_CTRL), //Over current
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));

	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 1);

	imx_iomux_v3_setup_multiple_pads(usb_hc1_pads,
					 ARRAY_SIZE(usb_hc1_pads));
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 0)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);
        setbits_le32(usbnc_usb_ctrl, UCTRL_OC_POL);

	return 0;
}

int board_ehci_power(int port, int on)
{
	switch (port) {
	case 0:
		break;
	case 1:
		if (on)
			gpio_direction_output(IMX_GPIO_NR(3, 31), 1);
		else
			gpio_direction_output(IMX_GPIO_NR(3, 31), 0);
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return -EINVAL;
	}
	return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	return 0;
}

static iomux_v3_cfg_t const peripheral_reset_pad[] = {
	MX6_PAD_EIM_A25__GPIO5_IO02 | MUX_PAD_CTRL(RESET_OD_PAD_CTRL)
};

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	//RESET Peripherals
	imx_iomux_v3_setup_multiple_pads(peripheral_reset_pad,
						 ARRAY_SIZE(peripheral_reset_pad));
	gpio_request(IMX_GPIO_NR(5, 2), "GPIO 5_2");
	gpio_set_value(IMX_GPIO_NR(5, 2), 0);
	mdelay(2);
	gpio_set_value(IMX_GPIO_NR(5, 2), 1);

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1); 
        setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	return 0;
}

int power_init_board(void)
{
	return 0;
}
void ldo_mode_set(int ldo_bypass)
{
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	//{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},

	{NULL,	 0},
};
#endif

int board_late_init(void)
{

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	setenv("board_name", "ZEBRA");

	if (is_mx6dqp())
		setenv("board_rev", "MX6QP");
	else if (is_cpu_type(MXC_CPU_MX6Q) || is_cpu_type(MXC_CPU_MX6D))
		setenv("board_rev", "MX6Q");
	else if (is_cpu_type(MXC_CPU_MX6DL) || is_cpu_type(MXC_CPU_MX6SOLO))
		setenv("board_rev", "MX6DL");
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: MX6-Zebra\n");
	return 0;
}

#ifdef CONFIG_POWER
int power_init_board(void)
{
        struct pmic *pfuze;
        unsigned int reg;
        int ret;

        pfuze = pfuze_common_init(I2C_PMIC);
        if (!pfuze)
                return -ENODEV;

        /*if (is_mx6dqp())*/ /* Use APS setting for all */
                ret = pfuze_mode_init(pfuze, APS_APS);
        /*else*/ /* Don't allow hard PFM mode setting */
                /*ret = pfuze_mode_init(pfuze, APS_PFM);*/

        if (ret < 0)
                return ret;

        if (is_mx6dqp()) {
                /* set SW1C staby volatage 1.075V*/
                pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
                reg &= ~0x3f;
                reg |= 0x1f;
                pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

                /* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
                pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
                reg &= ~0xc0;
                reg |= 0x40;
                pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);

                /* set SW2/VDDARM staby volatage 0.975V*/
                pmic_reg_read(pfuze, PFUZE100_SW2STBY, &reg);
                reg &= ~0x3f;
                reg |= 0x17;
                pmic_reg_write(pfuze, PFUZE100_SW2STBY, reg);

                /* set SW2/VDDARM step ramp up time to from 16us to 4us/25mV */
                pmic_reg_read(pfuze, PFUZE100_SW2CONF, &reg);
                reg &= ~0xc0;
                reg |= 0x40;
                pmic_reg_write(pfuze, PFUZE100_SW2CONF, reg);
        } else {
                /* set SW1AB staby volatage 0.975V*/
                pmic_reg_read(pfuze, PFUZE100_SW1ABSTBY, &reg);
                reg &= ~0x3f;
                reg |= 0x1b;
                pmic_reg_write(pfuze, PFUZE100_SW1ABSTBY, reg);

                /* set SW1AB/VDDARM step ramp up time from 16us to 4us/25mV */
                pmic_reg_read(pfuze, PFUZE100_SW1ABCONF, &reg);
                reg &= ~0xc0;
                reg |= 0x40;
                pmic_reg_write(pfuze, PFUZE100_SW1ABCONF, reg);

                /* set SW1C staby volatage 0.975V*/
                pmic_reg_read(pfuze, PFUZE100_SW1CSTBY, &reg);
                reg &= ~0x3f;
                reg |= 0x1b;
                pmic_reg_write(pfuze, PFUZE100_SW1CSTBY, reg);

                /* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
                pmic_reg_read(pfuze, PFUZE100_SW1CCONF, &reg);
                reg &= ~0xc0;
                reg |= 0x40;
                pmic_reg_write(pfuze, PFUZE100_SW1CCONF, reg);
        }

        return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
        unsigned int value;
        int is_400M;
        unsigned char vddarm;
        struct pmic *p = pmic_get("PFUZE100");

        if (!p) {
                printf("No PMIC found!\n");
                return;
        }

        /* switch to ldo_bypass mode , boot on 800Mhz */
        if (ldo_bypass) {
                prep_anatop_bypass();
                if (is_mx6dqp()) {
                        /* decrease VDDARM for 400Mhz DQP:1.1V*/
                        pmic_reg_read(p, PFUZE100_SW2VOL, &value);
                        value &= ~0x3f;
                        value |= 0x1c;
                        pmic_reg_write(p, PFUZE100_SW2VOL, value);
                } else {
                        /* decrease VDDARM for 400Mhz DQ:1.1V, DL:1.275V */
                        pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
                        value &= ~0x3f;
                        if (is_cpu_type(MXC_CPU_MX6DL))
                                value |= 0x27;
                        else
                                value |= 0x20;

                        pmic_reg_write(p, PFUZE100_SW1ABVOL, value);
                }
                /* increase VDDSOC to 1.3V */
                pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
                value &= ~0x3f;
                value |= 0x28;
                pmic_reg_write(p, PFUZE100_SW1CVOL, value);

                /*
                 * MX6Q/DQP:
                 * VDDARM:1.15V@800M; VDDSOC:1.175V@800M
                 * VDDARM:0.975V@400M; VDDSOC:1.175V@400M
                 * MX6DL:
                 * VDDARM:1.175V@800M; VDDSOC:1.175V@800M
                 * VDDARM:1.075V@400M; VDDSOC:1.175V@400M
                 */
                is_400M = set_anatop_bypass(2);
                if (is_mx6dqp()) {
                        pmic_reg_read(p, PFUZE100_SW2VOL, &value);
                        value &= ~0x3f;
                        if (is_400M)
                                value |= 0x17;
                        else
                                value |= 0x1e;
                        pmic_reg_write(p, PFUZE100_SW2VOL, value);
                }

                if (is_400M) {
                        if (is_cpu_type(MXC_CPU_MX6DL))
                                /*vddarm = 0x1f;*/ /* 1.075V */
                                vddarm = 0x21; /* 1.125V=spec */

                        else
                                vddarm = 0x1b;
                } else {
                        if (is_cpu_type(MXC_CPU_MX6DL))
                                vddarm = 0x23;
                        else
                                vddarm = 0x22;
                }
                pmic_reg_read(p, PFUZE100_SW1ABVOL, &value);
                value &= ~0x3f;
                value |= vddarm;
                pmic_reg_write(p, PFUZE100_SW1ABVOL, value);

                /* decrease VDDSOC to 1.175V */
                pmic_reg_read(p, PFUZE100_SW1CVOL, &value);
                value &= ~0x3f;
                value |= 0x23;
                pmic_reg_write(p, PFUZE100_SW1CVOL, value);

                finish_anatop_bypass();
                printf("switch to ldo_bypass mode!\n");
        }
}
#endif
#endif

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY

#define GPIO_VOL_DN_KEY IMX_GPIO_NR(1, 5)
iomux_v3_cfg_t const recovery_key_pads[] = {
	(MX6_PAD_GPIO_5__GPIO1_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

int is_recovery_key_pressing(void)
{
	int button_pressed = 0;

    /* Check Recovery Combo Button press or not. */
	imx_iomux_v3_setup_multiple_pads(recovery_key_pads,
			ARRAY_SIZE(recovery_key_pads));

    gpio_direction_input(GPIO_VOL_DN_KEY);

    if (gpio_get_value(GPIO_VOL_DN_KEY) == 0) { /* VOL_DN key is low assert */
		button_pressed = 1;
		printf("Recovery key pressed\n");
    }

	return  button_pressed;
}


#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/


#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	gpio_request(KEY_VOL_UP, "KEY Volume UP");
	gpio_direction_input(KEY_VOL_UP);

	/* Only enter in Falcon mode if KEY_VOL_UP is pressed */
	return gpio_get_value(KEY_VOL_UP);
}
#endif

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

static void gpr_init(void)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* enable AXI cache for VDOA/VPU/IPU */
	writel(0xF00000CF, &iomux->gpr[4]);
	if (is_mx6dqp()) {
		/* set IPU AXI-id1 Qos=0x1 AXI-id0/2/3 Qos=0x7 */
		writel(0x007F007F, &iomux->gpr[6]);
		writel(0x007F007F, &iomux->gpr[7]);
	} else {
		/* set IPU AXI-id0 Qos=0xf(bypass) AXI-id1 Qos=0x7 */
		writel(0x007F007F, &iomux->gpr[6]);
		writel(0x007F007F, &iomux->gpr[7]);
	}
}

/*
 * This section requires the differentiation between iMX6 Sabre boards, but
 * for now, it will configure only for the mx6q variant.
 */
static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		/* width of data bus:0=16,1=32,2=64 */
		.dsize = 2,
		/* config for full 4GB range so that get_mem_size() works */
		.cs_density = 32, /* 32Gb per CS */
		/* single chip select */
		.ncs = 1,
		.cs1_mirror = 0,
		.rtt_wr = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Wr = RZQ/4 */
		.rtt_nom = 1 /*DDR3_RTT_60_OHM*/,	/* RTT_Nom = RZQ/4 */
		.walat = 1,	/* Write additional latency */
		.ralat = 5,	/* Read additional latency */
		.mif3_mode = 3,	/* Command prediction working mode */
		.bi_on = 1,	/* Bank interleaving enabled */
		.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
		.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
		.ddr_type = DDR_TYPE_DDR3,
	};

	if (is_mx6dqp()) {
		mx6dq_dram_iocfg(64, &mx6dqp_ddr_ioregs, &mx6_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6dqp_mmcd_calib, &mem_ddr);
	} else {
		mx6dq_dram_iocfg(64, &mx6_ddr_ioregs, &mx6_grp_ioregs);
		mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
	}
}

void board_init_f(ulong dummy)
{
	/* DDR initialization */
	spl_dram_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif
