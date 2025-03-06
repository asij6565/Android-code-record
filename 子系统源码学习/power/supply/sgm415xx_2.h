/* SPDX-License-Identifier: GPL-2.0-only */
// sgm4154x_2 Charger Driver
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.sg-micro.com

#ifndef _SGM4154x_2_CHARGER_H
#define _SGM4154x_2_CHARGER_H

#include <linux/i2c.h>

#define SGM4154x_2_MANUFACTURER	"Texas Instruments"
//#define __SGM41541_CHIP_ID__
//#define __SGM41542_CHIP_ID__
#define __SGM41512_CHIP_ID__
//#define __SGM41513_CHIP_ID__
//#define __SGM41513A_CHIP_ID__
//#define __SGM41513D_CHIP_ID__
//#define __SGM41516_CHIP_ID__
//#define __SGM41516D_CHIP_ID__
//#define __SGM41543_CHIP_ID__
//#define __SGM41543D_CHIP_ID__

#ifdef __SGM41541_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41541"
#define SGM4154x_2_PN_ID     (BIT(6)| BIT(5))
#endif

#ifdef __SGM41542_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41542"
#define SGM4154x_2_PN_ID      (BIT(6)| BIT(5)| BIT(3))
#endif

#ifdef __SGM41512_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41512"
#define SGM4154x_2_PN_ID      (BIT(5)| BIT(3))
#endif

#ifdef __SGM41513_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41513"
#define SGM4154x_2_PN_ID      0
#define SGM41511_PN_ID      BIT(4)
#endif

#ifdef __SGM41513A_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41513A"
#define SGM4154x_2_PN_ID      BIT(3)
#endif

#ifdef __SGM41513D_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41513D"
#define SGM4154x_2_PN_ID      BIT(3)
#endif

#ifdef __SGM41516_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41516"
#define SGM4154x_2_PN_ID      (BIT(6)| BIT(5))
#endif

#ifdef __SGM41516D_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41516D"
#define SGM4154x_2_PN_ID     (BIT(6)| BIT(5)| BIT(3))
#endif

#ifdef __SGM41543_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41543"
#define SGM4154x_2_PN_ID      BIT(6)
#endif

#ifdef __SGM41543D_CHIP_ID__
#define SGM4154x_2_NAME		"sgm41543D"
#define SGM4154x_2_PN_ID      (BIT(6)| BIT(3))
#endif

/*define register*/
#define SGM4154x_2_CHRG_CTRL_0	0x00
#define SGM4154x_2_CHRG_CTRL_1	0x01
#define SGM4154x_2_CHRG_CTRL_2	0x02
#define SGM4154x_2_CHRG_CTRL_3	0x03
#define SGM4154x_2_CHRG_CTRL_4	0x04
#define SGM4154x_2_CHRG_CTRL_5	0x05
#define SGM4154x_2_CHRG_CTRL_6	0x06
#define SGM4154x_2_CHRG_CTRL_7	0x07
#define SGM4154x_2_CHRG_STAT	    0x08
#define SGM4154x_2_CHRG_FAULT	    0x09
#define SGM4154x_2_CHRG_CTRL_a	0x0a
#define SGM4154x_2_CHRG_CTRL_b	0x0b
#define SGM4154x_2_CHRG_CTRL_c	0x0c
#define SGM4154x_2_CHRG_CTRL_d	0x0d
#define SGM4154x_2_INPUT_DET   	0x0e
#define SGM4154x_2_CHRG_CTRL_f	0x0f

/* charge status flags  */
#define SGM4154x_2_CHRG_EN		BIT(4)
#define SGM4154x_2_HIZ_EN		    BIT(7)
#define SGM4154x_2_TERM_EN		BIT(7)
#define SGM4154x_2_VAC_OVP_MASK	GENMASK(7, 6)
#define SGM4154x_2_DPDM_ONGOING   BIT(7)
#define SGM4154x_2_VBUS_GOOD      BIT(7)

#define SGM4154x_2_BOOSTV 		GENMASK(5, 4)
#define SGM4154x_2_BOOST_LIM 		BIT(7)
#define SGM4154x_2_OTG_EN		    BIT(5)

/* Part ID  */
#define SGM4154x_2_PN_MASK	    GENMASK(6, 3)

/* WDT TIMER SET  */
#define SGM4154x_2_WDT_TIMER_MASK        GENMASK(5, 4)
#define SGM4154x_2_WDT_TIMER_DISABLE     0
#define SGM4154x_2_WDT_TIMER_40S         BIT(4)
#define SGM4154x_2_WDT_TIMER_80S         BIT(5)
#define SGM4154x_2_WDT_TIMER_160S        (BIT(4)| BIT(5))

#define SGM4154x_2_WDT_RST_MASK          BIT(6)

/* SAFETY TIMER SET  */
#define SGM4154x_2_SAFETY_TIMER_MASK     GENMASK(3, 3)
#define SGM4154x_2_SAFETY_TIMER_DISABLE     0
#define SGM4154x_2_SAFETY_TIMER_EN       BIT(3)
#define SGM4154x_2_SAFETY_TIMER_5H         0
#define SGM4154x_2_SAFETY_TIMER_10H      BIT(2)

/* recharge voltage  */
#define SGM4154x_2_VRECHARGE              BIT(0)
#define SGM4154x_2_VRECHRG_STEP_mV		100
#define SGM4154x_2_VRECHRG_OFFSET_mV		100

/* charge status  */
#define SGM4154x_2_VSYS_STAT		BIT(0)
#define SGM4154x_2_THERM_STAT		BIT(1)
#define SGM4154x_2_PG_STAT		BIT(2)
#define SGM4154x_2_CHG_STAT_MASK	GENMASK(4, 3)
#define SGM4154x_2_PRECHRG		BIT(3)
#define SGM4154x_2_FAST_CHRG	    BIT(4)
#define SGM4154x_2_TERM_CHRG	    (BIT(3)| BIT(4))

/* charge type  */
#define SGM4154x_2_VBUS_STAT_MASK	GENMASK(7, 5)
#define SGM4154x_2_NOT_CHRGING	0
#define SGM4154x_2_USB_SDP		BIT(5)
#define SGM4154x_2_USB_CDP		BIT(6)
#define SGM4154x_2_USB_DCP		(BIT(5) | BIT(6))
#define SGM4154x_2_UNKNOWN	    (BIT(7) | BIT(5))
#define SGM4154x_2_NON_STANDARD	(BIT(7) | BIT(6))
#define SGM4154x_2_OTG_MODE	    (BIT(7) | BIT(6) | BIT(5))

/* TEMP Status  */
#define SGM4154x_2_TEMP_MASK	    GENMASK(2, 0)
#define SGM4154x_2_TEMP_NORMAL	BIT(0)
#define SGM4154x_2_TEMP_WARM	    BIT(1)
#define SGM4154x_2_TEMP_COOL	    (BIT(0) | BIT(1))
#define SGM4154x_2_TEMP_COLD	    (BIT(0) | BIT(3))
#define SGM4154x_2_TEMP_HOT	    (BIT(2) | BIT(3))

/* precharge current  */
#define SGM4154x_2_PRECHRG_CUR_MASK		GENMASK(7, 4)
#define SGM4154x_2_PRECHRG_CURRENT_STEP_uA		60000
#define SGM4154x_2_PRECHRG_I_MIN_uA		60000
#define SGM4154x_2_PRECHRG_I_MAX_uA		780000
#define SGM4154x_2_PRECHRG_I_DEF_uA		180000

/* termination current  */
#define SGM4154x_2_TERMCHRG_CUR_MASK		GENMASK(3, 0)
#define SGM4154x_2_TERMCHRG_CURRENT_STEP_uA	60000
#define SGM4154x_2_TERMCHRG_I_MIN_uA		60000
#define SGM4154x_2_TERMCHRG_I_MAX_uA		960000
#define SGM4154x_2_TERMCHRG_I_DEF_uA		150000

/* charge current  */
#define SGM4154x_2_ICHRG_I_MASK		GENMASK(5, 0)

#define SGM4154x_2_ICHRG_I_MIN_uA			0
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__))
#define SGM4154x_2_ICHRG_I_MAX_uA			3000000
#define SGM4154x_2_ICHRG_I_DEF_uA			1980000
#else
#define SGM4154x_2_ICHRG_I_STEP_uA	    60000
#define SGM4154x_2_ICHRG_I_MAX_uA			3780000
#define SGM4154x_2_ICHRG_I_DEF_uA			2040000
#endif
/* charge voltage  */
#define SGM4154x_2_VREG_V_MASK		GENMASK(7, 3)
#define SGM4154x_2_VREG_V_MAX_uV	    4624000
#define SGM4154x_2_VREG_V_MIN_uV	    3856000
#define SGM4154x_2_VREG_V_DEF_uV	    4208000
#define SGM4154x_2_VREG_V_STEP_uV	    32000

/* iindpm current  */
#define SGM4154x_2_IINDPM_I_MASK		GENMASK(4, 0)
#define SGM4154x_2_IINDPM_I_MIN_uA	100000
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__))
#define SGM4154x_2_IINDPM_I_MAX_uA	3200000
#else
#define SGM4154x_2_IINDPM_I_MAX_uA	3800000
#endif
#define SGM4154x_2_IINDPM_STEP_uA	    100000
#define SGM4154x_2_IINDPM_DEF_uA	    2400000

/* vindpm voltage  */
#define SGM4154x_2_VINDPM_V_MASK      GENMASK(3, 0)
#define SGM4154x_2_VINDPM_V_MIN_uV    3900000
#define SGM4154x_2_VINDPM_V_MAX_uV    12000000
#define SGM4154x_2_VINDPM_STEP_uV     100000
#define SGM4154x_2_VINDPM_DEF_uV	    3900000
#define SGM4154x_2_VINDPM_OS_MASK     GENMASK(1, 0)

/* DP DM SEL  */
#define SGM4154x_2_DP_VSEL_MASK       GENMASK(4, 3)
#define SGM4154x_2_DM_VSEL_MASK       GENMASK(2, 1)

/* PUMPX SET  */
#define SGM4154x_2_EN_PUMPX           BIT(7)
#define SGM4154x_2_PUMPX_UP           BIT(6)
#define SGM4154x_2_PUMPX_DN           BIT(5)

/* customer define jeita paramter */
#define JEITA_TEMP_ABOVE_T4_CV	0
#define JEITA_TEMP_T3_TO_T4_CV	4100000
#define JEITA_TEMP_T2_TO_T3_CV	4350000
#define JEITA_TEMP_T1_TO_T2_CV	4350000
#define JEITA_TEMP_T0_TO_T1_CV	0
#define JEITA_TEMP_BELOW_T0_CV	0

#define JEITA_TEMP_ABOVE_T4_CC_CURRENT	0
#define JEITA_TEMP_T3_TO_T4_CC_CURRENT	1000000
#define JEITA_TEMP_T2_TO_T3_CC_CURRENT	2400000
#define JEITA_TEMP_T1_TO_T2_CC_CURRENT	2000000
#define JEITA_TEMP_T0_TO_T1_CC_CURRENT	0
#define JEITA_TEMP_BELOW_T0_CC_CURRENT	0

#define TEMP_T4_THRES  50
#define TEMP_T4_THRES_MINUS_X_DEGREE 47
#define TEMP_T3_THRES  45
#define TEMP_T3_THRES_MINUS_X_DEGREE 39
#define TEMP_T2_THRES  20
#define TEMP_T2_THRES_PLUS_X_DEGREE 16
#define TEMP_T1_THRES  0
#define TEMP_T1_THRES_PLUS_X_DEGREE 6
#define TEMP_T0_THRES  0
#define TEMP_T0_THRES_PLUS_X_DEGREE  0
#define TEMP_NEG_10_THRES 0

struct sgm4154x_2_init_data {
	u32 ichg;	/* charge current		*/
	u32 ilim;	/* input current		*/
	u32 vreg;	/* regulation voltage		*/
	u32 iterm;	/* termination current		*/
	u32 iprechg;	/* precharge current		*/
	u32 vlim;	/* minimum system voltage limit */
	u32 max_ichg;
	u32 max_vreg;
	const char *chg_dev_name;
	char *irq_request_name;
	char *chg_en_request_name;
};

struct sgm4154x_2_state {
	bool vsys_stat;
	bool therm_stat;
	bool online;
	u8 chrg_stat;
	u8 vbus_status;

	bool chrg_en;
	bool hiz_en;
	bool term_en;
	bool vbus_gd;
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 ntc_fault;
};

struct sgm4154x_2_device {
	struct i2c_client *client;
	struct device *dev;
	struct mutex lock;
	struct mutex i2c_rw_lock;

	struct sgm4154x_2_init_data init_data;
	struct sgm4154x_2_state state;
	struct charger_device *chg_dev;
	struct regulator_dev *otg_rdev;
/*
	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *psy;
*/
	struct delayed_work charge_detect_delayed_work;
	struct delayed_work charge_monitor_work;
	struct wakeup_source *charger_wakelock;
	struct work_struct adapter_in_work;
	struct work_struct adapter_out_work;

	int chg_en_gpio;
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__)
	u8 is_sgm41511;
#endif
};

#endif /* _SGM4154x_2_CHARGER_H */
