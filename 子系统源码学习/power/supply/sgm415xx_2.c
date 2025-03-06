// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/types.h>
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#endif
//#include <mt-plat/mtk_boot.h>
//#include <mt-plat/upmu_common.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include "sgm415xx_2.h"
#include "charger_class.h"
//#include "mtk_charger.h"
#include "mtk_musb.h"
#include "charger_type.h"

/**********************************************************
 *
 *   [I2C Slave Setting]
 *
 *********************************************************/

#define SGM4154x_2_REG_NUM    (0xF)

/* SGM4154x_2 REG06 BOOST_LIM[5:4], uV */
static const unsigned int BOOST_VOLT_LIMIT[] = {
	4850000, 5000000, 5150000, 5300000
};
/* SGM4154x_2 REG02 BOOST_LIM[7:7], uA */
#if (defined(__SGM41542_CHIP_ID__) || defined(__SGM41541_CHIP_ID__)|| defined(__SGM41543_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	1200000, 2000000
};
#else
static const unsigned int BOOST_CURRENT_LIMIT[] = {
	500000, 1200000
};
#endif

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) )

static const unsigned int IPRECHG_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};

static const unsigned int ITERM_CURRENT_STABLE[] = {
	5000, 10000, 15000, 20000, 30000, 40000, 50000, 60000,
	80000, 100000, 120000, 140000, 160000, 180000, 200000, 240000
};
#endif

/*static enum power_supply_usb_type sgm4154x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
};*/

static const struct charger_properties sgm4154x_2_chg_props = {
	.alias_name = SGM4154x_2_NAME,
};

#define SGM_DEBUG_FLAG 1
#if SGM_DEBUG_FLAG
#define SGM_INFO(fmt, arg...)           pr_err("[wxw sgm] "fmt"\n", ##arg)
#else
#define SGM_INFO(fmt, arg...)
#endif

/**********************************************************
 *
 *   [Global Variable]
 *
 *********************************************************/

/**********************************************************
 *
 *   [I2C Function For Read/Write sgm4154x_2]
 *
 *********************************************************/
static int __sgm4154x_2_read_byte(struct sgm4154x_2_device *sgm, u8 reg, u8 *data)
{
    s32 ret;

#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
	if (sgm->is_sgm41511 && reg > SGM4154x_2_CHRG_CTRL_b) {
		return 0;
	}
#endif

    ret = i2c_smbus_read_byte_data(sgm->client, reg);
    if (ret < 0) {
        SGM_INFO("i2c read fail: can't read from reg 0x%02X\n", reg);
        return ret;
    }

    *data = (u8) ret;

    return 0;
}

static int __sgm4154x_2_write_byte(struct sgm4154x_2_device *sgm, int reg, u8 val)
{
    s32 ret;

#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
	if (sgm->is_sgm41511 && reg > SGM4154x_2_CHRG_CTRL_b) {
		return 0;
	}
#endif

    ret = i2c_smbus_write_byte_data(sgm->client, reg, val);
    if (ret < 0) {
        SGM_INFO("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
               val, reg, ret);
        return ret;
    }
    return 0;
}

static int sgm4154x_2_read_reg(struct sgm4154x_2_device *sgm, u8 reg, u8 *data)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_2_read_byte(sgm, reg, data);
	mutex_unlock(&sgm->i2c_rw_lock);

	return ret;
}

/*static int sgm4154x_2_write_reg(struct sgm4154x_2_device *sgm, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_2_write_byte(sgm, reg, val);
	mutex_unlock(&sgm->i2c_rw_lock);

	if (ret)
		pr_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	return ret;
}*/

static int sgm4154x_2_update_bits(struct sgm4154x_2_device *sgm, u8 reg,
		u8 mask, u8 val)
{
	int ret;
	u8 tmp;

	mutex_lock(&sgm->i2c_rw_lock);
	ret = __sgm4154x_2_read_byte(sgm, reg, &tmp);
	if (ret) {
		SGM_INFO("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= val & mask;

	ret = __sgm4154x_2_write_byte(sgm, reg, tmp);
	if (ret)
		SGM_INFO("Failed: reg=%02X, ret=%d\n", reg, ret);

out:
	mutex_unlock(&sgm->i2c_rw_lock);
	return ret;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/

static int sgm4154x_2_set_term_curr(struct sgm4154x_2_device *sgm, int uA)
{
	u8 reg_val;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) )

	for(reg_val = 1; reg_val < 16 && uA >= ITERM_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
if (sgm->is_sgm41511) {
	if (uA < SGM4154x_2_TERMCHRG_I_MIN_uA)
		uA = SGM4154x_2_TERMCHRG_I_MIN_uA;
	else if (uA > SGM4154x_2_TERMCHRG_I_MAX_uA)
		uA = SGM4154x_2_TERMCHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_2_TERMCHRG_I_MIN_uA) / SGM4154x_2_TERMCHRG_CURRENT_STEP_uA;
}
#endif
#else
	if (uA < SGM4154x_2_TERMCHRG_I_MIN_uA)
		uA = SGM4154x_2_TERMCHRG_I_MIN_uA;
	else if (uA > SGM4154x_2_TERMCHRG_I_MAX_uA)
		uA = SGM4154x_2_TERMCHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_2_TERMCHRG_I_MIN_uA) / SGM4154x_2_TERMCHRG_CURRENT_STEP_uA;
#endif

	return sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_3,
			SGM4154x_2_TERMCHRG_CUR_MASK, reg_val);
}

static int sgm4154x_2_set_prechrg_curr(struct sgm4154x_2_device *sgm, int uA)
{
	u8 reg_val;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) )
	for(reg_val = 1; reg_val < 16 && uA >= IPRECHG_CURRENT_STABLE[reg_val]; reg_val++)
		;
	reg_val--;
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
if (sgm->is_sgm41511) {
	if (uA < SGM4154x_2_PRECHRG_I_MIN_uA)
		uA = SGM4154x_2_PRECHRG_I_MIN_uA;
	else if (uA > SGM4154x_2_PRECHRG_I_MAX_uA)
		uA = SGM4154x_2_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_2_PRECHRG_I_MIN_uA) / SGM4154x_2_PRECHRG_CURRENT_STEP_uA;
}
#endif
#else
	if (uA < SGM4154x_2_PRECHRG_I_MIN_uA)
		uA = SGM4154x_2_PRECHRG_I_MIN_uA;
	else if (uA > SGM4154x_2_PRECHRG_I_MAX_uA)
		uA = SGM4154x_2_PRECHRG_I_MAX_uA;

	reg_val = (uA - SGM4154x_2_PRECHRG_I_MIN_uA) / SGM4154x_2_PRECHRG_CURRENT_STEP_uA;
#endif
	reg_val = reg_val << 4;
	return sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_3,
			SGM4154x_2_PRECHRG_CUR_MASK, reg_val);
}

static int sgm4154x_2_get_ichrg_curr(struct charger_device *chg_dev, unsigned int *ichrg)
{
	int ret;
	u8 ireg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_2, &ireg_val);
	if (ret)
		return ret;

	ireg_val = (ireg_val & SGM4154x_2_ICHRG_I_MASK);

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) )
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
#define SGM4154x_2_ICHRG_I_STEP_uA	    60000
if (sgm->is_sgm41511) {
	*ichrg = ireg_val * SGM4154x_2_ICHRG_I_STEP_uA;
} else {
#endif
	if (ireg_val <= 0x08)
		*ichrg = ireg_val * 5000;
	else if (ireg_val <= 0x0f)
		*ichrg = 40000 + (ireg_val - 0x08) * 10000;
	else if (ireg_val <= 0x17)
		*ichrg = 110000 + (ireg_val - 0x0f) * 20000;
	else if (ireg_val <= 0x20)
		*ichrg = 270000 + (ireg_val - 0x17) * 30000;
	else if (ireg_val <= 0x30)
		*ichrg = 540000 + (ireg_val - 0x20) * 60000;
	else if (ireg_val < 0x3d)
		*ichrg = 1500000 + (ireg_val - 0x30) * 120000;
	else
		*ichrg = 3000000;
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
}
#endif
#else
	*ichrg = ireg_val * SGM4154x_2_ICHRG_I_STEP_uA;
#endif

	return 0;
}

static int sgm4154x_2_set_ichrg_curr(struct charger_device *chg_dev, unsigned int uA)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	if (uA < SGM4154x_2_ICHRG_I_MIN_uA)
		uA = SGM4154x_2_ICHRG_I_MIN_uA;
	else if ( uA > sgm->init_data.max_ichg)
		uA = sgm->init_data.max_ichg;
#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) )
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
#define SGM4154x_2_ICHRG_I_STEP_uA	    60000
if (sgm->is_sgm41511) {
	reg_val = uA / SGM4154x_2_ICHRG_I_STEP_uA;
} else {
#endif
	if (uA <= 40000)
		reg_val = uA / 5000;
	else if (uA < 50000)
		reg_val = 0x08;
	else if (uA <= 110000)
		reg_val = 0x08 + (uA -40000) / 10000;
	else if (uA < 130000)
		reg_val = 0x0F;
	else if (uA <= 270000)
		reg_val = 0x0F + (uA -110000) / 20000;
	else if (uA < 300000)
		reg_val = 0x17;
	else if (uA <= 540000)
		reg_val = 0x17 + (uA -270000) / 30000;
	else if (uA < 600000)
		reg_val = 0x20;
	else if (uA <= 1500000)
		reg_val = 0x20 + (uA -540000) / 60000;
	else if (uA < 1620000)
		reg_val = 0x30;
	else if (uA <= 2940000)
		reg_val = 0x30 + (uA -1500000) / 120000;
	else
		reg_val = 0x3d;
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
}
#endif
#else
	reg_val = uA / SGM4154x_2_ICHRG_I_STEP_uA;
#endif
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_2,
			SGM4154x_2_ICHRG_I_MASK, reg_val);

	return ret;
}

static int sgm4154x_2_set_chrg_volt(struct charger_device *chg_dev, unsigned int chrg_volt)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	if (chrg_volt < SGM4154x_2_VREG_V_MIN_uV)
		chrg_volt = SGM4154x_2_VREG_V_MIN_uV;
	else if (chrg_volt > sgm->init_data.max_vreg)
		chrg_volt = sgm->init_data.max_vreg;

	reg_val = (chrg_volt-SGM4154x_2_VREG_V_MIN_uV) / SGM4154x_2_VREG_V_STEP_uV;
	reg_val = reg_val<<3;
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_4,
			SGM4154x_2_VREG_V_MASK, reg_val);

	return ret;
}

static int sgm4154x_2_get_chrg_volt(struct charger_device *chg_dev,unsigned int *volt)
{
	int ret;
	u8 vreg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_4, &vreg_val);
	if (ret)
		return ret;

	vreg_val = (vreg_val & SGM4154x_2_VREG_V_MASK)>>3;

	if (15 == vreg_val)
		*volt = 4352000; //default
	else if (vreg_val < 25)
		*volt = vreg_val*SGM4154x_2_VREG_V_STEP_uV + SGM4154x_2_VREG_V_MIN_uV;

	return 0;
}

/*static int sgm4154x_2_get_vindpm_offset_os(struct sgm4154x_2_device *sgm)
{
	int ret;
	u8 reg_val;

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_f, &reg_val);
	if (ret)
		return ret;

	reg_val = reg_val & SGM4154x_2_VINDPM_OS_MASK;

	return reg_val;
}

static int sgm4154x_2_set_vindpm_offset_os(struct sgm4154x_2_device *sgm,u8 offset_os)
{
	int ret;

	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_f,
			SGM4154x_2_VINDPM_OS_MASK, offset_os);

	if (ret){
		pr_err("%s fail\n",__func__);
		return ret;
	}

	return ret;
}*/
static int sgm4154x_2_set_input_volt_lim(struct charger_device *chg_dev, unsigned int vindpm)
{
	int ret;
	unsigned int offset;
	u8 reg_val;
	u8 os_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	if (vindpm < SGM4154x_2_VINDPM_V_MIN_uV ||
			vindpm > SGM4154x_2_VINDPM_V_MAX_uV)
		return -EINVAL;

	if (vindpm < 5900000){
		os_val = 0;
		offset = 3900000;
	}
	else if (vindpm >= 5900000 && vindpm < 7500000){
		os_val = 1;
		offset = 5900000; //uv
	}
	else if (vindpm >= 7500000 && vindpm < 10500000){
		os_val = 2;
		offset = 7500000; //uv
	}
	else{
		os_val = 3;
		offset = 10500000; //uv
	}

#if defined(__SGM41513_CHIP_ID__) || defined(__SGM41512_CHIP_ID__)
if (1){//sgm->is_sgm41511) {
	if (vindpm > 5400000)
		vindpm = 5400000;
	os_val = 0;
	offset = 3900000;
}
#endif
	SGM_INFO(" [wxw] %s os_val\n",__func__,os_val);
	/*sgm4154x_2_set_vindpm_offset_os(sgm,os_val);*/
	reg_val = (vindpm - offset) / SGM4154x_2_VINDPM_STEP_uV;

	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_6,
			SGM4154x_2_VINDPM_V_MASK, reg_val);

	return ret;
}

static int sgm4154x_2_set_input_curr_lim(struct charger_device *chg_dev, unsigned int iindpm)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	if (iindpm < SGM4154x_2_IINDPM_I_MIN_uA ||
			iindpm > SGM4154x_2_IINDPM_I_MAX_uA)
		return -EINVAL;

#if (defined(__SGM41513_CHIP_ID__) || defined(__SGM41513A_CHIP_ID__) || defined(__SGM41513D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) )
	reg_val = (iindpm-SGM4154x_2_IINDPM_I_MIN_uA) / SGM4154x_2_IINDPM_STEP_uA;
#else
	if (iindpm >= SGM4154x_2_IINDPM_I_MIN_uA && iindpm <= 3100000)//default
		reg_val = (iindpm-SGM4154x_2_IINDPM_I_MIN_uA) / SGM4154x_2_IINDPM_STEP_uA;
	else if (iindpm > 3100000 && iindpm < SGM4154x_2_IINDPM_I_MAX_uA)
		reg_val = 0x1E;
	else
		reg_val = SGM4154x_2_IINDPM_I_MASK;
#endif
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_0,
			SGM4154x_2_IINDPM_I_MASK, reg_val);
	return ret;
}

static int sgm4154x_2_get_input_curr_lim(struct charger_device *chg_dev,unsigned int *ilim)
{
	int ret;
	u8 reg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_0, &reg_val);
	if (ret)
		return ret;
	if (SGM4154x_2_IINDPM_I_MASK == (reg_val & SGM4154x_2_IINDPM_I_MASK))
		*ilim =  SGM4154x_2_IINDPM_I_MAX_uA;
	else
		*ilim = (reg_val & SGM4154x_2_IINDPM_I_MASK)*SGM4154x_2_IINDPM_STEP_uA + SGM4154x_2_IINDPM_I_MIN_uA;

	return 0;
}

static int sgm4154x_2_get_state(struct sgm4154x_2_device *sgm,
		struct sgm4154x_2_state *state)
{
	u8 chrg_stat;
	u8 fault;
	u8 chrg_param_0,chrg_param_1,chrg_param_2;
	int ret;

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_STAT, &chrg_stat);
		if (ret){
			SGM_INFO("%s read SGM4154x_2_CHRG_STAT fail\n",__func__);
			return ret;
		}
	}
	state->chrg_type = chrg_stat & SGM4154x_2_VBUS_STAT_MASK;
	state->chrg_stat = chrg_stat & SGM4154x_2_CHG_STAT_MASK;
	state->online = !!(chrg_stat & SGM4154x_2_PG_STAT);
	state->therm_stat = !!(chrg_stat & SGM4154x_2_THERM_STAT);
	state->vsys_stat = !!(chrg_stat & SGM4154x_2_VSYS_STAT);

	SGM_INFO("%s chrg_type =%d,chrg_stat =%d online = %d\n",__func__,state->chrg_type,state->chrg_stat,state->online);


	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_FAULT, &fault);
	if (ret){
		SGM_INFO("%s read SGM4154x_2_CHRG_FAULT fail\n",__func__);
		return ret;
	}
	state->chrg_fault = fault;
	state->ntc_fault = fault & SGM4154x_2_TEMP_MASK;
	state->health = state->ntc_fault;
	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_0, &chrg_param_0);
	if (ret){
		SGM_INFO("%s read SGM4154x_2_CHRG_CTRL_0 fail\n",__func__);
		return ret;
	}
	state->hiz_en = !!(chrg_param_0 & SGM4154x_2_HIZ_EN);

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_5, &chrg_param_1);
	if (ret){
		SGM_INFO("%s read SGM4154x_2_CHRG_CTRL_5 fail\n",__func__);
		return ret;
	}
	state->term_en = !!(chrg_param_1 & SGM4154x_2_TERM_EN);

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_a, &chrg_param_2);
	if (ret){
		SGM_INFO("%s read SGM4154x_2_CHRG_CTRL_a fail\n",__func__);
		return ret;
	}
	state->vbus_gd = !!(chrg_param_2 & SGM4154x_2_VBUS_GOOD);

	return 0;
}
#ifdef FIXME
static int sgm4154x_2_set_hiz_en(struct charger_device *chg_dev, bool hiz_en)
{
	u8 reg_val;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	dev_notice(sgm->dev, "%s:%d", __func__, hiz_en);
	reg_val = hiz_en ? SGM4154x_2_HIZ_EN : 0;

	return sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_0,
			SGM4154x_2_HIZ_EN, reg_val);
}
#endif

#define SGM4154x_2_TERMCHRG_I_uA	100000
#define SGM4154x_2_VREG_V_uV		4350000
static int sgm4154x_2_enable_charger(struct sgm4154x_2_device *sgm)
{
	int ret;

	ret = sgm4154x_2_set_term_curr(sgm, SGM4154x_2_TERMCHRG_I_uA);
	ret = sgm4154x_2_set_chrg_volt(sgm->chg_dev, SGM4154x_2_VREG_V_uV);

	gpio_direction_output(sgm->chg_en_gpio, 0);
	SGM_INFO("%s\n",__func__);
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, SGM4154x_2_CHRG_EN,
			SGM4154x_2_CHRG_EN);

	return ret;
}

static int sgm4154x_2_disable_charger(struct sgm4154x_2_device *sgm)
{
	int ret;
	SGM_INFO("%s\n",__func__);
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, SGM4154x_2_CHRG_EN,
			0);

	gpio_direction_output(sgm->chg_en_gpio, 0);
	return ret;
}

static bool is_charge_en = false;

static int sgm4154x_2_charging_switch(struct charger_device *chg_dev,bool enable)
{
	int ret;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);
	SGM_INFO("%s enable=%d\n",__func__,enable);
	if (enable){
                is_charge_en = true;
		ret = sgm4154x_2_enable_charger(sgm);
        }else{
		is_charge_en = false;
		ret = sgm4154x_2_disable_charger(sgm);
    }
	return ret;
}

static int sgm4154x_2_set_recharge_volt(struct sgm4154x_2_device *sgm, int mV)
{
	u8 reg_val;
	SGM_INFO("%s:%d", __func__, mV);
	reg_val = (mV - SGM4154x_2_VRECHRG_OFFSET_mV) / SGM4154x_2_VRECHRG_STEP_mV;

	return sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_4,
			SGM4154x_2_VRECHARGE, reg_val);
}

static int sgm4154x_2_set_wdt_rst(struct sgm4154x_2_device *sgm, bool is_rst)
{
	u8 val;

	if (is_rst)
		val = SGM4154x_2_WDT_RST_MASK;
	else
		val = 0;
	return sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1,
			SGM4154x_2_WDT_RST_MASK, val);
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm4154x_2_dump_register(struct charger_device *chg_dev)
{

	unsigned char i = 0;
	unsigned int ret = 0;
	unsigned char sgm4154x_2_reg[SGM4154x_2_REG_NUM] = { 0 };
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	for (i = 0; i < SGM4154x_2_REG_NUM; i++) {
		ret = sgm4154x_2_read_reg(sgm,i, &sgm4154x_2_reg[i]);
		if (ret) {
			SGM_INFO("[sgm4154x_2] i2c transfor error ret=%d\n", ret);
			return 1;
		}
		SGM_INFO("%s,[0x%x]=0x%x ",__func__, i, sgm4154x_2_reg[i]);
	}

	return 0;
}

/**********************************************************
 *
 *   [Internal Function]
 *
 *********************************************************/
static int sgm4154x_2_hw_chipid_detect(struct sgm4154x_2_device *sgm)
{
	int ret = 0;
	u8 val = 0;
	ret = sgm4154x_2_read_reg(sgm,SGM4154x_2_CHRG_CTRL_b,&val);
	if (ret < 0)
	{
		SGM_INFO("[%s] read SGM4154x_2_CHRG_CTRL_b fail\n", __func__);
		return ret;
	}
	val = val & SGM4154x_2_PN_MASK;
	SGM_INFO("[%s] Reg[0x0B]=0x%x\n", __func__,val);

	return val;
}

static int sgm4154x_2_reset_watch_dog_timer(struct charger_device
		*chg_dev)
{
	int ret;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	SGM_INFO("charging_reset_watch_dog_timer\n");

	ret = sgm4154x_2_set_wdt_rst(sgm,0x1);	/* RST watchdog */
	sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_5, GENMASK(5, 4), BIT(5) | BIT(4)); //160s

	return ret;
}


static int sgm4154x_2_get_charging_status(struct charger_device *chg_dev,
		bool *is_done)
{
	//struct sgm4154x_2_state state;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);
	//sgm4154x_2_get_state(sgm, &state);
#if IS_ENABLED(CONFIG_MT6366_DUMMY_CHARGER_TYPE_DETECT)
	u8 chrg_stat;
	int ret;
	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_STAT, &chrg_stat);
	if (ret){
		ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_STAT, &chrg_stat);
		if (ret){
			SGM_INFO("%s read SGM4154x_2_CHRG_STAT fail\n",__func__);
			chrg_stat = sgm->state.chrg_stat;
		}
	}
	sgm->state.chrg_stat = chrg_stat & SGM4154x_2_CHG_STAT_MASK;
#endif
	printk("sgm4154x chrg_stat = %d\n",sgm->state.chrg_stat);
	if (sgm->state.chrg_stat == SGM4154x_2_TERM_CHRG)
		*is_done = true;
	else
		*is_done = false;

	return 0;
}
static int sgm4154x_2_get_vbus(struct charger_device *chgdev, u32 *vbus)
{
	struct sgm4154x_2_device *sgm = charger_get_data(chgdev);
	printk("sgm4154x vbus_gd = %d\n",sgm->state.vbus_gd);
	if(sgm->state.vbus_gd)
		*vbus = 5000000;
	else
		*vbus = 0;
#if IS_ENABLED(CONFIG_MT6366_DUMMY_CHARGER_TYPE_DETECT)
	return -1;
#endif
	return *vbus;
}

static int sgm4154x_2_set_en_timer(struct sgm4154x_2_device *sgm)
{
	int ret;

	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_5,
			SGM4154x_2_SAFETY_TIMER_EN, SGM4154x_2_SAFETY_TIMER_EN);

	return ret;
}

static int sgm4154x_2_set_disable_timer(struct sgm4154x_2_device *sgm)
{
	int ret;

	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_5,
			SGM4154x_2_SAFETY_TIMER_EN, 0);

	return ret;
}

static int sgm4154x_2_enable_safetytimer(struct charger_device *chg_dev,bool en)
{
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);
	int ret = 0;

	if (en)
		ret = sgm4154x_2_set_en_timer(sgm);
	else
		ret = sgm4154x_2_set_disable_timer(sgm);
	return ret;
}

static int sgm4154x_2_get_is_safetytimer_enable(struct charger_device
		*chg_dev,bool *en)
{
	int ret = 0;
	u8 val = 0;

	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_2_read_reg(sgm,SGM4154x_2_CHRG_CTRL_5,&val);
	if (ret < 0)
	{
		SGM_INFO("[%s] read SGM4154x_2_CHRG_CTRL_5 fail\n", __func__);
		return ret;
	}
	*en = !!(val & SGM4154x_2_SAFETY_TIMER_EN);
	return 0;
}

#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
static int sgm4154x_2_en_pe_current_partern(struct charger_device
		*chg_dev,bool is_up)
{
	int ret = 0;

	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_d,
			SGM4154x_2_EN_PUMPX, SGM4154x_2_EN_PUMPX);
	if (ret < 0)
	{
		SGM_INFO("[%s] read SGM4154x_2_CHRG_CTRL_d fail\n", __func__);
		return ret;
	}
	if (is_up)
		ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_d,
				SGM4154x_2_PUMPX_UP, SGM4154x_2_PUMPX_UP);
	else
		ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_d,
				SGM4154x_2_PUMPX_DN, SGM4154x_2_PUMPX_DN);
	return ret;
}
#else
static int sgm4154x_2_send_ta_current_partern(struct charger_device *chg_dev, bool is_increase)
{
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);
	SGM_INFO("%s is_increase = %d\n", __func__, is_increase);

	sgm4154x_2_disable_charger(sgm);
	msleep(50);
	sgm4154x_2_set_ichrg_curr(chg_dev, 2520000);       /* Set charging current 2520ma */
	sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
	sgm4154x_2_enable_charger(sgm); /* Enable Charging */
	sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_6, GENMASK(7, 6), BIT(7)|BIT(6)); /* Set OVP 12V input */
	sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_a, GENMASK(1, 0), BIT(1)|BIT(0)); /*set reg0a bit 0 1 as 1*/

	if (is_increase == true) {
		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(281);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(281);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(281);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(485);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(50);

		sgm4154x_2_set_input_curr_lim(chg_dev, 2000000);	/* 2000mA */
		msleep(200);
	} else {
		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(281);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(281);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(281);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(85);

		sgm4154x_2_set_input_curr_lim(chg_dev, 800000);	/* 800mA */
		msleep(485);

		sgm4154x_2_set_input_curr_lim(chg_dev, 100000);	/* 100mA */
		msleep(50);

		sgm4154x_2_set_input_curr_lim(chg_dev, 2100000);	/* 2100mA */
	}

	return 0;
}
#endif

static DEFINE_MUTEX(sgm4154x_2_type_det_lock);

extern void Charger_Detect_Init(void);

static int is_plugin = 0;
static int charger_type_checking = 0;
static int sgm4154x_2_enable_chg_type_det(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm4154x_2_device *sgm = dev_get_drvdata(&chg_dev->dev);
	union power_supply_propval propval;
	struct power_supply *chrdet_psy;
	enum charger_type chg_type = CHARGER_UNKNOWN;
	enum power_supply_usb_type psy_usb_type;
	enum power_supply_type psy_type;
	int wait_cdp_cnt = 200;
      int wait_plugin_cnt = 10;
	static bool first_connect = true;
	struct sgm4154x_2_state state;

	SGM_INFO("%s  en=%d \n",__func__,en);
	chrdet_psy = power_supply_get_by_name("charger");
	if (!chrdet_psy) {
		SGM_INFO("%s: get power supply failed\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&sgm4154x_2_type_det_lock);
	if (en) {
		if (first_connect) {
			while (wait_cdp_cnt >= 0) {
				if (is_usb_rdy()) {
					SGM_INFO("%s CDP, PASS\n",__func__);
					break;
				}
				SGM_INFO("%s CDP, block\n",__func__);
				msleep(100);
				wait_cdp_cnt--;
			}
			if (wait_cdp_cnt <= 0)
				SGM_INFO("%s CDP, timeout\n",__func__);
			else
				SGM_INFO("%s CDP, free\n",__func__);
		}
		Charger_Detect_Init();
		ret = sgm4154x_2_get_state(sgm,&state);
		if (!state.chrg_type) charger_type_checking = 1;
		while (wait_plugin_cnt >= 0) {
			SGM_INFO("%s vbus_gd=%d\n",__func__,sgm->state.vbus_gd);
			if (!charger_type_checking/*sgm->state.vbus_gd*/)
				break;
			msleep(100);
			wait_plugin_cnt--;
		}
		
		ret = sgm4154x_2_get_state(sgm,&state);
                
         	mutex_lock(&sgm->lock);
	        sgm->state = state;
	        mutex_unlock(&sgm->lock);

	        if(!sgm->state.vbus_gd) {
		        SGM_INFO("%s Vbus not present, disable charge\n",__func__);
		        //sgm4154x_2_disable_charger(sgm);
		        //goto err;
	        }
	        if(!state.online)
	        {
		        SGM_INFO("%s Vbus not online\n",__func__);
		        //goto err;
	        }

               SGM_INFO("%s type=%d\n",__func__,sgm->state.chrg_type);
	        switch(sgm->state.chrg_type) {
	        case SGM4154x_2_USB_SDP:
		        SGM_INFO("%s charger type: SDP\n",__func__);
		        //curr_in_limit = 500000;
                        chg_type = STANDARD_HOST;
                        psy_type = POWER_SUPPLY_TYPE_USB;
                        psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		        break;

	        case SGM4154x_2_USB_CDP:
		        SGM_INFO("%s charger type: CDP\n",__func__);
		        //curr_in_limit = 1500000;
                        chg_type = CHARGING_HOST;
                        psy_type = POWER_SUPPLY_TYPE_USB_CDP;
                        psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		        break;

	        case SGM4154x_2_USB_DCP:
		        SGM_INFO("%s charger type: DCP\n",__func__);
		        //curr_in_limit = 2000000;
                        chg_type = STANDARD_CHARGER;
                        psy_type = POWER_SUPPLY_TYPE_USB_DCP;
                        psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		        break;

                case SGM4154x_2_NON_STANDARD:
	        case SGM4154x_2_UNKNOWN:
		        SGM_INFO("%s charger type: UNKNOWN\n",__func__);
		        //curr_in_limit = 500000;
		        		psy_type = POWER_SUPPLY_TYPE_USB;
                        chg_type = NONSTANDARD_CHARGER;
                        psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		        break;

	        default:
		        SGM_INFO("%s charger type: default\n",__func__);
		        //curr_in_limit = 500000;
		        //break;
		        		psy_type = POWER_SUPPLY_TYPE_UNKNOWN;
                        chg_type = CHARGER_UNKNOWN;
                        psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		        break;
	        }

	}

	if (chg_type != STANDARD_CHARGER) {
		Charger_Detect_Release();
	}

	propval.intval = 1;
	ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret) {
		SGM_INFO("[%s] set chr_online failed:%d\n", __func__, ret);
	}

	SGM_INFO("[%s] en:%d chg_type:%d\n", __func__, en, chg_type);
	ret = power_supply_get_property(chrdet_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0) {
		SGM_INFO("[%s]: get charge type failed, ret = %d\n", __func__, ret);
		mutex_unlock(&sgm4154x_2_type_det_lock);
		return ret;
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(chrdet_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0) {
		SGM_INFO("[%s]: set charge type failed, ret = %d\n", __func__, ret);
		mutex_unlock(&sgm4154x_2_type_det_lock);
		return ret;
	}
	propval.intval = psy_usb_type;
	ret = power_supply_set_property(chrdet_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &propval);
	if (ret < 0) {
		SGM_INFO("[%s]: set usb type failed, ret = %d\n", __func__, ret);
		mutex_unlock(&sgm4154x_2_type_det_lock);
		return ret;
	}
	propval.intval = psy_type;
	ret = power_supply_set_property(chrdet_psy,
			POWER_SUPPLY_PROP_TYPE, &propval);
	if (ret < 0) {
		SGM_INFO("[%s]: set power type failed, ret = %d\n", __func__, ret);
		mutex_unlock(&sgm4154x_2_type_det_lock);
		return ret;
	}
		/* usb */
		if ((chg_type == STANDARD_HOST) ||
			(chg_type == CHARGING_HOST) ||
			(chg_type == NONSTANDARD_CHARGER)) {
			//mt_usb_connect();

		} else {
			//mt_usb_disconnect();
		}	
	power_supply_changed(chrdet_psy);
	
	mutex_unlock(&sgm4154x_2_type_det_lock);
	return ret;
}
#if !defined(__SGM41513_CHIP_ID__)
static void charger_monitor_work_func(struct work_struct *work)
{
#if 0
	int ret = 0;
	struct sgm4154x_2_device * sgm = NULL;
	struct delayed_work *charge_monitor_work = NULL;
	//static u8 last_chg_method = 0;
	struct sgm4154x_2_state state;

	charge_monitor_work = container_of(work, struct delayed_work, work);
	if(charge_monitor_work == NULL) {
		SGM_INFO(" %s Cann't get charge_monitor_work\n",__func__);
		return ;
	}
	sgm = container_of(charge_monitor_work, struct sgm4154x_2_device, charge_monitor_work);
	if(sgm == NULL) {
		SGM_INFO("%s Cann't get sgm \n",__func__);
		return ;
	}

	ret = sgm4154x_2_get_state(sgm, &state);
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);

	if(!sgm->state.vbus_gd) {
		SGM_INFO(" %s Vbus not present, disable charge\n",__func__);
		sgm4154x_2_disable_charger(sgm);
		goto OUT;
	}
	if(!state.online)
	{
		SGM_INFO("%s Vbus not online\n",__func__);
		goto OUT;
	}
	sgm4154x_2_dump_register(sgm->chg_dev);
	SGM_INFO("%s\n",__func__);
OUT:
	schedule_delayed_work(&sgm->charge_monitor_work, 10*HZ);
#endif
}

static void charger_detect_work_func(struct work_struct *work)
{
	struct delayed_work *charge_detect_delayed_work = NULL;
	struct sgm4154x_2_device * sgm = NULL;
	u8 chrg_param;
	//static int charge_type_old = 0;
#if 0//(defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__))
	int curr_in_limit = 0;
#endif
	struct sgm4154x_2_state state;
	int ret;
	SGM_INFO("%s\n",__func__);
	charge_detect_delayed_work = container_of(work, struct delayed_work, work);
	if(charge_detect_delayed_work == NULL) {
		SGM_INFO("%s Cann't get charge_detect_delayed_work\n",__func__);
		return ;
	}
	sgm = container_of(charge_detect_delayed_work, struct sgm4154x_2_device, charge_detect_delayed_work);
	if(sgm == NULL) {
		SGM_INFO("%s Cann't get sgm4154x_2_device\n",__func__);
		return ;
	}

#if IS_ENABLED(CONFIG_MT6366_DUMMY_CHARGER_TYPE_DETECT)
	return;
#endif

	//if (!sgm->charger_wakelock->active)
	//	__pm_stay_awake(sgm->charger_wakelock);

	//ret = sgm4154x_2_get_state(sgm, &state);
	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_a, &chrg_param);
	if (ret){
		SGM_INFO("%s read SGM4154x_2_CHRG_CTRL_a fail\n",__func__);
		return;
	}
	state.vbus_gd = !!(chrg_param & SGM4154x_2_VBUS_GOOD);

	if ((state.vbus_gd && !is_plugin) || (state.vbus_gd && charger_type_checking)) {
		SGM_INFO("%s adapter plugged in\n",__func__);
                is_plugin = 1;
                if (charger_type_checking)
                        charger_type_checking = 0;
                else
		        schedule_work(&sgm->adapter_in_work);
	} else if (!state.vbus_gd && is_plugin) {
		SGM_INFO("%s adapter removed\n",__func__);
                is_plugin = 0;
                charger_type_checking = 0;
		schedule_work(&sgm->adapter_out_work);
	}
		
	mutex_lock(&sgm->lock);
	sgm->state = state;
	mutex_unlock(&sgm->lock);
	SGM_INFO("%s type=%d vbus_gd=%d\n",__func__,sgm->state.chrg_type,sgm->state.vbus_gd);
	//if(!sgm->state.vbus_gd) {
	//	printk(" [wxw] Vbus not present, disable charge\n");
	//	sgm4154x_2_disable_charger(sgm);
	//	goto err;
	//}
	//if(!state.online)
	//{
	//	printk(" [wxw] Vbus not online\n");
	//	goto err;
	//}
#if 0//(defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__)||defined(__SGM41512_CHIP_ID__))
        printk(" [wxw] %s type=%d vbus_gd=%d\n",__func__,sgm->state.chrg_type,sgm->state.vbus_gd);
	switch(sgm->state.chrg_type) {
	case SGM4154x_2_USB_SDP:
		printk(" [wxw] %s charger type: SDP\n",__func__);
		curr_in_limit = 500000;
		break;

	case SGM4154x_2_USB_CDP:
		printk(" [wxw] %s charger type: CDP\n",__func__);
		curr_in_limit = 1500000;
		break;

	case SGM4154x_2_USB_DCP:
		printk(" [wxw] %s charger type: DCP\n",__func__);
		curr_in_limit = 2000000;
		break;

	case SGM4154x_2_UNKNOWN:
		printk(" [wxw] %s charger type: UNKNOWN\n",__func__);
		curr_in_limit = 500000;
		break;

	default:
		printk(" [wxw] %s charger type: default\n",__func__);
		//curr_in_limit = 500000;
		//break;
		return;
	}

	//set charge parameters
	dev_err(sgm->dev, "Update: curr_in_limit = %d\n", curr_in_limit);
	sgm4154x_2_set_input_curr_lim(sgm->chg_dev, curr_in_limit);

#endif
	//enable charge
	//sgm4154x_2_enable_charger(sgm);
	//sgm4154x_2_dump_register(sgm->chg_dev);
	//return;
//err:
	//release wakelock
	//dev_err(sgm->dev, "Relax wakelock\n");
	//__pm_relax(sgm->charger_wakelock);
	return;
}

static irqreturn_t sgm4154x_2_irq_handler_thread(int irq, void *private)
{
	struct sgm4154x_2_device *sgm = private;

	//lock wakelock
	SGM_INFO("%s entry\n",__func__);

	schedule_delayed_work(&sgm->charge_detect_delayed_work, 100);

	return IRQ_HANDLED;
}
#endif

static int sgm4154x_2_hw_init(struct sgm4154x_2_device *sgm)
{
	int ret = 0;
	struct power_supply_battery_info bat_info = { };

	bat_info.constant_charge_current_max_ua =
		SGM4154x_2_ICHRG_I_DEF_uA;

	bat_info.constant_charge_voltage_max_uv =
		SGM4154x_2_VREG_V_uV;

	bat_info.precharge_current_ua =
		SGM4154x_2_PRECHRG_I_DEF_uA;

	bat_info.charge_term_current_ua =
		SGM4154x_2_TERMCHRG_I_uA;

	sgm->init_data.max_ichg =
		SGM4154x_2_ICHRG_I_MAX_uA;

	sgm->init_data.max_vreg =
		SGM4154x_2_VREG_V_MAX_uV;

	ret = sgm4154x_2_set_ichrg_curr(sgm->chg_dev,
			bat_info.constant_charge_current_max_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_2_set_prechrg_curr(sgm, bat_info.precharge_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_2_set_chrg_volt(sgm->chg_dev,
			bat_info.constant_charge_voltage_max_uv);
	if (ret)
		goto err_out;

	ret = sgm4154x_2_set_term_curr(sgm, bat_info.charge_term_current_ua);
	if (ret)
		goto err_out;

	ret = sgm4154x_2_set_input_curr_lim(sgm->chg_dev, sgm->init_data.ilim);
	if (ret)
		goto err_out;

	ret = sgm4154x_2_set_recharge_volt(sgm, 200);//100~200mv
	if (ret)
		goto err_out;

	SGM_INFO("%s ichrg_curr:%d prechrg_curr:%d chrg_vol:%d"
			" term_curr:%d input_curr_lim:%d",__func__,
			bat_info.constant_charge_current_max_ua,
			bat_info.precharge_current_ua,
			bat_info.constant_charge_voltage_max_uv,
			bat_info.charge_term_current_ua,
			sgm->init_data.ilim);

	return 0;

err_out:
	return ret;

}

static int sgm4154x_2_parse_dt(struct sgm4154x_2_device *sgm)
{
	int ret;
	int irq_gpio = 0, irqn = 0;
	int chg_en_gpio = 0;
	int len;

	ret = device_property_read_u32(sgm->dev,
			"input-voltage-limit-microvolt",
			&sgm->init_data.vlim);
	if (ret)
		sgm->init_data.vlim = SGM4154x_2_VINDPM_DEF_uV;

	if (sgm->init_data.vlim > SGM4154x_2_VINDPM_V_MAX_uV ||
			sgm->init_data.vlim < SGM4154x_2_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(sgm->dev,
			"input-current-limit-microamp",
			&sgm->init_data.ilim);
	if (ret)
		sgm->init_data.ilim = SGM4154x_2_IINDPM_DEF_uA;

	if (sgm->init_data.ilim > SGM4154x_2_IINDPM_I_MAX_uA ||
			sgm->init_data.ilim < SGM4154x_2_IINDPM_I_MIN_uA)
		return -EINVAL;

	if (of_property_read_string(sgm->dev->of_node, "chg_name",
		&sgm->init_data.chg_dev_name) < 0) {
		sgm->init_data.chg_dev_name = "primary_chg";
		dev_notice(sgm->dev, "%s: no charger name\n", __func__);
	}

	irq_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}
	len = strlen(dev_name(sgm->dev));
	sgm->init_data.irq_request_name = devm_kzalloc(sgm->dev, len + 14, GFP_KERNEL);
	snprintf(sgm->init_data.irq_request_name, len + 14, "%s_sgm41542_irq", dev_name(sgm->dev));
	ret = gpio_request(irq_gpio, sgm->init_data.irq_request_name);
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}
	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(sgm->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	sgm->client->irq = irqn;

	chg_en_gpio = of_get_named_gpio(sgm->dev->of_node, "sgm,chg-en-gpio", 0);
	if (!gpio_is_valid(chg_en_gpio))
	{
		dev_err(sgm->dev, "%s: %d gpio get failed\n", __func__, chg_en_gpio);
		return -EINVAL;
	}
	len = strlen(dev_name(sgm->dev));
	sgm->init_data.chg_en_request_name = devm_kzalloc(sgm->dev, len + 12, GFP_KERNEL);
	snprintf(sgm->init_data.chg_en_request_name, len + 12, "%s_sgm_chg_en", dev_name(sgm->dev));
	ret = gpio_request(chg_en_gpio, sgm->init_data.chg_en_request_name);
	if (ret) {
		dev_err(sgm->dev, "%s: %d gpio request failed\n", __func__, chg_en_gpio);
		return ret;
	}
	gpio_direction_output(chg_en_gpio, 0);//default disable charge
	sgm->chg_en_gpio = chg_en_gpio;
	return 0;
}

static int sgm4154x_2_enable_vbus_chg(struct sgm4154x_2_device *sgm)
{
	int ret = 0;
	//struct sgm4154x_2_device *sgm = rdev_get_drvdata(rdev);
	SGM_INFO("%s entry\n", __func__);
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, SGM4154x_2_OTG_EN,
			SGM4154x_2_OTG_EN);
	return ret;
}

static int sgm4154x_2_disable_vbus_chg(struct sgm4154x_2_device *sgm)
{
	int ret = 0;
	//struct sgm4154x_2_device *sgm = rdev_get_drvdata(rdev);
	SGM_INFO(" %s entry\n", __func__);
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, SGM4154x_2_OTG_EN,
			0);

	return ret;
}


static int sgm4154x_2_enable_vbus(struct regulator_dev *rdev)//struct sgm4154x_2_device *sgm)
{
	int ret = 0;
	struct sgm4154x_2_device *sgm = rdev_get_drvdata(rdev);
	SGM_INFO("%s entry\n", __func__);
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, SGM4154x_2_OTG_EN,
			SGM4154x_2_OTG_EN);
	return ret;
}

static int sgm4154x_2_disable_vbus(struct regulator_dev *rdev)//struct sgm4154x_2_device *sgm)
{
	int ret = 0;
	struct sgm4154x_2_device *sgm = rdev_get_drvdata(rdev);
	SGM_INFO(" %s entry\n", __func__);
	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, SGM4154x_2_OTG_EN,
			0);

	return ret;
}
#if 1
static int sgm4154x_2_is_enabled_vbus(struct regulator_dev *rdev)
{
	u8 temp = 0;
	int ret = 0;
	struct sgm4154x_2_device *sgm = rdev_get_drvdata(rdev);

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_1, &temp);
	return (temp&SGM4154x_2_OTG_EN)? 1 : 0;
}
#endif
static int sgm4154x_2_enable_otg(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
    struct sgm4154x_2_device *sgm = dev_get_drvdata(&chg_dev->dev);

	SGM_INFO("%s en = %d\n", __func__, en);
	if (en) {
		ret = sgm4154x_2_enable_vbus_chg(sgm);
	} else {
		ret = sgm4154x_2_disable_vbus_chg(sgm);
	}
	return ret;
}

static int sgm4154x_2_set_boost_current_limit(struct charger_device *chg_dev, u32 uA)
{
	int ret = 0;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	if (uA == BOOST_CURRENT_LIMIT[0]){
		ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_2, SGM4154x_2_BOOST_LIM,
				0);
	}

	else if (uA == BOOST_CURRENT_LIMIT[1]){
		ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_2, SGM4154x_2_BOOST_LIM,
				BIT(7));
	}
	return ret;
}
#if 1
static struct regulator_ops sgm4154x_2_vbus_ops = {
	.enable = sgm4154x_2_enable_vbus,
	.disable = sgm4154x_2_disable_vbus,
	.is_enabled = sgm4154x_2_is_enabled_vbus,
};

static const struct regulator_desc sgm4154x_2_otg_rdesc = {
	.of_match = "usb-otg-vbus",
	.name = "usb-otg-vbus",
	.ops = &sgm4154x_2_vbus_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 5000000,
	.n_voltages = 1,
};

static int sgm4154x_2_vbus_regulator_register(struct sgm4154x_2_device *sgm)
{
	struct regulator_config config = {};
	int ret = 0;
	/* otg regulator */
	config.dev = sgm->dev;
	config.driver_data = sgm;
	sgm->otg_rdev = devm_regulator_register(sgm->dev,
			&sgm4154x_2_otg_rdesc, &config);
	sgm->otg_rdev->constraints->valid_ops_mask |= REGULATOR_CHANGE_STATUS;
	if (IS_ERR(sgm->otg_rdev)) {
		ret = PTR_ERR(sgm->otg_rdev);
		pr_info("%s: register otg regulator failed (%d)\n", __func__, ret);
	}
	return ret;
}
#endif

static int sgm4154x_2_is_chip_enabled(struct charger_device *chg_dev, bool *en)
{
	SGM_INFO("%s charging_flag=%d\n",__func__,is_charge_en);
	*en = is_charge_en;
	return 0;
}

static int sgm4154x_2_plug_in(struct charger_device *chg_dev)
{
        int ret = 0;
	SGM_INFO("%s\n", __func__);
        ret = sgm4154x_2_charging_switch(chg_dev,true);
        if (ret < 0) {
                SGM_INFO("fail to enable charge!");
        }
	return ret;
}

static int sgm4154x_2_plug_out(struct charger_device *chg_dev)
{
	int ret = 0;
	SGM_INFO("%s\n", __func__);
        ret = sgm4154x_2_charging_switch(chg_dev,false);
        if (ret < 0) {
                SGM_INFO("fail to disable charge!");
        }
	return ret;
}

#if IS_ENABLED(CONFIG_MT6366_DUMMY_CHARGER_TYPE_DETECT)
enum sgm41512_pmu_chg_type {
	SGM41512_CHG_TYPE_NOVBUS = 0,
	SGM41512_CHG_TYPE_SDP,
	SGM41512_CHG_TYPE_CDP,
	SGM41512_CHG_TYPE_DCP,
	SGM41512_CHG_TYPE_UNKNOWN=5,
	SGM41512_CHG_TYPE_SDPNSTD,
	SGM41512_CHG_TYPE_OTG,
	SGM41512_CHG_TYPE_MAX,
};

struct chg_type_record{
	int chg_type;
	int chg_type_count;
};

static void sgm41512_force_dpdm_enable(struct sgm4154x_2_device *sgm)
{
	unsigned int ret = 0;

	ret = sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_7, BIT(7), BIT(7));
}

#define GETARRAYNUM(array) (ARRAY_SIZE(array))
static unsigned int sgm41512_get_vbus_stat(struct sgm4154x_2_device *sgm)
{
	unsigned int i, j, ret = 0;
	unsigned char val = 0;
	const int retry_count = 5;
	unsigned int unknown_count = 0,sdp_count = 0,cdp_count = 0,
					dcp_count = 0,sdpnstd_count = 0,otg_count = 0;
	struct chg_type_record sgm41512_chg_type_record[] = {
			{SGM41512_CHG_TYPE_SDP, sdp_count},
			{SGM41512_CHG_TYPE_CDP, cdp_count},
			{SGM41512_CHG_TYPE_DCP, dcp_count},
			{SGM41512_CHG_TYPE_SDPNSTD, sdpnstd_count},
			{SGM41512_CHG_TYPE_OTG, otg_count},
			{SGM41512_CHG_TYPE_UNKNOWN, unknown_count},
	};

	for (i = 0; i < retry_count; i++){
		ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_STAT, &val);
		val = (val >> 5) & 0x07;//VBUS_STAT
		for (j = 0; j < GETARRAYNUM(sgm41512_chg_type_record); j++) {
			if (val == sgm41512_chg_type_record[j].chg_type){
				sgm41512_chg_type_record[j].chg_type_count ++;
				if (sgm41512_chg_type_record[j].chg_type_count >= retry_count-2)
					return sgm41512_chg_type_record[j].chg_type;
				break;
			}
		}
	}
	return SGM41512_CHG_TYPE_NOVBUS;
}

static unsigned int sgm41512_get_iidet_status(struct sgm4154x_2_device *sgm)
{
	unsigned char val = 0;
	unsigned int ret = 0;

	ret = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_CTRL_7, &val);
	val = (val >> 7) & 0x01; //FORCE_DPDM

	return val;
}

static int sgm4154x_2_ext_chgdet(struct charger_device *chg_dev, bool en)
{
	unsigned int usb_status = 0,bq_detect_count=0;
	unsigned int iidet_bit=1;
	int bq_chg_type = CHARGER_UNKNOWN;
	struct sgm4154x_2_device *sgm = dev_get_drvdata(&chg_dev->dev);

	sgm4154x_2_dump_register(chg_dev);
	pr_info( "kernel_sgm41512_chg_type detect]\n");
	Charger_Detect_Init();

	sgm41512_force_dpdm_enable(sgm);
	//usb_status = sgm41512_get_vbus_stat();
	//pr_info("%s: usb_stats1 = 0x%X\n", __func__, usb_status);
	do{
		msleep(50);
		iidet_bit =sgm41512_get_iidet_status(sgm);
		bq_detect_count++;
		pr_info("%s: count_max=%d,iidet_bit=%d,usb_status =%d\n",
				__func__,bq_detect_count,iidet_bit,
				sgm41512_get_vbus_stat(sgm));
		if(bq_detect_count>16)
			iidet_bit = 0;
	}while (iidet_bit);
	usb_status = sgm41512_get_vbus_stat(sgm);
	pr_info("%s: usb_stats2 = 0x%X\n", __func__, usb_status);

	switch (usb_status) {
	case SGM41512_CHG_TYPE_SDP:
		bq_chg_type = STANDARD_HOST;
		break;
	case SGM41512_CHG_TYPE_UNKNOWN:
		bq_chg_type = NONSTANDARD_CHARGER;
		break;
	case SGM41512_CHG_TYPE_CDP:
		bq_chg_type = CHARGING_HOST;
		break;
	case SGM41512_CHG_TYPE_DCP:
		bq_chg_type = STANDARD_CHARGER;
		break;
	case SGM41512_CHG_TYPE_SDPNSTD:
		bq_chg_type = APPLE_2_1A_CHARGER;
		break;
	default:
		bq_chg_type = CHARGER_UNKNOWN;
		break;
	}
	pr_info("%s: bq_chg_type = %d\n", __func__, bq_chg_type);
	Charger_Detect_Release();
	return bq_chg_type;
}

#include "mtk_pe2.h"
static int sgm4154x_2_set_pep20_efficiency_table(struct charger_device *chg_dev)
{
	struct mtk_pe20 *pe2 = NULL;
	struct chg_alg_device *alg = NULL;
	pr_info("%s\n",__func__);
	alg = get_chg_alg_by_name("pe2");
	if (!alg) {
		pr_err("%s: get alg error\n", __func__);
		return -EINVAL;
	}
	pe2 = dev_get_drvdata(&alg->dev);
	if (!pe2) {
		pr_err("%s: get pe2 error\n", __func__);
		return -EINVAL;
	}

	pe2->profile[0].vchr = 8000000;
	pe2->profile[1].vchr = 8000000;
	pe2->profile[2].vchr = 8000000;
	pe2->profile[3].vchr = 8500000;
	pe2->profile[4].vchr = 8500000;
	pe2->profile[5].vchr = 8500000;
	pe2->profile[6].vchr = 9000000;
	pe2->profile[7].vchr = 9000000;
	pe2->profile[8].vchr = 9000000;
	pe2->profile[9].vchr = 9000000;
	return 0;
}

struct timespec64 ptime[13];
static int cptime[13][2];
static int dtime(int i)
{
	struct timespec64 time;

	time = timespec64_sub(ptime[i], ptime[i-1]);
	return time.tv_nsec/1000000;
}

#define PEOFFTIME 40
#define PEONTIME 90
static int sgm4154x_2_set_pep20_current_pattern(struct charger_device *chg_dev,
				u32 uV)
{
	int value;
	int i, j = 0;
	int flag;
	int errcnt = 0;
	printk("%s\n",__func__);
	sgm4154x_2_set_ichrg_curr(chg_dev, 2000000);

	mdelay(20);
	value = (uV - 5500000) / 500000;

	sgm4154x_2_set_input_curr_lim(chg_dev, 100000);
	mdelay(150);

	ktime_get_boottime_ts64(&ptime[j++]);
	for (i = 4; i >= 0; i--) {
		flag = value & (1 << i);
		if (flag == 0) {
			sgm4154x_2_set_input_curr_lim(chg_dev, 800000);
			mdelay(PEOFFTIME);
			ktime_get_boottime_ts64(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				errcnt = 1;
				return -1;
			}
			j++;
			sgm4154x_2_set_input_curr_lim(chg_dev, 100000);
			mdelay(PEONTIME);
			ktime_get_boottime_ts64(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				errcnt = 1;
				return -1;
			}
			j++;
		} else {
			sgm4154x_2_set_input_curr_lim(chg_dev, 800000);
			mdelay(PEONTIME);
			ktime_get_boottime_ts64(&ptime[j]);
			cptime[j][0] = PEONTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 90 || cptime[j][1] > 115) {
				errcnt = 1;
				return -1;
			}
			j++;
			sgm4154x_2_set_input_curr_lim(chg_dev, 100000);
			mdelay(PEOFFTIME);
			ktime_get_boottime_ts64(&ptime[j]);
			cptime[j][0] = PEOFFTIME;
			cptime[j][1] = dtime(j);
			if (cptime[j][1] < 30 || cptime[j][1] > 65) {
				errcnt = 1;
				return -1;
			}
			j++;
		}
	}

	sgm4154x_2_set_input_curr_lim(chg_dev, 800000);
	mdelay(200);
	ktime_get_boottime_ts64(&ptime[j]);
	cptime[j][0] = 200;
	cptime[j][1] = dtime(j);
	if (cptime[j][1] < 180 || cptime[j][1] > 240) {
		errcnt = 1;
		return -1;
	}
	j++;

	sgm4154x_2_set_input_curr_lim(chg_dev, 100000);
	mdelay(150);
	sgm4154x_2_set_input_curr_lim(chg_dev, 800000);
	//sgm4154x_2_set_input_curr_lim(chg_dev,2000000);
	mdelay(300);

	if (errcnt == 0)
		return 0;
	return -1;
}

static int sgm4154x_2_reset_ta(struct charger_device *chg_dev)
{
	pr_info("%s enter!\n", __func__);
	sgm4154x_2_set_ichrg_curr(chg_dev, 800000); //512mA

	sgm4154x_2_set_input_curr_lim(chg_dev, 100000);
	msleep(250);//250ms
	sgm4154x_2_set_input_curr_lim(chg_dev, 2000000);

	return 0;
}
#endif

static int sgm4154x_2_set_batfet_en(struct charger_device *chg_dev, bool en)
{
	int ret = 0;
	struct sgm4154x_2_device *sgm = charger_get_data(chg_dev);

	pr_info("%s en=%d\n", __func__, en);
	if (en) {
		//sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_7, GENMASK(3, 3), BIT(3));
		//sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_7, GENMASK(5, 5), 0);
	} else {
		sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_1, GENMASK(7, 7), 0);
		//sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_7, GENMASK(3, 3), 0);
		//sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_7, GENMASK(5, 5), BIT(5));
	}

	return ret;
}

static struct charger_ops sgm4154x_2_chg_ops = {
#ifdef FIXME
	.enable_hz = sgm4154x_2_set_hiz_en,
#endif
	.enable_hz = sgm4154x_2_set_batfet_en,

	/* cable plug in/out for primary charger */
	.plug_in = sgm4154x_2_plug_in,
	.plug_out = sgm4154x_2_plug_out,
	/* Normal charging */
	.dump_registers = sgm4154x_2_dump_register,
	.enable = sgm4154x_2_charging_switch,
	.is_chip_enabled = sgm4154x_2_is_chip_enabled,
	.get_charging_current = sgm4154x_2_get_ichrg_curr,
	.set_charging_current = sgm4154x_2_set_ichrg_curr,
	.get_input_current = sgm4154x_2_get_input_curr_lim,
	.set_input_current = sgm4154x_2_set_input_curr_lim,
	.get_constant_voltage = sgm4154x_2_get_chrg_volt,
	.set_constant_voltage = sgm4154x_2_set_chrg_volt,
	.kick_wdt = sgm4154x_2_reset_watch_dog_timer,
	.set_mivr = sgm4154x_2_set_input_volt_lim,
	.is_charging_done = sgm4154x_2_get_charging_status,
	.get_vbus_adc = sgm4154x_2_get_vbus,
	/* Safety timer */
	.enable_safety_timer = sgm4154x_2_enable_safetytimer,
	.is_safety_timer_enabled = sgm4154x_2_get_is_safetytimer_enable,

	/* Power path */
	/*.enable_powerpath = sgm4154x_2_enable_power_path, */
	/*.is_powerpath_enabled = sgm4154x_2_get_is_power_path_enable, */

	/* OTG */
	.enable_otg = sgm4154x_2_enable_otg,
	.set_boost_current_limit = sgm4154x_2_set_boost_current_limit,
	//.event = sgm4154x_2_do_event,

	/* charger type detection */
	//.enable_chg_type_det = sgm4154x_2_enable_chg_type_det,
#if IS_ENABLED(CONFIG_MT6366_DUMMY_CHARGER_TYPE_DETECT)
	.enable_chg_type_det = sgm4154x_2_ext_chgdet,
#endif

	/* PE+/PE+20 */
#if (defined(__SGM41542_CHIP_ID__)|| defined(__SGM41516D_CHIP_ID__)|| defined(__SGM41543D_CHIP_ID__))
	.send_ta_current_pattern = sgm4154x_2_en_pe_current_partern,
#else
	.send_ta_current_pattern = sgm4154x_2_send_ta_current_partern,
#endif
#if IS_ENABLED(CONFIG_MT6366_DUMMY_CHARGER_TYPE_DETECT)
	.set_pe20_efficiency_table = sgm4154x_2_set_pep20_efficiency_table,
	.send_ta20_current_pattern = sgm4154x_2_set_pep20_current_pattern,
	.reset_ta = sgm4154x_2_reset_ta,
#else
	.set_pe20_efficiency_table = NULL,
	.send_ta20_current_pattern = NULL,
#endif
	//	.set_ta20_reset = NULL,
	.enable_cable_drop_comp = NULL,
};

static void charge_adapter_in_workfunc(struct work_struct *work)
{
	struct sgm4154x_2_device *sgm = container_of(work, struct sgm4154x_2_device, adapter_in_work);
	int ret = 0;
	struct power_supply *chrdet_psy;
	union power_supply_propval propval;

	sgm4154x_2_enable_charger(sgm);
	
	chrdet_psy = power_supply_get_by_name("charger");
	if (chrdet_psy) {
		propval.intval = 1;
		/*ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret) {
			SGM_INFO("[%s] set chr_online failed:%d\n", __func__, ret);
			return;
		}*/
		ret = sgm4154x_2_enable_chg_type_det(sgm->chg_dev, true);
		if (ret)
			SGM_INFO("[%s] update chr_type failed:%d\n", __func__, ret);

		//bq2589x_set_hz(bq, 0);
	} else {
		SGM_INFO("[%s] get chrdet_psy failed:%d\n", __func__, ret);
	}
	//if (pe.enable)
		//schedule_delayed_work(&bq->monitor_work, 0);
}

static void charge_adapter_out_workfunc(struct work_struct *work)
{
	//struct sgm4154x_2_device *sgm = container_of(work, struct sgm4154x_2_device, adapter_out_work);
	int ret = 0;
	struct power_supply *chrdet_psy;
	union power_supply_propval propval;

	//ret = bq2589x_set_input_volt_limit(bq, 4600);
	//if (ret < 0)
	//	dev_info(bq->dev, "%s:reset vindpm threshold to 4400 failed:%d\n", __func__, ret);
	//else
	//	dev_info(bq->dev, "%s:reset vindpm threshold to 4400 successfully\n", __func__);

	//if (pe.enable)
	//	cancel_delayed_work_sync(&bq->monitor_work);

	//cancel_delayed_work_sync(&bq->pe_volt_tune_work);
	//bq2589x_reset_pe_param();
	//__pm_relax(&bq->pe_tune_wakelock);
	//sgm4154x_2_disable_charger(sgm);

	chrdet_psy = power_supply_get_by_name("charger");
	if (chrdet_psy) {
		propval.intval = CHARGER_UNKNOWN;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret)
			SGM_INFO("[%s] reset chr_type failed:%d\n", __func__, ret);
		propval.intval = 0;
		ret = power_supply_set_property(chrdet_psy,
				POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret)
			SGM_INFO("[%s] reset chr_online failed:%d\n", __func__, ret);
	} else {
		SGM_INFO("[%s] get chrdet_psy failed:%d\n", __func__, ret);
	}
}
/*
static enum power_supply_property sgm4154x_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
};
static int sgm4154x_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mt_charger *sgm = power_supply_get_drvdata(psy);
	int vbus = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		vbus = sgm4154x_2_read_reg(sgm, SGM4154x_2_CHRG_STAT, &chrg_stat);;
		printk("sgm4154x charger status = %d\n",vbus);
		if(vbus == 0)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else if(vbus == 1 || vbus == 2)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if(vbus == 3)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
int sgm4154x_charger_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		printk("fuyiye do nothing\n");
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int sgm4154x_charger_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return 1;
	default:
		return 0;
	}
}
static char *sgm4154x_charger_supplied_to[] = {
	"battery",
	"mtk-master-charger"
};
*/
static int sgm4154x_2_driver_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct device *dev = &client->dev;
	struct sgm4154x_2_device *sgm;

	char *name = NULL;

	SGM_INFO("[%s] %s probe\n", __func__, dev_name(dev));

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->client = client;
	sgm->dev = dev;

	mutex_init(&sgm->lock);
	mutex_init(&sgm->i2c_rw_lock);

	i2c_set_clientdata(client, sgm);

	ret = sgm4154x_2_parse_dt(sgm);
	if (ret)
		return ret;

#if defined(__SGM41513_CHIP_ID__)
	sgm->client->addr = 0x6b;
	sgm->is_sgm41511 = 1;
	ret = sgm4154x_2_hw_chipid_detect(sgm);
	if (ret != SGM41511_PN_ID) {
		sgm->client->addr = 0x1a;
		sgm->is_sgm41511 = 0;
		ret = sgm4154x_2_hw_chipid_detect(sgm);
		if (ret != SGM4154x_2_PN_ID) {
			SGM_INFO("[%s] device not found !!!\n", __func__);
			return ret;
		}
		SGM_INFO("[%s] sgm41513 detect !!!\n", __func__);
	} else {
		SGM_INFO("[%s] sgm41511 detect !!!\n", __func__);
	}
#else
	ret = sgm4154x_2_hw_chipid_detect(sgm);
	if (ret == 0){
		SGM_INFO("[%s] device not found !!!\n", __func__);
		return ret;
	}
#endif

	name = devm_kasprintf(sgm->dev, GFP_KERNEL, "%s","sgm4154x_2 suspend wakelock");
	sgm->charger_wakelock =	wakeup_source_register(NULL, name);

	/* Register charger device */
	sgm->chg_dev = charger_device_register(sgm->init_data.chg_dev_name,
			&client->dev, sgm,
			&sgm4154x_2_chg_ops,
			&sgm4154x_2_chg_props);
	if (IS_ERR_OR_NULL(sgm->chg_dev)) {
		SGM_INFO("%s: register charger device  failed\n", __func__);
		ret = PTR_ERR(sgm->chg_dev);
		return ret;
	}

	ret = sgm4154x_2_hw_init(sgm);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
	}
#if defined(__SGM41513_CHIP_ID__)||defined(__SGM41512_CHIP_ID__) 
	sgm4154x_2_update_bits(sgm, SGM4154x_2_CHRG_CTRL_5, GENMASK(5, 4), 0); //disable wdt
	if (0 != strcmp("primary_chg", sgm->init_data.chg_dev_name)) {
		sgm4154x_2_disable_charger(sgm);// disable others except primary_chg
	}
	sgm4154x_2_dump_register(sgm->chg_dev);
#endif
/*
	sgm->psy_desc.name = "sgm415xx";
	sgm->psy_desc.type			= POWER_SUPPLY_TYPE_UNKNOWN,
	sgm->psy_desc.properties		= sgm4154x_charger_properties,
	sgm->psy_desc.num_properties		= ARRAY_SIZE(sgm4154x_charger_properties),
	sgm->psy_desc.get_property		= sgm4154x_charger_get_property,
	sgm->psy_desc.set_property		= sgm4154x_charger_set_property,
	sgm->psy_desc.property_is_writeable	= sgm4154x_charger_property_is_writeable,

	sgm->psy_cfg.drv_data = sgm;
	sgm->psy_cfg.of_node = client->dev.of_node;;
	sgm->psy_cfg.supplied_to = sgm4154x_charger_supplied_to;
	sgm->psy_cfg.num_supplicants = ARRAY_SIZE(sgm4154x_charger_supplied_to);
	sgm->psy = power_supply_register(&client->dev, &sgm->psy_desc,
			&sgm->psy_cfg);
	if (IS_ERR(sgm->psy)) {
		pr_notice("%s Failed to register power supply: %ld\n",
			__func__, PTR_ERR(sgm->psy));
		return PTR_ERR(sgm->psy);
	}
*/

#if !defined(__SGM41513_CHIP_ID__)
	INIT_DELAYED_WORK(&sgm->charge_detect_delayed_work, charger_detect_work_func);
	INIT_DELAYED_WORK(&sgm->charge_monitor_work, charger_monitor_work_func);
	INIT_WORK(&sgm->adapter_in_work, charge_adapter_in_workfunc);
	INIT_WORK(&sgm->adapter_out_work, charge_adapter_out_workfunc);
	schedule_delayed_work(&sgm->charge_detect_delayed_work,0);//for plug in charger bootup detect charger
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
				sgm4154x_2_irq_handler_thread,
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
				dev_name(&client->dev), sgm);
		if (ret)
			return ret;
		enable_irq_wake(client->irq);
	}
#endif

	//OTG setting
	//sgm4154x_2_set_otg_voltage(sgm->chg_dev, 5000000); //5V
	//sgm4154x_2_set_otg_current(sgm->chg_dev, 1200000); //1.2A

	ret = sgm4154x_2_vbus_regulator_register(sgm);
#if defined(__SGM41513_CHIP_ID__)
	schedule_delayed_work(&sgm->charge_monitor_work,100);
#endif

	return ret;

}

static int sgm4154x_2_charger_remove(struct i2c_client *client)
{
	struct sgm4154x_2_device *sgm = i2c_get_clientdata(client);

#if defined(__SGM41513_CHIP_ID__)
	cancel_delayed_work_sync(&sgm->charge_monitor_work);
#endif

	regulator_unregister(sgm->otg_rdev);

	mutex_destroy(&sgm->lock);
	mutex_destroy(&sgm->i2c_rw_lock);

	return 0;
}

static void sgm4154x_2_charger_shutdown(struct i2c_client *client)
{
	int ret = 0;

	struct sgm4154x_2_device *sgm = i2c_get_clientdata(client);
	ret = sgm4154x_2_disable_charger(sgm);
	if (ret) {
		SGM_INFO("Failed to disable charger, ret = %d\n", ret);
	}
	cancel_work_sync(&sgm->adapter_in_work);
	cancel_work_sync(&sgm->adapter_out_work);
	SGM_INFO("sgm4154x_2_charger_shutdown\n");
}

static const struct i2c_device_id sgm4154x_2_i2c_ids[] = {
	{ "sgm41541", 0 },
	{ "sgm41542", 1 },
	{ "sgm41543", 2 },
	{ "sgm41543D", 3 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sgm4154x_2_i2c_ids);

static const struct of_device_id sgm4154x_2_of_match[] = {
	{ .compatible = "sgm,sgm41541", },
	{ .compatible = "sgm,sgm41542", },
    { .compatible = "sgm,sgm41512", },
	{ .compatible = "sgm,sgm41543", },
	{ .compatible = "sgm,sgm41543D", },
	{ },
};
MODULE_DEVICE_TABLE(of, sgm4154x_2_of_match);

static struct i2c_driver sgm4154x_2_driver = {
	.driver = {
		.name = "sgm4154x_2-charger",
		.of_match_table = sgm4154x_2_of_match,
	},
	.probe = sgm4154x_2_driver_probe,
	.remove = sgm4154x_2_charger_remove,
	.shutdown = sgm4154x_2_charger_shutdown,
	.id_table = sgm4154x_2_i2c_ids,
};
module_i2c_driver(sgm4154x_2_driver);

MODULE_AUTHOR(" qhq <Allen_qin@sg-micro.com>");
MODULE_DESCRIPTION("sgm4154x_2 charger driver");
MODULE_LICENSE("GPL v2");
