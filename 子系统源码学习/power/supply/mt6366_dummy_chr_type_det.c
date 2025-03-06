/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
//#include <linux/pm_wakeup.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>
#include <linux/power_supply.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/iio/consumer.h>
#include <linux/mfd/mt6397/core.h>

#include "mtk_battery.h"
#include "charger_type.h"
#include "charger_class.h"
#if 1
#include <linux/suspend.h>
#include <tcpm.h>
#endif

static bool dbg_log_en;
module_param(dbg_log_en, bool, 0644);
#define mt_dbg(dev, fmt, ...) \
	do { \
		if (dbg_log_en) \
			dev_info(dev, "%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#if 1
#define DROI_USING_CHARGER_CLASS_INTF
#endif

enum boot_mode {
    NORMAL_BOOT                    = 0,
    META_BOOT                      = 1,
    RECOVERY_BOOT                  = 2,
    FACTORY_BOOT                   = 4,
    ALARM_BOOT                     = 7,
    KERNEL_POWER_OFF_CHARGING_BOOT = 8,
    FASTBOOT_BOOT = 99
};

struct chrdet_tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

static unsigned int get_boot_mode_from_dts(void)
{
	struct device_node *np_chosen = NULL;
	struct chrdet_tag_bootmode *tag = NULL;

	np_chosen = of_find_node_by_path("/chosen");
	if (!np_chosen) {
		pr_notice("log_store: warning: not find node: '/chosen'\n");

		np_chosen = of_find_node_by_path("/chosen@0");
		if (!np_chosen) {
			pr_notice("log_store: warning: not find node: '/chosen@0'\n");
			return NORMAL_BOOT;
		}
	}

	tag = (struct chrdet_tag_bootmode *)
			of_get_property(np_chosen, "atag,boot", NULL);
	if (!tag) {
		pr_notice("log_store: error: not find tag: 'atag,boot';\n");
		return NORMAL_BOOT;
	}

	pr_notice("log_store: bootmode: 0x%x boottype: 0x%x.\n",
		tag->bootmode, tag->boottype);

	return tag->bootmode;
}

struct mt6366_chrdet_data {
	enum charger_type g_chr_type;
	struct work_struct chr_work;
	struct mutex chrdet_lock;
#if defined(DROI_USING_CHARGER_CLASS_INTF)
	struct charger_device *primary_charger;
	int first_connect;
#endif

	struct device *dev;
	struct regmap *rmap;
	struct power_supply *chrdet_psy;
	struct power_supply_desc psy_desc1;
	struct power_supply *chrtyp_psy;
	struct power_supply_desc psy_desc2;

	struct iio_channel *chan_vbus_voltage;

	int chg_online;
	enum charger_type chg_type;
	enum power_supply_usb_type psy_usb_type;
	int vbat0_flag;

#if 1
	/* typec notify */
	struct tcpc_device *tcpc_dev;
	struct notifier_block pd_nb;
	/* chg det */
	wait_queue_head_t attach_wq;
	atomic_t chrdet_start;
	struct task_struct *attach_task;
	struct mutex attach_lock;
	int typec_attach;
	/* suspend notify */
	struct notifier_block pm_nb;
	bool pd_rdy;
	bool is_plugin;
#endif
};

#if 0
#define PMIC_BASE (0x0)
#define MT6358_CHR_CON0  ((unsigned int)(PMIC_BASE+0xa88))
#define PMIC_RGS_CHRDET_ADDR MT6358_CHR_CON0
#define PMIC_RGS_CHRDET_MASK 0x1
#define PMIC_RGS_CHRDET_SHIFT   5
inline u32 chrdet_read(struct regmap *rmap, u32 addr)
{
	u32 val = 0;

	regmap_read(rmap, addr, &val);
	return val;
}

inline u32 chrdet_read_bits(struct regmap *rmap, u32 addr, u32 shift, u32 mask)
{
	u32 val = 0;

	val = chrdet_read(rmap, addr);
	return ((val>>shift) & mask);
}

static u32 pmic_get_register_value_rgs_chrdet(struct regmap *rmap)
{
	return chrdet_read_bits(rmap, PMIC_RGS_CHRDET_ADDR, PMIC_RGS_CHRDET_SHIFT, PMIC_RGS_CHRDET_MASK);
}
#endif

const struct of_device_id mt6366_chrdet_of_match[] = {
	{
		.compatible = "mediatek,mt6366-charger-type",
	}, {
		/* sentinel */
	},
};

static int chrdet_inform_psy_changed(struct mt6366_chrdet_data *ddata, enum charger_type chg_type,
				bool chg_online)
{
	int ret = 0;
	union power_supply_propval propval;

	dev_info(ddata->dev, "charger type: %s: online = %d, type = %d\n",
		__func__, chg_online, chg_type);

	/* Inform chg det power supply */
	if (chg_online) {
		propval.intval = chg_online;
		ret = power_supply_set_property(ddata->chrdet_psy,
			POWER_SUPPLY_PROP_ONLINE, &propval);
		if (ret < 0)
			dev_info(ddata->dev, "%s: psy online failed, ret = %d\n",
				__func__, ret);

		propval.intval = chg_type;
		ret = power_supply_set_property(ddata->chrdet_psy,
			POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
		if (ret < 0)
			dev_info(ddata->dev, "%s: psy type failed, ret = %d\n",
				__func__, ret);

		return ret;
	}

	propval.intval = chg_type;
	ret = power_supply_set_property(ddata->chrdet_psy,
		POWER_SUPPLY_PROP_CHARGE_TYPE, &propval);
	if (ret < 0)
		dev_info(ddata->dev, "%s: psy type failed, ret(%d)\n",
			__func__, ret);

	propval.intval = chg_online;
	ret = power_supply_set_property(ddata->chrdet_psy,
		POWER_SUPPLY_PROP_ONLINE, &propval);
	if (ret < 0)
		dev_info(ddata->dev, "%s: psy online failed, ret(%d)\n",
			__func__, ret);
	return ret;
}

static int hw_charging_get_charger_type(struct mt6366_chrdet_data *ddata)
{
#if !defined(DROI_USING_CHARGER_CLASS_INTF)
	return STANDARD_HOST;
#else
	int ret = -1;
	int timeout = 10;
	int boot_mode = get_boot_mode_from_dts();

	dev_info(ddata->dev, "%s: ysj_hw_bc11_init boot_mode = %d，timeout=%d\n", __func__, boot_mode, timeout);

	msleep(200);
	if (boot_mode != RECOVERY_BOOT) {
		if (ddata->first_connect == true) {
			dev_err(ddata->dev, "ysj is_usb_rdy=%d\n", is_usb_rdy());
			if (is_usb_rdy() == false) {
				while (is_usb_rdy() == false && timeout > 0) {
					msleep(100);
					timeout--;
				}
				if (timeout == 0)
					dev_info(ddata->dev, "CDP, timeout\n");
				else
					dev_info(ddata->dev, "CDP, free\n");
			} else
				dev_info(ddata->dev, "CDP, pass\n");
			ddata->first_connect = false;
		}
	}

	ret = charger_dev_enable_chg_type_det(ddata->primary_charger, 1);
	if (ret < 0) {
		dev_err(ddata->dev, "charger_dev_enable_chg_type_det failed\n");
		return CHARGER_UNKNOWN;
	}
	return ret;
#endif
}

#if 1
#define FAST_CHG_WATT 7500000 /* mW */
/* map with charger.c */
enum attach_type {
	ATTACH_TYPE_NONE,
	ATTACH_TYPE_PWR_RDY,
	ATTACH_TYPE_TYPEC,
	ATTACH_TYPE_PD,
	ATTACH_TYPE_PD_SDP,
	ATTACH_TYPE_PD_DCP,
	ATTACH_TYPE_PD_NONSTD,
};

static int typec_attach_thread(void *data)
{
	struct mt6366_chrdet_data *ddata = data;
	int ret = 0, attach;
	//union power_supply_propval val;

	pr_info("%s: ++\n", __func__);
	while (!kthread_should_stop()) {
		if (ddata == NULL) {
			pr_notice("%s: ddata is null\n", __func__);
			return -ENODEV;
		}
		ret = wait_event_interruptible(ddata->attach_wq,
			   atomic_read(&ddata->chrdet_start) > 0 ||
							 kthread_should_stop());
		if (ret == -ERESTARTSYS) {
			pr_notice("%s: error when wait_event_interruptible\n",
				__func__);
			break;
		}
		if (ret < 0) {
			pr_notice("%s: wait event been interrupted(%d)\n",
				  __func__, ret);
			continue;
		}
		if (kthread_should_stop())
			break;
		mutex_lock(&ddata->attach_lock);
		attach = ddata->typec_attach;
		atomic_set(&ddata->chrdet_start, 0);
		mutex_unlock(&ddata->attach_lock);
		pr_notice("%s attach:%d\n", __func__, attach);

#if 0
		val.intval = attach;
		ret = power_supply_set_property(ddata->bc12_psy,
					POWER_SUPPLY_PROP_ONLINE, &val);
		if (ret < 0)
			dev_info(ddata->dev,
				 "%s: fail to set online(%d)\n", __func__, ret);
#else
		ddata->is_plugin = !!attach;
		schedule_work(&ddata->chr_work);
#endif
	}
	return ret;
}

static void handle_typec_pd_attach(struct mt6366_chrdet_data *ddata, int attach)
{
	mutex_lock(&ddata->attach_lock);
	ddata->typec_attach = attach;
	ddata->pd_rdy = attach ? ddata->pd_rdy : false;
	atomic_inc(&ddata->chrdet_start);
	wake_up_interruptible(&ddata->attach_wq);
	mutex_unlock(&ddata->attach_lock);
}

static void handle_pd_rdy_attach(struct mt6366_chrdet_data *ddata, struct tcp_notify *noti)
{
	int attach = 0, usb_comm = 0, watt = 0, pd_rdy = 0;
	struct tcpm_remote_power_cap cap;
	memset(&cap, 0, sizeof(cap));

	if (noti->pd_state.connected == PD_CONNECT_PE_READY_SNK ||
	    noti->pd_state.connected == PD_CONNECT_PE_READY_SNK_PD30 ||
	    noti->pd_state.connected == PD_CONNECT_PE_READY_SNK_APDO) {
		mutex_lock(&ddata->attach_lock);
		pd_rdy = ddata->pd_rdy;
		if (pd_rdy) {
			dev_info(ddata->dev,
				 "%s: pd_rdy is already done\n", __func__);
			mutex_unlock(&ddata->attach_lock);
			return;
		}
		ddata->pd_rdy = true;
		mutex_unlock(&ddata->attach_lock);

		usb_comm = tcpm_is_comm_capable(ddata->tcpc_dev);
		tcpm_get_remote_power_cap(ddata->tcpc_dev, &cap);
		watt = cap.max_mv[0] * cap.ma[0];
		dev_info(ddata->dev, "%s: mv:%d, ma:%d, watt: %d\n",
			 __func__, cap.max_mv[0], cap.ma[0], watt);

		if (usb_comm)
			attach = ATTACH_TYPE_PD_SDP;
		else
			attach = watt >= FAST_CHG_WATT ? ATTACH_TYPE_PD_DCP :
				 ATTACH_TYPE_PD_NONSTD;
		pr_notice("%s: pd_attach:%d\n", __func__, attach);
		handle_typec_pd_attach(ddata, attach);
	}
}

static void handle_audio_attach(struct mt6366_chrdet_data *ddata, struct tcp_notify *noti)
{
	int attach;
	uint8_t attach_state;

	attach_state = tcpm_inquire_typec_attach_state(ddata->tcpc_dev);
	if (attach_state == TYPEC_ATTACHED_AUDIO) {
		attach = !!noti->vbus_state.mv ? ATTACH_TYPE_PD_NONSTD :
			 ATTACH_TYPE_NONE;
		pr_notice("%s: audio_attach:%d\n", __func__, attach);
		handle_typec_pd_attach(ddata, attach);
	}
}

static int pd_tcp_notifier_call(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mt6366_chrdet_data *ddata = (struct mt6366_chrdet_data *)container_of(nb,
						    struct mt6366_chrdet_data, pd_nb);

	switch (event) {
	case TCP_NOTIFY_SINK_VBUS:
		handle_audio_attach(ddata, noti);
		break;
	case TCP_NOTIFY_PD_STATE:
		handle_pd_rdy_attach(ddata, noti);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
		    (noti->typec_state.new_state == TYPEC_ATTACHED_SNK ||
		    noti->typec_state.new_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		    noti->typec_state.new_state == TYPEC_ATTACHED_NORP_SRC)) {
			pr_info("%s USB Plug in, pol = %d\n", __func__,
					noti->typec_state.polarity);
			handle_typec_pd_attach(ddata, ATTACH_TYPE_TYPEC);
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SNK ||
		    noti->typec_state.old_state == TYPEC_ATTACHED_CUSTOM_SRC ||
		    noti->typec_state.old_state == TYPEC_ATTACHED_NORP_SRC ||
		    noti->typec_state.old_state == TYPEC_ATTACHED_AUDIO)
			&& noti->typec_state.new_state == TYPEC_UNATTACHED) {
			pr_info("%s USB Plug out\n", __func__);
			handle_typec_pd_attach(ddata, ATTACH_TYPE_NONE);
		} else if (noti->typec_state.old_state == TYPEC_ATTACHED_SRC &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			pr_info("%s Source_to_Sink, turn to PD Flow\n", __func__);
		}  else if (noti->typec_state.old_state == TYPEC_ATTACHED_SNK &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			pr_info("%s Sink_to_Source\n", __func__);
			handle_typec_pd_attach(ddata, ATTACH_TYPE_NONE);
		}
		break;
	case TCP_NOTIFY_EXT_DISCHARGE:
		if (noti->en_state.en)
			pr_info("%s turn on charger discharge\n", __func__);
		else
			pr_info("%s turn off charger discharge\n", __func__);
		break;
	default:
		break;
	};
	return NOTIFY_OK;
}

static int chg_type_det_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		pr_info("%s: enter PM_SUSPEND_PREPARE\n", __func__);
		break;
	case PM_POST_SUSPEND:
		pr_info("%s: enter PM_POST_SUSPEND\n", __func__);
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static void do_charger_detection_work(struct work_struct *work)
{
	struct mt6366_chrdet_data *ddata = container_of(work, struct mt6366_chrdet_data, chr_work);

	mutex_lock(&ddata->chrdet_lock);

	if (ddata->is_plugin) {
		dev_info(ddata->dev, "charger type: charger IN\n");
		ddata->g_chr_type = hw_charging_get_charger_type(ddata);
		chrdet_inform_psy_changed(ddata, ddata->g_chr_type, 1);
	} else {
		dev_info(ddata->dev, "charger type: charger OUT\n");
		ddata->g_chr_type = CHARGER_UNKNOWN;
		chrdet_inform_psy_changed(ddata, ddata->g_chr_type, 0);
	}
	power_supply_changed(ddata->chrdet_psy);

	mutex_unlock(&ddata->chrdet_lock);
}
#else
#include <linux/extcon.h>
static bool droi_usb_is_host(void)
{
	static struct extcon_dev *extcon_dev = NULL;
	static char *extcon_name = "soc:extcon_usb";

	if (IS_ERR_OR_NULL(extcon_dev)) {
		extcon_dev = extcon_get_extcon_dev(extcon_name);
		if (IS_ERR_OR_NULL(extcon_dev)) {
			pr_err("%s: Cannot find extcon_dev for %s\n", __func__, extcon_name);
			return false;
		}
	}

	return extcon_get_state(extcon_dev, EXTCON_USB_HOST) ? true : false;
}

static void do_charger_detect(struct mt6366_chrdet_data *ddata)
{
#if 1
	if (droi_usb_is_host()) {
		ddata->g_chr_type = CHARGER_UNKNOWN;
		dev_info(ddata->dev, "charger type: UNKNOWN, Now is usb host mode. Skip detection!!!\n");
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		if (ddata->first_connect == true) {
			ddata->first_connect = false;
		}
#endif
		return;
	}
#endif

	mutex_lock(&ddata->chrdet_lock);

	if (pmic_get_register_value_rgs_chrdet(ddata->rmap)) {
		dev_info(ddata->dev, "charger type: charger IN\n");
		ddata->g_chr_type = hw_charging_get_charger_type(ddata);
		chrdet_inform_psy_changed(ddata, ddata->g_chr_type, 1);
	} else {
		dev_info(ddata->dev, "charger type: charger OUT\n");
		ddata->g_chr_type = CHARGER_UNKNOWN;
		chrdet_inform_psy_changed(ddata, ddata->g_chr_type, 0);
	}
	power_supply_changed(ddata->chrdet_psy);

	mutex_unlock(&ddata->chrdet_lock);
}

/*****************************************************************************
 * PMIC Int Handler
 ******************************************************************************/
static irqreturn_t chrdet_int_handler(int irq, void *data)
{
	struct mt6366_chrdet_data *ddata = data;

	if (!pmic_get_register_value_rgs_chrdet(ddata->rmap)) {
		int boot_mode = get_boot_mode_from_dts();

		if (boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT) {
			dev_info(ddata->dev, "[%s] Unplug Charger/USB\n", __func__);
#ifndef CONFIG_TCPC_CLASS
			//orderly_poweroff(true);
#else
			return IRQ_HANDLED;
#endif
		}
	}
	do_charger_detect(ddata);

	return IRQ_HANDLED;
}

/************************************************
 * Charger Probe Related
 ************************************************/
static void do_charger_detection_work(struct work_struct *work)
{
	struct mt6366_chrdet_data *ddata = container_of(work, struct mt6366_chrdet_data, chr_work);

	if (pmic_get_register_value_rgs_chrdet(ddata->rmap))
		do_charger_detect(ddata);
}
#endif

static enum power_supply_usb_type mt6366_chrdet_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
};

static enum power_supply_property mt6366_chrdet_psy_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CALIBRATE,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
};

static int mt6366_chrdet_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
	case POWER_SUPPLY_PROP_TYPE:
	case POWER_SUPPLY_PROP_USB_TYPE:
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		return 1;
	default:
		return 0;
	}
	return 0;
}

static int mt6366_chrdet_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6366_chrdet_data *ddata = power_supply_get_drvdata(psy);
	int ret = 0;

	mt_dbg(ddata->dev, "get psp=%d\n", psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Mediatek";
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = ddata->chg_online;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (ddata->chg_online) {
#if defined(DROI_USING_CHARGER_CLASS_INTF)
			bool chg_done = false;
			charger_dev_is_charging_done(ddata->primary_charger, &chg_done);
			if (chg_done) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
			} else {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
			dev_info(ddata->dev, "[droi]%s: chg_done=%d, intval=%d\n", __func__, chg_done, val->intval);
#else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
#endif
		} else {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
		//return -EINVAL;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		ret = charger_dev_get_charging_current(ddata->primary_charger, &val->intval);
		break;
#endif
		return -EINVAL;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		ret = charger_dev_get_constant_voltage(ddata->primary_charger, &val->intval);
		break;
#endif
		return -EINVAL;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		ret = charger_dev_get_input_current(ddata->primary_charger, &val->intval);
		break;
#endif
		return -EINVAL;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		val->intval = 4400000;
		break;
#endif
		return -EINVAL;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		val->intval = 100000;
		break;
#endif
		return -EINVAL;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (ddata->psy_desc1.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 500000;
		else if (ddata->psy_desc1.type == POWER_SUPPLY_TYPE_USB_DCP)
			val->intval = 1500000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		//if (ddata->psy_desc1.type == POWER_SUPPLY_TYPE_USB)
			val->intval = 5000000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = ddata->chg_type;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = ddata->psy_usb_type;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = ddata->psy_desc1.type;
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
#if defined(DROI_USING_CHARGER_CLASS_INTF)
		break;
#endif
		return -EINVAL;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		val->intval = ddata->vbat0_flag;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mt6366_chrdet_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	int ret = 0;
	struct mt6366_chrdet_data *ddata = power_supply_get_drvdata(psy);

	dev_info(ddata->dev, "%s: set psp=%d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ddata->chg_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		//break;
		return -EINVAL;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		//break;
		return -EINVAL;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		//break;
		return -EINVAL;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		//break;
		return -EINVAL;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		//break;
		return -EINVAL;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		//break;
		return -EINVAL;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ddata->chg_type = val->intval;
		if (STANDARD_HOST == ddata->chg_type) {
			ddata->psy_desc1.type = POWER_SUPPLY_TYPE_USB;
			ddata->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		} else if (CHARGING_HOST == ddata->chg_type) {
			ddata->psy_desc1.type = POWER_SUPPLY_TYPE_USB_CDP;
			ddata->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		} else if (STANDARD_CHARGER == ddata->chg_type) {
			ddata->psy_desc1.type = POWER_SUPPLY_TYPE_USB_DCP;
			ddata->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		} else if (NONSTANDARD_CHARGER == ddata->chg_type) {
			ddata->psy_desc1.type = POWER_SUPPLY_TYPE_USB;
			ddata->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		} else {
			ddata->psy_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
			ddata->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ddata->psy_usb_type = val->intval;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		ddata->psy_desc1.type = val->intval;
		break;
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		ddata->vbat0_flag = val->intval;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static char *mt6366_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static const struct power_supply_desc mt6366_psy_desc1 = {
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = mt6366_chrdet_psy_usb_types,
	.num_usb_types = ARRAY_SIZE(mt6366_chrdet_psy_usb_types),
	.properties = mt6366_chrdet_psy_properties,
	.num_properties = ARRAY_SIZE(mt6366_chrdet_psy_properties),
	.property_is_writeable = mt6366_chrdet_property_is_writeable,
	.get_property = mt6366_chrdet_get_property,
	.set_property = mt6366_chrdet_set_property,
};

static int mt6366_chrdet_init_psy1(struct mt6366_chrdet_data *ddata)
{
	struct power_supply_config cfg1 = {
		.drv_data = ddata,
		.of_node = ddata->dev->of_node,
		.supplied_to = mt6366_psy_supplied_to,
		.num_supplicants = ARRAY_SIZE(mt6366_psy_supplied_to),
	};

	dev_info(ddata->dev, "%s\n", __func__);
	memcpy(&ddata->psy_desc1, &mt6366_psy_desc1, sizeof(ddata->psy_desc1));
	ddata->psy_desc1.name = dev_name(ddata->dev);
	ddata->chrdet_psy = devm_power_supply_register(ddata->dev, &ddata->psy_desc1, &cfg1);
	return IS_ERR(ddata->chrdet_psy) ? PTR_ERR(ddata->chrdet_psy) : 0;
}

static enum power_supply_property mt6366_chrtyp_psy_properties[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int mt6366_chrtyp_property_is_writeable(struct power_supply *psy,
					    enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return 0;
	default:
		return 0;
	}
	return 0;
}

#define R_CHARGER_1	330
#define R_CHARGER_2	39
static int mt6366_chrtyp_get_vbus_voltage(struct mt6366_chrdet_data *ddata, int *val)
{
	int ret;

	if (!IS_ERR(ddata->chan_vbus_voltage)) {
		ret = iio_read_channel_processed(ddata->chan_vbus_voltage, val);
		if (ret < 0)
			dev_notice(ddata->dev, "[%s]read fail,ret=%d\n", __func__, ret);
	} else {
		dev_notice(ddata->dev, "[%s]chan error %d\n", __func__, ddata->chan_vbus_voltage);
		ret = -ENOTSUPP;
	}

	*val = (((R_CHARGER_1 + R_CHARGER_2) * 100 * *val) / R_CHARGER_2) / 100;

	pr_notice("[%s] vbus: %d\n", __func__, *val);

	return ret;
}

static int mt6366_chrtyp_get_int_vbus_voltage(struct mt6366_chrdet_data *ddata)
{
	int val;

	mt6366_chrtyp_get_vbus_voltage(ddata, &val);
	return val;
}

static int mt6366_chrtyp_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6366_chrdet_data *ddata = power_supply_get_drvdata(psy);
	int ret = 0;

	mt_dbg(ddata->dev, "get psp=%d\n", psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = mt6366_chrtyp_get_int_vbus_voltage(ddata);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int mt6366_chrtyp_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	int ret = 0;
	struct mt6366_chrdet_data *ddata = power_supply_get_drvdata(psy);

	dev_info(ddata->dev, "%s: set psp=%d\n", __func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct power_supply_desc mt6366_psy_desc2 = {
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = mt6366_chrtyp_psy_properties,
	.num_properties = ARRAY_SIZE(mt6366_chrtyp_psy_properties),
	.property_is_writeable = mt6366_chrtyp_property_is_writeable,
	.get_property = mt6366_chrtyp_get_property,
	.set_property = mt6366_chrtyp_set_property,
};

static int mt6366_chrdet_init_psy2(struct mt6366_chrdet_data *ddata)
{
	struct power_supply_config cfg2 = {
		.drv_data = ddata,
	};

	dev_info(ddata->dev, "%s\n", __func__);
	memcpy(&ddata->psy_desc2, &mt6366_psy_desc2, sizeof(ddata->psy_desc2));
	ddata->psy_desc2.name = "mtk_charger_type";
	ddata->chrtyp_psy = devm_power_supply_register(ddata->dev, &ddata->psy_desc2, &cfg2);
	return IS_ERR(ddata->chrtyp_psy) ? PTR_ERR(ddata->chrtyp_psy) : 0;
}

static int mt6366_chrdet_probe(struct platform_device *pdev)
{
	//int chrdet_irq = -1;
	int ret = -1;
	struct mt6366_chrdet_data *ddata;
	struct device *dev = &pdev->dev;
	struct mt6397_chip *mt6397_chip = dev_get_drvdata(pdev->dev.parent);
	const struct of_device_id *of_id = of_match_device(mt6366_chrdet_of_match, &pdev->dev);
	struct iio_channel *chan_vbus_voltage;

	dev_info(dev, "%s\n", __func__);
	if (!of_id) {
		dev_err(dev, "Error: No device match found\n");
		return -ENODEV;
	}

	chan_vbus_voltage = devm_iio_channel_get(
		&pdev->dev, "pmic_vbus_voltage");
	if (IS_ERR(chan_vbus_voltage)) {
		ret = PTR_ERR(chan_vbus_voltage);
		dev_err(dev, "chan_vbus_voltage auxadc get fail, ret=%d\n", ret);
		return -EPROBE_DEFER;
	}

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata) {
		dev_err(dev, "failed to alloc ddata\n");
		return -ENOMEM;
	}

	ddata->chan_vbus_voltage = chan_vbus_voltage;
	ddata->rmap = mt6397_chip->regmap;
	ddata->dev = dev;
	ret = mt6366_chrdet_init_psy1(ddata);
	if (ret < 0) {
		dev_err(dev, "failed to init power supply1\n");
		goto err_init_psy;
	}

	ret = mt6366_chrdet_init_psy2(ddata);
	if (ret < 0) {
		dev_err(dev, "failed to init power supply2\n");
		goto err_init_psy;
	}

	mutex_init(&ddata->chrdet_lock);

#if defined(DROI_USING_CHARGER_CLASS_INTF)
	ddata->primary_charger = get_charger_by_name("primary_chg");
	if (!ddata->primary_charger) {
		dev_info(dev, "%s: get primary charger device failed\n", __func__);
		ret = -EINVAL;
		goto err_get_charger;
	}
	ddata->first_connect = true;
#endif

#if 1
	init_waitqueue_head(&ddata->attach_wq);
	atomic_set(&ddata->chrdet_start, 0);
	mutex_init(&ddata->attach_lock);
	platform_set_drvdata(pdev, ddata);

	ddata->tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	if (!ddata->tcpc_dev) {
		pr_notice("%s get tcpc device type_c_port0 fail\n", __func__);
		ret = -ENODEV;
		goto err_get_tcpc;
	}

	ddata->pm_nb.notifier_call = chg_type_det_pm_event;
	ddata->pd_nb.notifier_call = pd_tcp_notifier_call;

	ret = register_pm_notifier(&ddata->pm_nb);
	if (ret < 0)
		pr_notice("%s: register pm failed\n", __func__);

	ret = register_tcp_dev_notifier(ddata->tcpc_dev, &ddata->pd_nb,
					TCP_NOTIFY_TYPE_ALL);
	if (ret < 0) {
		pr_notice("%s: register tcpc notifer fail\n", __func__);
		ret = -EINVAL;
		goto err_get_tcpc;
	}

	ddata->attach_task = kthread_run(typec_attach_thread, ddata,
				       "attach_thread");
	if (IS_ERR(ddata->attach_task)) {
		pr_notice("%s: run typec attach kthread fail\n", __func__);
		ret = PTR_ERR(ddata->attach_task);
		goto err_get_tcpc;
	}

	INIT_WORK(&ddata->chr_work, do_charger_detection_work);
#else
	/* do charger detect here to prevent HW miss interrupt*/
	INIT_WORK(&ddata->chr_work, do_charger_detection_work);
	schedule_work(&ddata->chr_work);

	/* register pmic interrupt */
	chrdet_irq = platform_get_irq_byname(pdev, "chrdet");
	if (chrdet_irq < 0) {
		dev_info(dev, "Error: Get chrdet irq failed (%d)\n", chrdet_irq);
		ret = chrdet_irq;
		goto err_get_irq;
	}
	ret = devm_request_threaded_irq(&pdev->dev, chrdet_irq, NULL, chrdet_int_handler,
				IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "chrdet", ddata);
	if (ret) {
		dev_info(dev, "Error: Get thread irq request failed (%d) (%d)\n", chrdet_irq, ret);
		goto err_get_irq;
	}
#endif

	dev_info(dev, "%s: done!\n", __func__);
	return 0;

err_get_tcpc:
	mutex_destroy(&ddata->attach_lock);
#if defined(DROI_USING_CHARGER_CLASS_INTF)
err_get_charger:
#endif
	mutex_destroy(&ddata->chrdet_lock);
err_init_psy:
	devm_kfree(dev, ddata);
	return ret;
}

static int mt6366_chrdet_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "[droi]%s done!", __func__);
	return 0;
}

static struct platform_driver mt6366_chrdet_driver = {
	.probe = mt6366_chrdet_probe,
	.remove = mt6366_chrdet_remove,
	.driver = {
		.name = "mt6366-charger-type",
		.of_match_table = mt6366_chrdet_of_match,
	},
};

static int __init mt6366_chrdet_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&mt6366_chrdet_driver);
	if (ret)
		return -ENODEV;
	return 0;
}
static void __exit mt6366_chrdet_exit(void)
{
	platform_driver_unregister(&mt6366_chrdet_driver);
}
module_init(mt6366_chrdet_init);
module_exit(mt6366_chrdet_exit);

/* Module information */
MODULE_DESCRIPTION("MT6789 PMIC mt6366 dummy charger type detect driver");
MODULE_LICENSE("GPL v2");
