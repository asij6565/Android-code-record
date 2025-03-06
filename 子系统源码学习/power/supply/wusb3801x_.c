/************************************************************************
 *
 *  WILLSEMI TypeC Chipset Driver for Linux & Android.  
 *
 *
 * ######################################################################
 *
 *  Author: lei.huang (lhuang@sh-willsemi.com)
 *
 * Copyright (c) 2016, WillSemi Inc. All rights reserved.
 *
 ************************************************************************/

/************************************************************************
 *  Include files
 ************************************************************************/
#define __MEDIATEK_PLATFORM__
//#define __QCOM_PLATFORM__
//#define __TEST_CC_PATCH__
//#define __RESET_PATCH__
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/irq.h>
#include <linux/delay.h>
//#include <linux/wakelock.h>
#include <linux/workqueue.h>
//#include <mt-plat/v1/charger_type.h>


#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "../../misc/mediatek/usb_c/inc/typec.h"
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>

/*dingjun add  20171207*/
#include <linux/export.h>

#if defined(__MEDIATEK_PLATFORM__)
//#include <mt-plat/mt_gpio.h>
//#include <mt-plat/mt_gpio_core.h>
#if defined(MACH_TYPE_MT6735) || defined(MACH_TYPE_MT6735M)
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#else /*MACH_TYPE_MT6735 || MACH_TYPE_MT6735M*/
//#include <typec.h>
#endif /*MACH_TYPE_MT6735 || MACH_TYPE_MT6735M*/
#ifdef CONFIG_MTK_BQ24196_SUPPORT
#include "bq24196.h"
#endif /*CONFIG_MTK_BQ24196_SUPPORT*/
#ifdef CONFIG_MTK_BQ24297_SUPPORT
#include "bq24297.h"
#endif /*CONFIG_MTK_BQ24297_SUPPORT*/
#ifdef CONFIG_MTK_BQ24296_SUPPORT
//#include "bq24296.h"
#endif /*CONFIG_MTK_BQ24296_SUPPORT*/
#ifdef CONFIG_MTK_NCP1851_SUPPORT
#include "ncp1851.h"
#endif /*CONFIG_MTK_NCP1851_SUPPORT*/
#ifdef CONFIG_MTK_NCP1854_SUPPORT
#include "ncp1854.h"
#endif /*CONFIG_MTK_NCP1854_SUPPORT*/
#ifdef CONFIG_CHARGER_BQ25601
#include "../../../../power/supply/bq25601.h"
#endif
#include "class-dual-role.h"
#else /*__MEDIATEK_PLATFORM__*/
#include <linux/usb/class-dual-role.h>
#endif /*__MEDIATEK_PLATFORM__*/

/* Test for kernel version.
 * 3.18 => use DMA for I2C, no support for standardized
 * GPIO access in DTS.
 * 4.4 => the opposite.
 */
//#include <linux/version.h>

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0))
//# define KERNEL318MTK 1
//#endif

#ifdef dev_dbg
#undef dev_dbg
#define dev_dbg dev_info
#endif

/*TODO:  Define GPIO IRQ here or use DTS to describe interrupt*/
#if 1
#ifndef WUSB3801X_INT_PIN
#define WUSB3801X_INT_PIN 5   
//#error "No Interrupt GPIO specified."
#endif /*WUSB3801X_INT_PIN*/
#endif 


/***************************************************************************************************************************
* Example DTS for msm platform, User shall change the interrupt type, number, gpio ... according to their speific board design. 
****************************************************************************************************************************
        i2c@78b6000 { 
                wusb3801@c0 {
                        compatible = "qcom,wusb3801";
                        reg = <0xc0>;
                        qcom,irq-gpio = <&msm_gpio 21 0x8008>;   
                        interrupt-parent = <&msm_gpio>;
                        interrupts = <21 0>;
                        interrupt-names = "wusb3801_int_irq";
                        wusb3801,irq-gpio = <&msm_gpio 21 0x8008>;
                        wusb3801,reset-gpio = <&msm_gpio 12 0x0>;
                        wusb3801,init-mode = <0x24>;
                        wusb3801,host-current = <0x01>;
                        wusb3801,drp-toggle-time = <40>;
                };
        };
****************************************************************************************************************************
*/


/**
 * Options to enable force detection feature for DRP
 */
 
/*#define __WITH_POWER_BANK_MODE__*/  

/*
 *Bit operations if we don't want to include #include <linux/bitops.h> 
 */
 
#undef  __CONST_FFS
#define __CONST_FFS(_x) \
        ((_x) & 0x0F ? ((_x) & 0x03 ? ((_x) & 0x01 ? 0 : 1) :\
                                      ((_x) & 0x04 ? 2 : 3)) :\
                       ((_x) & 0x30 ? ((_x) & 0x10 ? 4 : 5) :\
                                      ((_x) & 0x40 ? 6 : 7)))

#undef  FFS
#define FFS(_x) \
        ((_x) ? __CONST_FFS(_x) : 0)

#undef  BITS
#define BITS(_end, _start) \
        ((BIT(_end) - BIT(_start)) + BIT(_end))

#undef  __BITS_GET
#define __BITS_GET(_byte, _mask, _shift) \
        (((_byte) & (_mask)) >> (_shift))

#undef  BITS_GET
#define BITS_GET(_byte, _bit) \
        __BITS_GET(_byte, _bit, FFS(_bit))

#undef  __BITS_SET
#define __BITS_SET(_byte, _mask, _shift, _val) \
        (((_byte) & ~(_mask)) | (((_val) << (_shift)) & (_mask)))

#undef  BITS_SET
#define BITS_SET(_byte, _bit, _val) \
        __BITS_SET(_byte, _bit, FFS(_bit), _val)

#undef  BITS_MATCH
#define BITS_MATCH(_byte, _bit) \
        (((_byte) & (_bit)) == (_bit))

/* Register Map */

#define WUSB3801_REG_VERSION_ID         0x01
#define WUSB3801_REG_CONTROL0           0x02
#define WUSB3801_REG_INTERRUPT          0x03
#define WUSB3801_REG_STATUS             0x04
#define WUSB3801_REG_CONTROL1           0x05
#define WUSB3801_REG_TEST0              0x06
#define WUSB3801_REG_TEST_01            0x07
#define WUSB3801_REG_TEST_02            0x08
#define WUSB3801_REG_TEST_03            0x09
#define WUSB3801_REG_TEST_04            0x0A
#define WUSB3801_REG_TEST_05            0x0B
#define WUSB3801_REG_TEST_06            0x0C
#define WUSB3801_REG_TEST_07            0x0D
#define WUSB3801_REG_TEST_08            0x0E
#define WUSB3801_REG_TEST_09            0x0F
#define WUSB3801_REG_TEST_0A            0x10
#define WUSB3801_REG_TEST_0B            0x11
#define WUSB3801_REG_TEST_0C            0x12
#define WUSB3801_REG_TEST_0D            0x13
#define WUSB3801_REG_TEST_0E            0x14
#define WUSB3801_REG_TEST_0F            0x15
#define WUSB3801_REG_TEST_10            0x16
#define WUSB3801_REG_TEST_11            0x17
#define WUSB3801_REG_TEST_12            0x18


#define WUSB3801_SLAVE_ADDR0            0xc0
#define WUSB3801_SLAVE_ADDR1            0xd0


/*Available modes*/
#define WUSB3801_DRP_ACC                (BIT_REG_CTRL0_RLE_DRP)
#define WUSB3801_DRP                    (BIT_REG_CTRL0_RLE_DRP | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SNK_ACC                (BIT_REG_CTRL0_RLE_SNK)
#define WUSB3801_SNK                    (BIT_REG_CTRL0_RLE_SNK | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_SRC_ACC                (BIT_REG_CTRL0_RLE_SRC) 
#define WUSB3801_SRC                    (BIT_REG_CTRL0_RLE_SRC | BIT_REG_CTRL0_DIS_ACC)
#define WUSB3801_DRP_PREFER_SRC_ACC     (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SRC)  
#define WUSB3801_DRP_PREFER_SRC         (WUSB3801_DRP     | BIT_REG_CTRL0_TRY_SRC)
#define WUSB3801_DRP_PREFER_SNK_ACC     (WUSB3801_DRP_ACC | BIT_REG_CTRL0_TRY_SNK)           
#define WUSB3801_DRP_PREFER_SNK         (WUSB3801_DRP     | BIT_REG_CTRL0_TRY_SNK)


/*TODO: redefine your prefer role here*/
#define WUSB3801_INIT_MODE              (WUSB3801_DRP_PREFER_SNK_ACC)
                                        
/*Registers relevant values*/                                      
#define WUSB3801_VENDOR_ID              0x06

/*Switch to enable/disable feature of specified Registers*/             
#define BIT_REG_CTRL0_DIS_ACC           (0x01 << 7)
#define BIT_REG_CTRL0_TRY_SRC           (0x02 << 5)
#define BIT_REG_CTRL0_TRY_SNK           (0x01 << 5)
#define BIT_REG_CTRL0_CUR_DEF           (0x00 << 3)
#define BIT_REG_CTRL0_CUR_1P5           (0x01 << 3)
#define BIT_REG_CTRL0_CUR_3P0           (0x02 << 3)
#define BIT_REG_CTRL0_RLE_SNK           (0x00 << 1)
#define BIT_REG_CTRL0_RLE_SRC           (0x01 << 1)
#define BIT_REG_CTRL0_RLE_DRP           (0x02 << 1)
#define BIT_REG_CTRL0_INT_MSK           (0x01 << 0)
           
                                        
#define BIT_REG_STATUS_VBUS             (0x01 << 7)
#define BIT_REG_STATUS_STANDBY          (0x00 << 5)
#define BIT_REG_STATUS_CUR_DEF          (0x01 << 5)
#define BIT_REG_STATUS_CUR_MID          (0x02 << 5)
#define BIT_REG_STATUS_CUR_HIGH         (0x03 << 5)
                                        
#define BIT_REG_STATUS_ATC_STB          (0x00 << 1)
#define BIT_REG_STATUS_ATC_SNK          (0x01 << 1)
#define BIT_REG_STATUS_ATC_SRC          (0x02 << 1)
#define BIT_REG_STATUS_ATC_ACC          (0x03 << 1)
#define BIT_REG_STATUS_ATC_DACC         (0x04 << 1)
                                        
#define BIT_REG_STATUS_PLR_STB          (0x00 << 0)
#define BIT_REG_STATUS_PLR_CC1          (0x01 << 0)
#define BIT_REG_STATUS_PLR_CC2          (0x02 << 0)
#define BIT_REG_STATUS_PLR_BOTH         (0x03 << 0)
                                        
#define BIT_REG_CTRL1_SW02_DIN          (0x01 << 4)
#define BIT_REG_CTRL1_SW02_EN           (0x01 << 3)
#define BIT_REG_CTRL1_SW01_DIN          (0x01 << 2)
#define BIT_REG_CTRL1_SW01_EN           (0x01 << 1)
#define BIT_REG_CTRL1_SM_RST            (0x01 << 0)



#define BIT_REG_TEST02_FORCE_ERR_RCY    (0x01)

#define WUSB3801_WAIT_VBUS               0x40
/*Fixed duty cycle period. 40ms:40ms*/
#define WUSB3801_TGL_40MS                0
#define WUSB3801_HOST_DEFAULT            0
#define WUSB3801_HOST_1500MA             1
#define WUSB3801_HOST_3000MA             2
#define WUSB3801_INT_ENABLE              0x00
#define WUSB3801_INT_DISABLE             0x01
#define WUSB3801_DISABLED                0x0A
#define WUSB3801_ERR_REC                 0x01
#define WUSB3801_VBUS_OK                 0x80

#define WUSB3801_SNK_0MA                (0x00 << 5)
#define WUSB3801_SNK_DEFAULT            (0x01 << 5)
#define WUSB3801_SNK_1500MA             (0x02 << 5)
#define WUSB3801_SNK_3000MA             (0x03 << 5)
#define WUSB3801_ATTACH                  0x1C

//#define WUSB3801_TYPE_PWR_ACC           (0x00 << 2) /*Ra/Rd treated as Open*/  
#define WUSB3801_TYPE_INVALID           (0x00)
#define WUSB3801_TYPE_SNK               (0x01 << 2)
#define WUSB3801_TYPE_SRC               (0x02 << 2)      
#define WUSB3801_TYPE_AUD_ACC           (0x03 << 2)
#define WUSB3801_TYPE_DBG_ACC           (0x04 << 2)

#define WUSB3801_INT_DETACH              (0x01 << 1)
#define WUSB3801_INT_ATTACH              (0x01 << 0)

#define WUSB3801_REV20                   0x02

/* Masks for Read-Modified-Write operations*/
#define WUSB3801_HOST_CUR_MASK           0x18  /*Host current for IIC*/
#define WUSB3801_INT_MASK                0x01
#define WUSB3801_BCLVL_MASK              0x60
#define WUSB3801_TYPE_MASK               0x1C
#define WUSB3801_MODE_MASK               0xE6  /*Roles relevant bits*/
#define WUSB3801_INT_STS_MASK            0x03
#define WUSB3801_FORCE_ERR_RCY_MASK      0x80  /*Force Error recovery*/
#define WUSB3801_ROLE_MASK               0x06
#define WUSB3801_VENDOR_ID_MASK          0x07
#define WUSB3801_VERSION_ID_MASK         0xF8
#define WUSB3801_POLARITY_CC_MASK        0x03


/* WUSB3801 STATES MACHINES */
#define WUSB3801_STATE_DISABLED             0x00
#define WUSB3801_STATE_ERROR_RECOVERY       0x01
#define WUSB3801_STATE_UNATTACHED_SNK       0x02
#define WUSB3801_STATE_UNATTACHED_SRC       0x03
#define WUSB3801_STATE_ATTACHWAIT_SNK       0x04
#define WUSB3801_STATE_ATTACHWAIT_SRC       0x05
#define WUSB3801_STATE_ATTACHED_SNK         0x06
#define WUSB3801_STATE_ATTACHED_SRC         0x07
#define WUSB3801_STATE_AUDIO_ACCESSORY      0x08
#define WUSB3801_STATE_DEBUG_ACCESSORY      0x09
#define WUSB3801_STATE_TRY_SNK              0x0A
#define WUSB3801_STATE_TRYWAIT_SRC          0x0B
#define WUSB3801_STATE_TRY_SRC              0x0C
#define WUSB3801_STATE_TRYWAIT_SNK          0x0D

/* wake lock timeout in ms */
#define WUSB3801_WAKE_LOCK_TIMEOUT          1000
/*1.5 Seconds timeout for force detection*/
#define ROLE_SWITCH_TIMEOUT		              1500

#if 0//defined(__MEDIATEK_PLATFORM__)

enum dual_role_supported_modes {
  DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP = 0,
  DUAL_ROLE_SUPPORTED_MODES_DFP,
  DUAL_ROLE_SUPPORTED_MODES_UFP,
  /*The following should be the last element*/
  DUAL_ROLE_PROP_SUPPORTED_MODES_TOTAL,
};

enum {
  DUAL_ROLE_PROP_MODE_UFP = 0,
  DUAL_ROLE_PROP_MODE_DFP,
  DUAL_ROLE_PROP_MODE_NONE,
  /*The following should be the last element*/
  DUAL_ROLE_PROP_MODE_TOTAL,
};

enum {
   DUAL_ROLE_PROP_PR_SRC = 0,
   DUAL_ROLE_PROP_PR_SNK,
   DUAL_ROLE_PROP_PR_NONE,
   /*The following should be the last element*/
   DUAL_ROLE_PROP_PR_TOTAL,
};

enum {
   DUAL_ROLE_PROP_DR_HOST = 0,
   DUAL_ROLE_PROP_DR_DEVICE,
   DUAL_ROLE_PROP_DR_NONE,
  /*The following should be the last element*/
   DUAL_ROLE_PROP_DR_TOTAL,
};

enum {
   DUAL_ROLE_PROP_VCONN_SUPPLY_NO = 0,
   DUAL_ROLE_PROP_VCONN_SUPPLY_YES,
   /*The following should be the last element*/
   DUAL_ROLE_PROP_VCONN_SUPPLY_TOTAL,
};

enum dual_role_property {
   DUAL_ROLE_PROP_SUPPORTED_MODES = 0,
   DUAL_ROLE_PROP_MODE,
   DUAL_ROLE_PROP_PR,
   DUAL_ROLE_PROP_DR,
   DUAL_ROLE_PROP_VCONN_SUPPLY,
};

struct dual_role_phy_instance {
	/* Driver private data */
	void *drv_data;
};
#if 1//wuxiwen modify for tyep c
extern void *dual_role_get_drvdata(struct dual_role_phy_instance *dual_role);
#else
void *dual_role_get_drvdata(struct dual_role_phy_instance *dual_role)
{
	return dual_role->drv_data;
}
#endif
static struct dual_role_phy_instance dual_role_service;

#endif /*__MEDIATEK_PLATFORM__*/

/*Private data*/
typedef struct wusb3801_data 
{
	uint32_t  int_gpio;
	uint8_t  init_mode;
	uint8_t  dfp_power;
	uint8_t  dttime;
}wusb3801_data_t;

/*Working context structure*/
typedef struct wusb3801_chip 
{
	struct      i2c_client *client;
	struct      wusb3801_data *pdata;
	struct      workqueue_struct  *cc_wq;
	int         irq_gpio;
	int         ufp_power;
#ifdef 	__TEST_CC_PATCH__
	uint8_t     reg4_val;
	uint8_t     cc_test_flag;
	uint8_t     cc_sts;
#endif	
	uint8_t     mode;
	uint8_t     dev_id;
	uint8_t     type;
	uint8_t     state;
	uint8_t     bc_lvl;
	uint8_t     dfp_power;
	uint8_t     dttime;
	uint8_t     attached;
	uint8_t     init_state;
	uint8_t		defer_init;	//work round the additional interrupt caused by mode change 
	int         try_attcnt;
	struct      work_struct dwork;
	//struct      wake_lock wlock;
	struct      mutex mlock;
	struct      power_supply *usb_psy;
	struct      dual_role_phy_instance *dual_role;
	struct      dual_role_phy_desc *desc;
	int    irq;
#ifdef 	__QCOM_PLATFORM__
	struct pinctrl *wusb3801_pinctrl;
    struct pinctrl_state *gpio_state_active;
    struct pinctrl_state *gpio_state_suspend;
#endif	
}wusb3801_chip_t;

#define wusb3801_update_state(chip, st)                      \
	if(chip) {                                                 \
		chip->state = st;                                        \
		dev_info(&chip->client->dev, "%s: %s\n", __func__, #st); \
		wake_up_interruptible(&mode_switch);                     \
	}

#define STR(s)    #s
#define STRV(s)   STR(s)

/* dingjun 2017.12.04 start */
static u32 wusb3801x_irq;

/* dingjun add 20171211 start */
static struct usbtypc g_typec;
static int  g_host_connected=0;
static int  g_device_connected=0;
/* dingjun add 20171211 end */

#ifdef CONFIG_MTK_BQ25890_SUPPORT
extern void bq25890_set_wd_timer(unsigned int val);
extern unsigned int bq25890_read_interface(unsigned char RegNum, unsigned char *val, unsigned char MASK,
				  unsigned char SHIFT);
#endif

#ifdef CONFIG_MTK_BQ24296_SUPPORT
extern void bq24296_set_watchdog(unsigned int val);
#endif


static void wusb3801_detach(struct wusb3801_chip *chip);
//static unsigned wusb3801_is_vbus_on(struct wusb3801_chip *chip);
DECLARE_WAIT_QUEUE_HEAD(mode_switch);

#if defined(CONFIG_RX_GPIO_USE)
extern int aeon_gpio_set(int *i);
//static int dpdm_en_high=8;
static int dpdm_en_low=9;
#endif
#if defined(CONFIG_DROI_MH07S_GPIO)
#include "../../droi/droi_mh07s_gpio.h"
#endif

#ifdef CONFIG_MTK_FLASHLIGHT_LM3643
extern int lm3643_pinctrl_set(int pin, int state); /* dingjun add 20171220*/
#endif

extern void accdet_workqueue_func(void);
/*dingjun add here 20171207*/
static u8 g_is_ep_insert = 0; /* 0: plug-out, 1: plug-in */
u8 get_earphone_state(void)
{
	return g_is_ep_insert;
}
EXPORT_SYMBOL(get_earphone_state);

#if 0
//extern void accdet_eint_int_handler(void);
extern void wusb3801x_accdet_workqueue_func(void);
extern void wusb3801x_disable_micbias(void);
extern void wusb3801x_accdet_eint_workqueue(void);
extern void wusb3801x_set_g_cur_eint_state(int setto);
#endif


#if 0//def CONFIG_AW2013_LED
extern void aw2013_breath_led0(void);
extern void aw2013_breath_led1(void);
extern void aw2013_breath_led2(void);
extern void aw2013_breath_all(int led0,int led1,int led2);
extern void aw2013_led_off(void);
#endif

/************************************************************************
 *
 *       wusb3801_write_masked_byte
 *
 *  Description :
 *  -------------
 *  Read-Modified-Writeback operation through I2C communication.
 *
 *  Parameter     :
 *  -----------
 *  Client         :    Pointer to I2C client. 
 *  Addr           :    Address of internal register of slave device
 *  Mask           :    Mask to prevent irrelevant bit changes
 *  val            :    Data wants to write.   
 *  Return values  :    Zero if no error, else error code.
 *  ---------------
 *  None
 *
 ************************************************************************/
 
static int wusb3801_write_masked_byte(struct i2c_client *client,
					uint8_t addr, uint8_t mask, uint8_t val)
{
	int rc;
	if (!mask){
		/* no actual access */
		rc = -EINVAL;
		goto out;
	}
	rc = i2c_smbus_read_byte_data(client, addr);
	if (!IS_ERR_VALUE((unsigned long)rc)){
		rc = i2c_smbus_write_byte_data(client,
			addr, BITS_SET((uint8_t)rc, mask, val));
	}
out:
	return rc;
}

static int wusb3801_read_device_id(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_VERSION_ID);
	printk("VendorID register: 0x%02x\n", rc );
	if (IS_ERR_VALUE((unsigned long)rc))
		return rc;
	dev_err(cdev, "VendorID register: 0x%02x\n", rc );
	printk("VendorID register: 0x%02x\n", rc );
	if((rc & WUSB3801_VENDOR_ID_MASK) != WUSB3801_VENDOR_ID){
		return -EINVAL;	
	}
	chip->dev_id = rc;
	dev_err(cdev, "Vendor id: 0x%02x, Version id: 0x%02x\n", rc & WUSB3801_VENDOR_ID_MASK,(rc & WUSB3801_VERSION_ID_MASK) >> 3);
	printk("Vendor id: 0x%02x, Version id: 0x%02x\n", rc & WUSB3801_VENDOR_ID_MASK,
	                                                         (rc & WUSB3801_VERSION_ID_MASK) >> 3);															 
	
	return 0;
}

static int wusb3801_update_status(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;

	/* Get control0 register */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}

	chip->mode      =  rc & WUSB3801_MODE_MASK;
	chip->dfp_power =  BITS_GET(rc, WUSB3801_HOST_CUR_MASK);
	chip->dttime    =  WUSB3801_TGL_40MS;

	return 0;
}

#if 0
static int wusb3801_check_modes(uint8_t mode)
{
	switch(mode){
		case WUSB3801_DRP_ACC            :
		case WUSB3801_DRP                :
		case WUSB3801_SNK_ACC            :
		case WUSB3801_SNK                :
		case WUSB3801_SRC_ACC            :
		case WUSB3801_SRC                :
		case WUSB3801_DRP_PREFER_SRC_ACC :
		case WUSB3801_DRP_PREFER_SRC     :
		case WUSB3801_DRP_PREFER_SNK_ACC :
		case WUSB3801_DRP_PREFER_SNK     :
			return 0;
			break;
		default:
			break;
	}
	return -EINVAL;
}
#endif

/*
static int wusb3801_reset_chip_to_error_recovery(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_CONTROL1, BIT_REG_CTRL1_SM_RST);

	if (IS_ERR_VALUE((unsigned long)rc)) 
	{
		dev_err(cdev, "failed to write manual(%d)\n", rc);
	}

	return rc;
}
*/

/*
 *  Spec lets transitioning to below states from any state
 *  WUSB3801_STATE_DISABLED
 *  WUSB3801_STATE_ERROR_RECOVERY
 */
static int wusb3801_set_chip_state(struct wusb3801_chip *chip, uint8_t state)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if(state > WUSB3801_STATE_UNATTACHED_SRC)
		return -EINVAL;
		

  rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL1, 
			   (state == WUSB3801_STATE_DISABLED) ? \
			             WUSB3801_DISABLED :        \
			             0);

	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "failed to write state machine(%d)\n", rc);
	}
	
	chip->init_state = state;

	return rc;
}


static int wusb3801_set_mode(struct wusb3801_chip *chip, uint8_t mode)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
	
	if (mode != chip->mode) {
		/*
		rc = wusb3801_write_masked_byte(chip->client,
				WUSB3801_REG_CONTROL0,
				WUSB3801_MODE_MASK,
				mode);
				*/
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: fail to read mode\n", __func__);
		return rc;
	}
	rc &= ~WUSB3801_MODE_MASK;
	//rc |= mode;
	rc |= (mode | WUSB3801_INT_MASK);//Disable the chip interrupt
  rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL0, rc);

	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "failed to write mode(%d)\n", rc);
		return rc;
	}

	//Clear the chip interrupt
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: fail to clear chip interrupt\n", __func__);
		return rc;
	}

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_CONTROL0);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: fail to read chip interrupt\n", __func__);
		return rc;
	}
	rc &= ~WUSB3801_INT_MASK;//enable the chip interrupt
	rc = i2c_smbus_write_byte_data(chip->client,
			   WUSB3801_REG_CONTROL0, rc);

	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "failed to enable chip interrupt(%d)\n", rc);
		return rc;
	}

	chip->mode = mode;
  }

 	dev_dbg(cdev, "%s: mode (0x%02x) (0x%02x)\n", __func__, chip->mode , mode);

	return rc;
}

static int wusb3801_set_dfp_power(struct wusb3801_chip *chip, uint8_t hcurrent)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if (hcurrent == chip->dfp_power) {
		dev_dbg(cdev, "vaule is not updated(%d)\n",
					hcurrent);
		return rc;
	}

	rc = wusb3801_write_masked_byte(chip->client,
					WUSB3801_REG_CONTROL0,
					WUSB3801_HOST_CUR_MASK,
					hcurrent);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "failed to write current(%d)\n", rc);
		return rc;
	}

	chip->dfp_power = hcurrent;

	dev_dbg(cdev, "%s: host current(%d)\n", __func__, hcurrent);

	return rc;
}

/**********************************************************************
 * When 3A capable DRP device is connected without VBUS,
 * DRP always detect it as SINK device erroneously.
 * Since USB Type-C specification 1.0 and 1.1 doesn't
 * consider this corner case, apply workaround for this case.
 * Set host mode current to 1.5A initially, and then change
 * it to default USB current right after detection SINK port.
 ***********************************************************************/
static int wusb3801_init_force_dfp_power(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	rc = wusb3801_write_masked_byte(chip->client,
					WUSB3801_REG_CONTROL0,
					WUSB3801_HOST_CUR_MASK,
					WUSB3801_HOST_3000MA);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "failed to write current\n");
		return rc;
	}

	chip->dfp_power = WUSB3801_HOST_3000MA;

	dev_dbg(cdev, "%s: host current (%d)\n", __func__, rc);

	return rc;
}

/************************************************************************
 *
 *       wusb3801_set_toggle_time
 *
 *  Description :
 *  -------------
 *  Wusb3801 varints only support for fixed duty cycles periods (40ms:40ms)
 *
 ************************************************************************/
static int wusb3801_set_toggle_time(struct wusb3801_chip *chip, uint8_t toggle_time)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	if (toggle_time != WUSB3801_TGL_40MS) {
		dev_err(cdev, "toggle_time(%d) is unavailable\n", toggle_time);
		return -EINVAL;
	}
	
	chip->dttime = WUSB3801_TGL_40MS;

	dev_dbg(cdev, "%s: Fixed toggle time (%d)\n", __func__, chip->dttime);

	return rc;
}


static int wusb3801_init_reg(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;

	/* change current */
	rc = wusb3801_init_force_dfp_power(chip);
	if (IS_ERR_VALUE((unsigned long)rc))
		dev_err(cdev, "%s: failed to force dfp power\n",
				__func__);


	/* change mode */
	rc = wusb3801_set_mode(chip, chip->pdata->init_mode);
	if (IS_ERR_VALUE((unsigned long)rc))
		dev_err(cdev, "%s: failed to set mode\n",
				__func__);

  rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);

	if (IS_ERR_VALUE((unsigned long)rc))
		dev_err(cdev, "%s: Reset state failed.\n",
				__func__);

	return rc;
}

static int wusb3801_reset_device(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc = 0;
/*
	rc = i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_CONTROL1, BIT_REG_CTRL1_SM_RST);
		
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "reset fails\n");
		return rc;
	}
	*/

	rc = wusb3801_update_status(chip);
	if (IS_ERR_VALUE((unsigned long)rc))
		dev_err(cdev, "fail to read status\n");

	rc = wusb3801_init_reg(chip);
	if (IS_ERR_VALUE((unsigned long)rc))
		dev_err(cdev, "fail to init reg\n");

	wusb3801_detach(chip);

	/* clear global interrupt mask */
	rc = wusb3801_write_masked_byte(chip->client,
				WUSB3801_REG_CONTROL0,
				WUSB3801_INT_MASK,
				WUSB3801_INT_ENABLE);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: fail to init\n", __func__);
		return rc;
	}

	dev_info(cdev, "mode[0x%02x], host_cur[0x%02x], dttime[0x%02x]\n",
			chip->mode, chip->dfp_power, chip->dttime);

	return rc;
}

/************************************************************************
 *
 *       fregdump_show
 *
 *  Description :
 *  -------------
 *  Dump registers to user space. there is side-effects for Read/Clear 
 *  registers. For example interrupt status. 
 *
 ************************************************************************/
static ssize_t fregdump_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int i, rc, ret = 0;

	mutex_lock(&chip->mlock);
	for (i = WUSB3801_REG_VERSION_ID ; i <= WUSB3801_REG_TEST_12; i++) 
	{
		rc = i2c_smbus_read_byte_data(chip->client, (uint8_t)i);
		if (IS_ERR_VALUE((unsigned long)rc)) 
		{
			pr_err("cannot read 0x%02x\n", i);
			rc = 0;
		}
		ret += snprintf(buf + ret, 1024 - ret, "from 0x%02x read 0x%02x\n", (uint8_t)i, rc);
	}
	mutex_unlock(&chip->mlock);

	return ret;
}

DEVICE_ATTR(fregdump, S_IRUGO, fregdump_show, NULL);

/************************************************************************
 *
 *       ftype_show
 *
 *  Description :
 *  -------------
 *  Dump types of attached devices
 *
 ************************************************************************/
static ssize_t ftype_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	switch (chip->type) {
	case WUSB3801_TYPE_SNK:
		ret = snprintf(buf, PAGE_SIZE, "SINK(%d)\n", chip->type);
		break;
	case WUSB3801_TYPE_SRC:
		ret = snprintf(buf, PAGE_SIZE, "SOURCE(%d)\n", chip->type);
		break;
	case WUSB3801_TYPE_DBG_ACC:
		ret = snprintf(buf, PAGE_SIZE, "DEBUGACC(%d)\n", chip->type);
		break;
	case WUSB3801_TYPE_AUD_ACC:
		ret = snprintf(buf, PAGE_SIZE, "AUDIOACC(%d)\n", chip->type);
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "NOTYPE(%d)\n", chip->type);
		break;
	}
	mutex_unlock(&chip->mlock);

	return ret;
}

DEVICE_ATTR(ftype, S_IRUGO , ftype_show, NULL);

/************************************************************************
 *
 *  fchip_state_show/fchip_state_store
 *
 *  Description :
 *  -------------
 *  Get/Set state to/from user spaces.
 *
 ************************************************************************/
static ssize_t fchip_state_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return snprintf(buf, PAGE_SIZE,
			STRV(WUSB3801_STATE_DISABLED) " - WUSB3801_STATE_DISABLED\n"
			STRV(WUSB3801_STATE_ERROR_RECOVERY) " - WUSB3801_STATE_ERROR_RECOVERY\n"
			STRV(WUSB3801_STATE_UNATTACHED_SNK) " - WUSB3801_STATE_UNATTACHED_SNK\n"
			STRV(WUSB3801_STATE_UNATTACHED_SRC) " - WUSB3801_STATE_UNATTACHED_SRC\n");

}


static ssize_t fchip_state_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int state = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &state) == 1) {
		mutex_lock(&chip->mlock);
	  if(((state == WUSB3801_STATE_UNATTACHED_SNK) &&
	    ((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SRC)) || \
	    ((state == WUSB3801_STATE_UNATTACHED_SRC) &&
	    ((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SNK))){
			mutex_unlock(&chip->mlock);
			return -EINVAL;
		}
		

		rc = wusb3801_set_chip_state(chip, (uint8_t)state);
		if (IS_ERR_VALUE((unsigned long)rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}
		
		wusb3801_detach(chip);
		mutex_unlock(&chip->mlock);
		return size;
	}

	return -EINVAL;
}

DEVICE_ATTR(fchip_state, S_IRUGO | S_IWUSR, fchip_state_show, fchip_state_store);

/************************************************************************
 *
 *  fmode_show/fmode_store
 *
 *  Description :
 *  -------------
 *  Dump/Set role to/from user spaces.
 ************************************************************************/
static ssize_t fmode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	switch (chip->mode) {
	case WUSB3801_DRP_ACC:
		ret = snprintf(buf, PAGE_SIZE, "DRP+ACC(%d)\n", chip->mode);
		break;
	case WUSB3801_DRP:
		ret = snprintf(buf, PAGE_SIZE, "DRP(%d)\n", chip->mode);
		break;
  case WUSB3801_SNK_ACC:
		ret = snprintf(buf, PAGE_SIZE, "SNK+ACC(%d)\n", chip->mode);
		break;
	case WUSB3801_SNK:
		ret = snprintf(buf, PAGE_SIZE, "SNK(%d)\n", chip->mode);
		break;
	case WUSB3801_SRC_ACC:
		ret = snprintf(buf, PAGE_SIZE, "SRC+ACC(%d)\n", chip->mode);
		break;
	case WUSB3801_SRC:
		ret = snprintf(buf, PAGE_SIZE, "SRC(%d)\n", chip->mode);
		break;
  case WUSB3801_DRP_PREFER_SRC_ACC:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+ACC+PREFER_SRC(%d)\n", chip->mode);
    break;
  case WUSB3801_DRP_PREFER_SRC:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+PREFER_SRC(%d)\n", chip->mode);
    break;
  case WUSB3801_DRP_PREFER_SNK_ACC:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+ACC+PREFER_SNK(%d)\n", chip->mode);
    break;
  case WUSB3801_DRP_PREFER_SNK:
  	ret = snprintf(buf, PAGE_SIZE, "DRP+PREFER_SNK(%d)\n", chip->mode);
    break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "UNKNOWN(%d)\n", chip->mode);
		break;
	}
	mutex_unlock(&chip->mlock);

	return ret;
}

static ssize_t fmode_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int mode = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &mode) == 1) {
		mutex_lock(&chip->mlock);

		/*
		 * since device trigger to usb happens independent
		 * from charger based on vbus, setting SRC modes
		 * doesn't prevent usb enumeration as device
		 * KNOWN LIMITATION
		 */
		rc = wusb3801_set_mode(chip, (uint8_t)mode);
		if (IS_ERR_VALUE((unsigned long)rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}

		rc = wusb3801_set_chip_state(chip,
					WUSB3801_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE((unsigned long)rc)) {
			mutex_unlock(&chip->mlock);
			return rc;
		}


		wusb3801_detach(chip);
		mutex_unlock(&chip->mlock);
		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(fmode, S_IRUGO | S_IWUSR, fmode_show, fmode_store);

/************************************************************************
 *
 *  fdttime_show/fdttime_store
 *
 *  Description :
 *  -------------
 *  Get/Set duty cycles period from/to user spaces. 
 *  Noted that: Wusb3801 uses fixed duty cycles percentage.
 ************************************************************************/
static ssize_t fdttime_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->dttime);
	mutex_unlock(&chip->mlock);
	return ret;
}

static ssize_t fdttime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int dttime = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &dttime) == 1) {
		mutex_lock(&chip->mlock);
		rc = wusb3801_set_toggle_time(chip, (uint8_t)dttime);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE((unsigned long)rc))
			return rc;

		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(fdttime, S_IRUGO | S_IWUSR, fdttime_show, fdttime_store);

/************************************************************************
 *
 *  fhostcur_show/fhostcur_store
 *
 *  Description :
 *  -------------
 *  Get/Set host current from/to user spaces. 
 ************************************************************************/
static ssize_t fhostcur_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%u\n", chip->dfp_power);
	mutex_unlock(&chip->mlock);
	return ret;
}

static ssize_t fhostcur_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int buf = 0;
	int rc = 0;

	if (sscanf(buff, "%d", &buf) == 1) {
		mutex_lock(&chip->mlock);
		rc = wusb3801_set_dfp_power(chip, (uint8_t)buf);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE((unsigned long)rc))
			return rc;

		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(fhostcur, S_IRUGO | S_IWUSR, fhostcur_show, fhostcur_store);

/************************************************************************
 *
 *  fclientcur_show
 *
 *  Description :
 *  -------------
 *  dumps client current to user spaces. 
 ************************************************************************/
static ssize_t fclientcur_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&chip->mlock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", chip->ufp_power);
	mutex_unlock(&chip->mlock);
	return ret;
}
DEVICE_ATTR(fclientcur, S_IRUGO, fclientcur_show, NULL);

/************************************************************************
 *
 *  freset_store
 *
 *  Description :
 *  -------------
 *  Reset state machine of WUSB3801
 ************************************************************************/
 
static ssize_t freset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	uint32_t reset = 0;
	int rc = 0;

	if (sscanf(buff, "%u", &reset) == 1) {
		mutex_lock(&chip->mlock);
		rc = wusb3801_reset_device(chip);
		mutex_unlock(&chip->mlock);
		if (IS_ERR_VALUE((unsigned long)rc))
			return rc;

		return size;
	}

	return -EINVAL;
}
DEVICE_ATTR(freset, S_IWUSR, NULL, freset_store);



static int wusb3801_create_devices(struct device *cdev)
{
	int ret = 0;

	ret = device_create_file(cdev, &dev_attr_fchip_state);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fchip_state\n");
		ret = -ENODEV;
		goto err0;
	}

	ret = device_create_file(cdev, &dev_attr_ftype);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_ftype\n");
		ret = -ENODEV;
		goto err1;
	}

	ret = device_create_file(cdev, &dev_attr_fmode);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fmode\n");
		ret = -ENODEV;
		goto err2;
	}

	ret = device_create_file(cdev, &dev_attr_freset);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_freset\n");
		ret = -ENODEV;
		goto err3;
	}

	ret = device_create_file(cdev, &dev_attr_fdttime);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fdttime\n");
		ret = -ENODEV;
		goto err4;
	}

	ret = device_create_file(cdev, &dev_attr_fhostcur);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fhostcur\n");
		ret = -ENODEV;
		goto err5;
	}

	ret = device_create_file(cdev, &dev_attr_fclientcur);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fufpcur\n");
		ret = -ENODEV;
		goto err6;
	}

	ret = device_create_file(cdev, &dev_attr_fregdump);
	if (ret < 0) {
		dev_err(cdev, "failed to create dev_attr_fregdump\n");
		ret = -ENODEV;
		goto err7;
	}

	return ret;

err7:
	device_remove_file(cdev, &dev_attr_fclientcur);
err6:
	device_remove_file(cdev, &dev_attr_fhostcur);
err5:
	device_remove_file(cdev, &dev_attr_fdttime);
err4:
	device_remove_file(cdev, &dev_attr_freset);
err3:
	device_remove_file(cdev, &dev_attr_fmode);
err2:
	device_remove_file(cdev, &dev_attr_ftype);
err1:
	device_remove_file(cdev, &dev_attr_fchip_state);
err0:
	return ret;
}


static void wusb3801_destory_device(struct device *cdev)
{
	device_remove_file(cdev, &dev_attr_fchip_state);
	device_remove_file(cdev, &dev_attr_ftype);
	device_remove_file(cdev, &dev_attr_fmode);
	device_remove_file(cdev, &dev_attr_freset);
	device_remove_file(cdev, &dev_attr_fdttime);
	device_remove_file(cdev, &dev_attr_fhostcur);
	device_remove_file(cdev, &dev_attr_fclientcur);
	device_remove_file(cdev, &dev_attr_fregdump);
}

//POWER_SUPPLY_PROP_CURRENT_MAX

static int wusb3801_power_set_icurrent_max(struct wusb3801_chip *chip,
						int icurrent)
{
/*
* TODO: Set your PMIC function
*/
#if !defined(__MEDIATEK_PLATFORM__)
	const union power_supply_propval ret = {icurrent,};

	chip->ufp_power = icurrent;

#if defined(__WITH_KERNEL_VER4__)
	if (chip->usb_psy->desc->set_property)
		return chip->usb_psy->desc->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &ret);
#else /*__WITH_KERNEL_VER4__*/
  	if (chip->usb_psy->set_property)
		return chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, &ret);
#endif /*__WITH_KERNEL_VER4__*/

	return -ENXIO;
#endif /*__MEDIATEK_PLATFORM__*/
  return 0;
}

static void wusb3801_bclvl_changed(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc, limit;
	uint8_t status, type;

	rc = i2c_smbus_read_byte_data(chip->client,
				WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read\n", __func__);
		if(IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}

	status = (rc & WUSB3801_TYPE_MASK) ? 1 : 0;
	type = status  ? rc & WUSB3801_TYPE_MASK : WUSB3801_TYPE_INVALID;

	dev_dbg(cdev, "%s: sts[0x%02x], type[0x%02x]\n",__func__, status, type);

	if (type == WUSB3801_TYPE_SRC) {

		
		chip->bc_lvl = rc & WUSB3801_BCLVL_MASK;
		limit = (chip->bc_lvl == WUSB3801_SNK_3000MA ? 3000 :
				(chip->bc_lvl == WUSB3801_SNK_1500MA ? 1500 : 0));

		wusb3801_power_set_icurrent_max(chip, limit);
		
	    dev_dbg(cdev, "%s: limit = %d\n",__func__, limit);
	}
	
}

#if defined(__MEDIATEK_PLATFORM__)
static void mediatek_set_charger(int mode)
{
    if(mode&0xFF)
    {
#ifdef CONFIG_MTK_BQ24196_SUPPORT
        bq24196_set_chg_config(0x3); //OTG
        bq24196_set_boost_lim(0x1); //1.3A on VBUS
        bq24196_set_en_hiz(0x0);
#endif

#ifdef CONFIG_MTK_BQ24297_SUPPORT
        bq24297_set_otg_config(0x1); //OTG
        bq24297_set_boost_lim(0x1); //1.5A on VBUS
        bq24297_set_en_hiz(0x0);
#endif

#ifdef CONFIG_MTK_BQ24296_SUPPORT
        //bq24296_set_chg_config(0x0); //disable charge
        //bq24296_set_otg_config(0x1); //OTG
        //bq24296_set_boostv(0x7); //boost voltage 4.998V
        //bq24296_set_boost_lim(0x1); //1.5A on VBUS
        //bq24296_set_en_hiz(0x0);
#endif

#ifdef CONFIG_MTK_NCP1851_SUPPORT
        ncp1851_set_chg_en(0x0); //charger disable
        ncp1851_set_otg_en(0x1); //otg enable
#endif

#ifdef MTK_NCP1854_SUPPORT
        ncp1854_set_chg_en(0x0); //charger disable
        ncp1854_set_otg_en(0x1); //otg enable
#endif

#if defined(DROI_PRO_MH11P)
#else
#ifdef CONFIG_CHARGER_BQ25601
		bq25601_set_chg_config(0x0);
		bq25601_set_otg_config(0x1);
		bq25601_set_watchdog(0x0);
#endif
#endif
    }
    else
    {
#ifdef CONFIG_MTK_BQ24196_SUPPORT
        bq24196_set_chg_config(0x0); //OTG & Charge disabled
#endif

#ifdef CONFIG_MTK_BQ24297_SUPPORT
        bq24297_set_otg_config(0x0); //OTG & Charge disabled
#endif

#ifdef CONFIG_MTK_BQ24296_SUPPORT
        //bq24296_set_otg_config(0x0); //OTG disabled
       // bq24296_set_chg_config(0x0); //Charge disabled
#endif

#ifdef CONFIG_MTK_NCP1851_SUPPORT
        ncp1851_set_otg_en(0x0);
#endif

#ifdef MTK_NCP1854_SUPPORT
        ncp1854_set_otg_en(0x0);
#endif

#if defined(DROI_PRO_MH11P)
#else
#ifdef CONFIG_CHARGER_BQ25601
		bq25601_set_otg_config(0x0);
#endif
#endif
    }
}
#endif /*__MEDIATEK_PLATFORM__*/

#if 0
static void wusb3801_usb20_host_detected(struct wusb3801_chip *chip)
{
	//struct device *cdev = &chip->client->dev;
	/*
	* TODO: Add your code here if you want do something during 
	*       USB2.0 host has been detected.
	*/
}
#endif

extern int get_uisoc_for_wusb3801(void);
#ifdef CONFIG_AW2013_LED
extern void aw2013_red_led_on(void);
extern void aw2013_green_led_on(void);
extern void aw2013_led_off(void);
#endif

static void wusb3801_src_detected(struct wusb3801_chip *chip)
{

	struct device *cdev = &chip->client->dev;

	dev_info(cdev, "%s \n",__func__);

#ifdef CONFIG_MTK_FLASHLIGHT_LM3643
   	lm3643_pinctrl_set(0x20/*DPDMSW_EN_PIN*/, 0);
#endif

#if defined(CONFIG_RX_GPIO_USE)
   	aeon_gpio_set(&dpdm_en_low);
#endif

   if((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SRC){
		dev_err(cdev, "not support in source mode\n");
		if(IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}

#if 0
       int s_ui_soc = 0;
       /*Rixiang-dingjun-add start*/
       s_ui_soc = get_uisoc_for_wusb3801();

       printk("%s,%d-20180327-uisoc=%d\n",__func__,__LINE__,s_ui_soc);

	if(s_ui_soc>=95)
	{/*GREEN*/
	    aw2013_green_led_on();
	}
	else
	{/*RED*/
	    aw2013_red_led_on();
	}
       /*Rixiang-dingjun-add end*/
#endif
	
	wusb3801_update_state(chip, WUSB3801_STATE_ATTACHED_SNK);
	
	//if(wusb3801_is_vbus_on(chip))
	//{
	//	dev_err(cdev, "USB2.0 detected %s.\n", __func__);
	//	wusb3801_usb20_host_detected(chip);
	//}
	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC source has detected.
	*/
	
#if defined(__MEDIATEK_PLATFORM__)
  mediatek_set_charger(0);
#if 0 //defined(DROI_PRO_PH01P)
	mt_usb_disconnect_v1();
#endif
	dual_role_instance_changed(chip->dual_role);
#else /*__MEDIATEK_PLATFORM__*/
	dual_role_instance_changed(chip->dual_role);
#endif /*__MEDIATEK_PLATFORM__*/
	chip->type = WUSB3801_TYPE_SRC;
}

static void wusb3801_snk_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	//unsigned char get_val; /*for test*/
	
	dev_info(cdev, "%s \n",__func__);

#ifdef CONFIG_MTK_FLASHLIGHT_LM3643
	lm3643_pinctrl_set(0x20/*DPDMSW_EN_PIN*/, 0);
#endif

#if defined(CONFIG_RX_GPIO_USE)
	aeon_gpio_set(&dpdm_en_low);
#endif
	if((chip->mode & WUSB3801_ROLE_MASK) == BIT_REG_CTRL0_RLE_SNK){
		dev_err(cdev, "not support in sink mode\n");
		if(IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}
       printk("%s,%d\n",__func__,__LINE__);
	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Sink has detected.
	*/
#if defined(CONFIG_USB_MTK_OTG)

#ifdef CONFIG_MTK_FLASHLIGHT_LM3643
   	lm3643_pinctrl_set(0x20/*DPDMSW_EN_PIN*/, 0);
#endif
#if defined(CONFIG_RX_GPIO_USE)
   	aeon_gpio_set(&dpdm_en_low);
#endif

#ifdef CONFIG_MTK_BQ25890_SUPPORT
		bq25890_set_wd_timer(0);
#endif
#ifdef CONFIG_MTK_BQ24296_SUPPORT		  
		bq24296_set_watchdog(0);
#endif
	  
		//bq25890_read_interface(0x07, &get_val, 0x03, 0x04); /*for test*/
	    //printk("%s,%d--wd_timer value=0x%x\n", __func__, __LINE__, get_val);  
	    //g_typec.host_driver->on = 1;
	    //g_host_connected=1;
	    //g_typec.host_driver->vbus_enable(g_typec.host_driver->priv_data);
	    //g_typec.host_driver->enable(g_typec.host_driver->priv_data);	
#endif /* Rixiang-dingjun-add 201801015*/		
		
#if  0//def CONFIG_AW2013_LED
       //aw2013_breath_all(1, 1, 1);
       	aw2013_breath_led2();
#endif
		/*
		 * mode == WUSB3801_SRC/WUSB3801_SRC_ACC
		 */
#if !defined(__MEDIATEK_PLATFORM__)
		power_supply_set_usb_otg(chip->usb_psy, true);
#endif /*__MEDIATEK_PLATFORM__*/
		wusb3801_set_dfp_power(chip, chip->pdata->dfp_power);
		wusb3801_update_state(chip, WUSB3801_STATE_ATTACHED_SRC);
#if defined(__MEDIATEK_PLATFORM__)
    mediatek_set_charger(1);
#if 0 //defined(DROI_PRO_PH01P)
	mt_usb_host_connect_v1();
#endif
		dual_role_instance_changed(chip->dual_role);
#else /*__MEDIATEK_PLATFORM__*/
		dual_role_instance_changed(chip->dual_role);
#endif /*__MEDIATEK_PLATFORM__*/
		chip->type = WUSB3801_TYPE_SNK;
}

static void wusb3801_dbg_acc_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	dev_info(cdev, "%s \n",__func__);
	if (chip->mode & BIT_REG_CTRL0_DIS_ACC) {
		dev_err(cdev, "not support accessory mode\n");
		if(IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}

	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Debug accessory has detected.
	*/
	wusb3801_update_state(chip, WUSB3801_STATE_DEBUG_ACCESSORY);
}

static void wusb3801_aud_acc_detected(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	dev_info(cdev, "%s \n",__func__);
	if (chip->mode & BIT_REG_CTRL0_DIS_ACC) {
		dev_err(cdev, "not support accessory mode\n");
		if(IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		return;
	}

	/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Audio accessory has detected.
	*/
	/*dingjun 2017.12.07*/
	//wusb3801x_set_g_cur_eint_state(0x01);
	g_is_ep_insert=1;
	printk("--dingjun--%s,%d \n",__func__,__LINE__);
	wusb3801_update_state(chip, WUSB3801_STATE_AUDIO_ACCESSORY);
}

static void wusb3801_aud_acc_detach(struct wusb3801_chip *chip)
{
		//struct device *cdev = &chip->client->dev;
/**
	*  TODO: Add you code here if you want doing something special during
	*        TYPEC Audio accessory pull out.
	*/
	/*dingjun 2017.12.07*/
	//wusb3801x_set_g_cur_eint_state(0x00);
	g_is_ep_insert=0;
	printk("--dingjun--%s,%d \n",__func__,__LINE__);
}
#ifdef __RESET_PATCH__
static int reset_patch(struct wusb3801_chip *chip)
{
	int rc;
	struct device *cdev = &chip->client->dev;	
	//reg10 write 0x55
	rc = i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_04, 0x55);		
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "write reg3 0x1 failed 0x%02x\n", rc);
		return rc;
	}
	msleep(25);
	//reg10 write 0x00
	rc = i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_04, 0x00);		
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "write reg3 0x1 failed 0x%02x\n", rc);
		return rc;
	}	
	return 0;
}
#endif
static void wusb3801_detach(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
#ifdef __RESET_PATCH__
	int ret;
#endif	
	dev_err(cdev, "%s: type[0x%02x] chipstate[0x%02x]\n",
			__func__, chip->type, chip->state);
	
	switch (chip->state) {
	case WUSB3801_STATE_ATTACHED_SRC:
#if !defined(__MEDIATEK_PLATFORM__)
		power_supply_set_usb_otg(chip->usb_psy, false);
#endif /*__MEDIATEK_PLATFORM__*/
		wusb3801_init_force_dfp_power(chip);
#if defined(CONFIG_USB_MTK_OTG)  
	    //g_typec.host_driver->on = 0;
	    //g_host_connected=0;
	    //g_typec.host_driver->disable(g_typec.host_driver->priv_data);
	    //g_typec.host_driver->vbus_disable(g_typec.host_driver->priv_data);
#ifdef CONFIG_MTK_BQ25890_SUPPORT
		bq25890_set_wd_timer(1);
#endif

#ifdef CONFIG_MTK_BQ24296_SUPPORT
		bq24296_set_watchdog(1);
#endif
#endif
		break;

	case WUSB3801_STATE_ATTACHED_SNK:
		wusb3801_power_set_icurrent_max(chip, 0);

#ifdef CONFIG_AW2013_LED
        //aw2013_led_off();
#endif
		break;
	case WUSB3801_STATE_DEBUG_ACCESSORY:
	  break;
	case WUSB3801_STATE_AUDIO_ACCESSORY:
	  wusb3801_aud_acc_detach(chip);
		break;
	case WUSB3801_STATE_DISABLED:
	case WUSB3801_STATE_ERROR_RECOVERY:
		break;
	default:
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		break;
	}
	chip->type = WUSB3801_TYPE_INVALID;
	chip->bc_lvl = WUSB3801_SNK_3000MA;
	chip->ufp_power  = 0;
	chip->try_attcnt = 0;
	chip->attached   = 0;
#ifdef __RESET_PATCH__
	ret = reset_patch(chip);
	if(ret != 0)
		dev_err(cdev, "%s: reset patch failed\n", __func__);
#endif		
	//Excute the defered init mode into drp+try.snk when detached
	if(chip->defer_init == 1) {
		dev_err(cdev, "%s: reset to init mode\n", __func__);
		if (IS_ERR_VALUE((unsigned long)wusb3801_set_mode(chip, chip->pdata->init_mode)))
			dev_err(cdev, "%s: failed to set init mode\n", __func__);
		chip->defer_init = 0;
	}

	wusb3801_update_state(chip, WUSB3801_STATE_ERROR_RECOVERY);
#if 1//!defined(__MEDIATEK_PLATFORM__)
	dual_role_instance_changed(chip->dual_role);
#endif /*__MEDIATEK_PLATFORM__*/

#ifdef CONFIG_MTK_FLASHLIGHT_LM3643
	lm3643_pinctrl_set(0x20/*DPDMSW_EN_PIN*/, 1);
#endif
#if defined(CONFIG_RX_GPIO_USE)
	aeon_gpio_set(&dpdm_en_low);
#endif
#if defined(CONFIG_DROI_MH07S_GPIO)
	aeon_gpio_set(OTG_EN_SWTICH_LOW);
#endif
}


#if 0
static unsigned wusb3801_is_vbus_off(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;

	rc = i2c_smbus_read_byte_data(chip->client,
				WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return false;
	}

	return !((rc & WUSB3801_TYPE_MASK) && (rc & WUSB3801_VBUS_OK));
}
#endif /*if 0*/
#if 0
static unsigned wusb3801_is_vbus_on(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return false;
	}

	return !!(rc & WUSB3801_VBUS_OK);
}
#endif
#ifdef __RESET_PATCH__
static int test_cc_patch(struct wusb3801_chip *chip)
{		
	int rc;
	struct device *cdev = &chip->client->dev;
	dev_err(cdev, "%s \n",__func__);
	//mutex_lock(&chip->mlock);

	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_02, 0x82);	
	msleep(100);
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_09, 0xC0);	
	msleep(100);
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST0);
	msleep(10);
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_09, 0x00);	
	msleep(10);
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_02, 0x80);
	msleep(10);	
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_TEST_02, 0x00);	
	//mutex_unlock(&chip->mlock);
	dev_err(cdev, "%s rc = [0x%02x] \n",__func__, rc);
    return BITS_GET(rc, 0x40);
}
#endif
#if 1//!defined(__MEDIATEK_PLATFORM__)
static unsigned wusb3801_is_waiting_for_vbus(struct wusb3801_chip *chip)
{
  struct device *cdev = &chip->client->dev;
	int rc;
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_TEST_0A);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read test0a\n", __func__);
		return false;
	}
	return !!(rc & WUSB3801_WAIT_VBUS);
}
#endif /*__MEDIATEK_PLATFORM__*/

//#include "mtk_musb.h"

static void wusb3801_attach(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	int rc;
	uint8_t status, type;

	/* get status */
	rc = i2c_smbus_read_byte_data(chip->client,
			WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read status\n", __func__);
		return;
	}
#ifdef __TEST_CC_PATCH__
	if(chip->cc_test_flag == 0 &&  BITS_GET(rc, 0x80) ){
		chip->reg4_val = rc;
		chip->cc_sts = test_cc_patch(chip);	
		chip->cc_test_flag = 1;
		dev_err(cdev, "%s: cc_sts[0x%02x]\n", __func__, chip->cc_sts);	
		return;
	}
	if(chip->cc_test_flag == 1){
		chip->cc_test_flag = 0;
		if(chip->cc_sts)
			rc = rc | 0x10;
		else
			rc = rc | 0x01;
		dev_err(cdev, "%s: cc_test_patch rc[0x%02x]\n", __func__, rc);			
	}
#endif	
	status = (rc & WUSB3801_ATTACH) ? true : false;
	type = status ? \
			rc & WUSB3801_TYPE_MASK : WUSB3801_TYPE_INVALID;
	dev_info(cdev, "sts[0x%02x], type[0x%02x]\n", status, type);

	if (chip->state != WUSB3801_STATE_ERROR_RECOVERY) {
		rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE((unsigned long)rc))
			dev_err(cdev, "%s: failed to set error recovery\n",
					__func__);
		wusb3801_detach(chip);
		dev_err(cdev, "%s: Invaild chipstate[0x%02x]\n",
				__func__, chip->state);
		return;
	}
	

	switch (type) {
	case WUSB3801_TYPE_SRC:
		wusb3801_src_detected(chip);
		//mt_usb_connect();
		break;
	case WUSB3801_TYPE_SNK:
		wusb3801_snk_detected(chip);
		break;
	case WUSB3801_TYPE_DBG_ACC:
		wusb3801_dbg_acc_detected(chip);
		chip->type = type;
		break;
	case WUSB3801_TYPE_AUD_ACC:
		wusb3801_aud_acc_detected(chip);
		chip->type = type;
		break;
	case WUSB3801_TYPE_INVALID:
		wusb3801_detach(chip);
		
		dev_err(cdev, "%s: Invaild type[0x%02x]\n", __func__, type);
		break;
	default:
		rc = wusb3801_set_chip_state(chip,
				WUSB3801_STATE_ERROR_RECOVERY);
		if (IS_ERR_VALUE((unsigned long)rc))
			dev_err(cdev, "%s: failed to set error recovery\n",
					__func__);

		wusb3801_detach(chip);
		dev_err(cdev, "%s: Unknwon type[0x%02x]\n", __func__, type);
		break;
	}
  chip->attached = true;
#if defined(CONFIG_DROI_MH07S_GPIO)
	aeon_gpio_set(OTG_EN_SWTICH_HIGH);
#endif
}
static void wusb3801_work_handler(struct work_struct *work)
{
	struct wusb3801_chip *chip =
			container_of(work, struct wusb3801_chip, dwork);
	struct device *cdev = &chip->client->dev;
	int rc;
	uint8_t int_sts;
	dev_info(cdev,"%s \n",__func__);
	mutex_lock(&chip->mlock);
	/* get interrupt */
	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	mutex_unlock(&chip->mlock);	
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read interrupt\n", __func__);
		goto work_unlock;
	}
	int_sts = rc & WUSB3801_INT_STS_MASK;

	rc = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		dev_err(cdev, "%s: failed to read reg status\n", __func__);
		goto work_unlock;
	}
	dev_info(cdev,"%s WUSB3801_REG_STATUS : 0x%02x\n", __func__, rc);
	
	dev_info(cdev, "%s: int_sts[0x%02x]\n", __func__, int_sts);
	if (int_sts & WUSB3801_INT_DETACH){
		wusb3801_detach(chip);
	}
	if (int_sts & WUSB3801_INT_ATTACH){
		if(chip->attached){
			wusb3801_bclvl_changed(chip);
		}
		else{
			wusb3801_attach(chip);
		}
	}
work_unlock:
	enable_irq(wusb3801x_irq);
}

static irqreturn_t wusb3801_interrupt(int irq, void *data)
{
	struct wusb3801_chip *chip = (struct wusb3801_chip *)data;
	if (!chip) {
		dev_err(&chip->client->dev, "%s : called before init.\n", __func__);
		return IRQ_HANDLED;
	}
	disable_irq_nosync(irq);
	
	/*
	 * wake_lock_timeout, prevents multiple suspend entries
	 * before charger gets chance to trigger usb core for device
	 */
	/*wake_lock_timeout(&chip->wlock,
				msecs_to_jiffies(WUSB3801_WAKE_LOCK_TIMEOUT));*/
	if (!queue_work(chip->cc_wq, &chip->dwork))
	{
		dev_err(&chip->client->dev, "%s: can't alloc work\n", __func__);
		enable_irq(irq);
	}

	return IRQ_HANDLED;
}


#if 0
static int wusb3801_init_gpio(struct wusb3801_chip *chip)
{

	int rc = 0;
#if defined(__MEDIATEK_PLATFORM__)
	mt_set_gpio_mode_base(WUSB3801X_INT_PIN, GPIO_MODE_00);  // gpio mode
	mt_set_gpio_dir_base(WUSB3801X_INT_PIN, GPIO_DIR_IN); // input
	mt_set_gpio_pull_enable_base(WUSB3801X_INT_PIN,GPIO_PULL_ENABLE);//enable pull 
	mt_set_gpio_pull_select_base(WUSB3801X_INT_PIN,GPIO_PULL_UP);//pull up

//#else  /*__MEDIATEK_PLATFORM__*/
	struct device *cdev = &chip->client->dev;
	/* Start to enable wusb3801 Chip */
	if (gpio_is_valid(chip->pdata->int_gpio)) {
		rc = gpio_request_one(chip->pdata->int_gpio,
				GPIOF_DIR_IN, "wusb3801_int_gpio");
		if (rc)
			dev_err(cdev, "unable to request int_gpio %d\n",
					chip->pdata->int_gpio);
	} else {
		dev_err(cdev, "int_gpio %d is not valid\n",
				chip->pdata->int_gpio);
		rc = -EINVAL;
	}
#endif /*__MEDIATEK_PLATFORM__*/
	return rc;
}
#endif

static void wusb3801_free_gpio(struct wusb3801_chip *chip)
{
	if (gpio_is_valid(chip->pdata->int_gpio))
		gpio_free(chip->pdata->int_gpio);
}

#if 0
static int wusb3801_parse_dt(struct wusb3801_chip *chip)
{
	struct device *cdev = &chip->client->dev;
	struct device_node *dev_node = cdev->of_node;
	struct wusb3801_data *data = chip->pdata;
	u32 val = 0;
	int rc = 0;
	dev_err(cdev, "dev_node->name=%s.\n",dev_node->name);
	data->int_gpio = of_get_named_gpio_flags(dev_node,
				"wusb3801,irq-gpio", 0,NULL);
	dev_err(cdev, "data->int_gpio=%d",data->int_gpio);
	if (data->int_gpio < 0) {
		dev_err(cdev, "int_gpio is not available\n");
		rc = data->int_gpio;
        //data->int_gpio = WUSB3801X_INT_PIN;
		goto out;
	}

	rc = of_property_read_u32(dev_node,
				"wusb3801,init-mode", &val);
	data->init_mode = (u8)val;
	dev_err(cdev, "data->init_mode=%d.\n",data->init_mode);
	if (rc || wusb3801_check_modes(data->init_mode)) {
		dev_err(cdev, "init mode is not available and set default\n");
		data->init_mode = WUSB3801_INIT_MODE; //WUSB3801_DRP_ACC;
		rc = 0;
	}

	rc = of_property_read_u32(dev_node,
				"wusb3801,host-current", &val);
	data->dfp_power = (u8)val;
	dev_err(cdev, "data->dfp_power=%d.\n",data->dfp_power);
	if (rc || (data->dfp_power > WUSB3801_HOST_3000MA)) {
		dev_err(cdev, "host current is not available and set default\n");
		data->dfp_power = WUSB3801_HOST_DEFAULT;
		rc = 0;
	}

	rc = of_property_read_u32(dev_node,
				"wusb3801,drp-toggle-time", &val);
	data->dttime = (u8)val;
	if (rc || (data->dttime != WUSB3801_TGL_40MS)) {
		dev_err(cdev, "Fixed drp time and set default (40ms:40ms)\n");
		data->dttime = WUSB3801_TGL_40MS;
		rc = 0;
	}
	
	dev_dbg(cdev, "init_mode:%d dfp_power:%d toggle_time:%d\n",
			data->init_mode, data->dfp_power, data->dttime);

out:
	return rc;
}
#endif


#if 1//!defined(__MEDIATEK_PLATFORM__)
static enum dual_role_property wusb3801_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

 /* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
static int dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
			enum dual_role_property prop,
			unsigned int *val)
{
	struct wusb3801_chip *chip;
	struct i2c_client *client = dual_role_get_drvdata(dual_role);

	int ret = 0;

	if (!client)
	{
		pr_info("%s: Get prop %d fail. Can not get drvdata!\n",__func__, prop);
		return -EINVAL;
	}

	chip = i2c_get_clientdata(client);

	mutex_lock(&chip->mlock);
	if (chip->state == WUSB3801_STATE_ATTACHED_SRC) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			ret = -EINVAL;
	} else if (chip->state == WUSB3801_STATE_ATTACHED_SNK) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			ret = -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			ret = -EINVAL;
	}
	mutex_unlock(&chip->mlock);

	pr_info("%s: chip->state = 0x%02x, prop = %d, val = %d\n",__func__,chip->state, prop, *val);
	return ret;
}

/* Decides whether userspace can change a specific property */
static int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				enum dual_role_property prop) {

	if (prop == DUAL_ROLE_PROP_MODE) {
                pr_info("%s: prop = %d, is DUAL_ROLE_PROP_MODE\n",__func__,prop);
		return 1;
	} else {
                pr_info("%s: prop = %d, not DUAL_ROLE_PROP_MODE\n",__func__,prop);
		return 1; // Patch for PowerRole & DataRole Swap
	}
}



/************************************************************************
 *
 *  dual_role_set_prop
 *
 *  Description :
 *  -------------
 *  Force set chip to a specified role and wait for final state.
 *  there are 2 different implementations. user can choice one by enable/disable
 *  __WITH_POWER_BANK_MODE__ definition.
 *  When __WITH_POWER_BANK_MODE__ is enabled. the Role swap is detemined by software.
 *  the software will force set the mode to either SOURCE or SINK, then swap role if 
 *  detection failure.
 *  When __WITH_POWER_BANK_MODE__ is disable, the role swap is determined by hardware.
 ************************************************************************/
 
/* ***********************************************************************
 * Callback for "echo <value> > /sys/class/dual_role_usb/<name>/<property>"
 * Block until the entire final state is reached.
 * Blocking is one of the better ways to signal when the operation
 * is done.
 * This function tries to switched to Attached.SRC or Attached.SNK
 * by forcing the mode into SRC or SNK.
 * On failure, we fall back to previous state machine.
 ************************************************************************/

static int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				const unsigned int *val) {
#if defined(__WITH_POWER_BANK_MODE__)
	struct wusb3801_chip *chip;

	struct i2c_client *client = dual_role_get_drvdata(dual_role);

	u8 mode, target_state, fallback_mode, fallback_state;
	int rc;

	struct device *cdev;
	long timeout;

	if (!client)
		return -EIO;

	chip = i2c_get_clientdata(client);
	cdev = &client->dev;

	/* WUSB3801 only support MODE change. So (DFP, SRC, HOST) are coupled, and
	   (UFP, SNK, DEVICE) are coupled. Swap any role will cause the MODE change.
	*/
	if (prop == DUAL_ROLE_PROP_MODE) {
		if (*val == DUAL_ROLE_PROP_MODE_DFP) {
			dev_dbg(cdev, "%s: Setting DFP mode\n", __func__);
			mode = WUSB3801_SRC;
			fallback_mode = WUSB3801_SNK;
			target_state = WUSB3801_STATE_ATTACHED_SRC;
			fallback_state = WUSB3801_STATE_ATTACHED_SNK;
		} else if (*val == DUAL_ROLE_PROP_MODE_UFP) {
			dev_dbg(cdev, "%s: Setting UFP mode\n", __func__);
			mode = WUSB3801_SNK;
			fallback_mode = WUSB3801_SRC;
			target_state = WUSB3801_STATE_ATTACHED_SNK;
			fallback_state = WUSB3801_STATE_ATTACHED_SRC;
		} else {
			dev_err(cdev, "%s: Trying to set invalid mode\n",
								__func__);
			return -EINVAL;
		}
	} else if (prop == DUAL_ROLE_PROP_PR) {
		if (*val == DUAL_ROLE_PROP_PR_SRC) {
			dev_dbg(cdev, "%s: Setting SRC mode\n", __func__);
			mode = WUSB3801_SRC;
			fallback_mode = WUSB3801_SNK;
			target_state = WUSB3801_STATE_ATTACHED_SRC;
			fallback_state = WUSB3801_STATE_ATTACHED_SNK;
		} else if (*val == DUAL_ROLE_PROP_PR_SNK) {
			dev_dbg(cdev, "%s: Setting SNK mode\n", __func__);
			mode = WUSB3801_SNK;
			fallback_mode = WUSB3801_SRC;
			target_state = WUSB3801_STATE_ATTACHED_SNK;
			fallback_state = WUSB3801_STATE_ATTACHED_SRC;
		} else {
			dev_err(cdev, "%s: Trying to set invalid mode\n",
								__func__);
			return -EINVAL;
		}
	} else if (prop == DUAL_ROLE_PROP_DR) {
		if (*val == DUAL_ROLE_PROP_DR_HOST) {
			dev_dbg(cdev, "%s: Setting HOST mode\n", __func__);
			mode = WUSB3801_SRC;
			fallback_mode = WUSB3801_SNK;
			target_state = WUSB3801_STATE_ATTACHED_SRC;
			fallback_state = WUSB3801_STATE_ATTACHED_SNK;
		} else if (*val == DUAL_ROLE_PROP_DR_DEVICE) {
			dev_dbg(cdev, "%s: Setting DEVICE mode\n", __func__);
			mode = WUSB3801_SNK;
			fallback_mode = WUSB3801_SRC;
			target_state = WUSB3801_STATE_ATTACHED_SNK;
			fallback_state = WUSB3801_STATE_ATTACHED_SRC;
		} else {
			dev_err(cdev, "%s: Trying to set invalid mode\n",
								__func__);
			return -EINVAL;
		}
	} else {
		dev_err(cdev, "%s: Property cannot be set\n", __func__);
		return -EINVAL;
	}

	if (chip->state == target_state)
		return 0;

	mutex_lock(&chip->mlock);

    dev_info(cdev, "%s: enter wusb3801_set_mode mode = 0x%02x\n",__func__,mode);
	rc = wusb3801_set_mode(chip, (u8)mode);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		if (IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		mutex_unlock(&chip->mlock);
		return rc;
	}
    dev_info(cdev, "%s: enter wusb3801_set_chip_state WUSB3801_STATE_ERROR_RECOVERY\n",__func__);
	rc = wusb3801_set_chip_state(chip, WUSB3801_STATE_ERROR_RECOVERY);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		if (IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		mutex_unlock(&chip->mlock);
		return rc;
	}

	wusb3801_detach(chip);

	mutex_unlock(&chip->mlock);

	timeout = wait_event_interruptible_timeout(mode_switch,
			chip->state == target_state,
			msecs_to_jiffies(ROLE_SWITCH_TIMEOUT));

	if (timeout > 0)
	{
		chip->defer_init = 1;	// Recover the mode register setting to initial value when detached
		return 0;
	}
	else
	{
		if(wusb3801_is_waiting_for_vbus(chip))
		{
			dev_err(cdev, "%s: Device is blocking on waiting for VBus.\n", __func__);
		}	
	}

	mutex_lock(&chip->mlock);
	rc = wusb3801_set_mode(chip, (u8)fallback_mode);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		if (IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to set mode\n", __func__);
		mutex_unlock(&chip->mlock);
		return rc;
	}
	rc = wusb3801_set_chip_state(chip, WUSB3801_STATE_ERROR_RECOVERY);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		if (IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to set state\n", __func__);
		mutex_unlock(&chip->mlock);
		return rc;
	}
	mutex_unlock(&chip->mlock);

	timeout = wait_event_interruptible_timeout(mode_switch,
			chip->state == fallback_state,
			msecs_to_jiffies(ROLE_SWITCH_TIMEOUT));

	mutex_lock(&chip->mlock);
	//if (IS_ERR_VALUE(wusb3801_set_mode(chip,	chip->pdata->init_mode)))
	//	dev_err(cdev, "%s: failed to set init mode\n", __func__);
	chip->defer_init = 1;	// Deffer the mode register setting to detached
	mutex_unlock(&chip->mlock);

	return -EIO;


#else /*__WITH_POWER_BANK_MODE__*/

	struct wusb3801_chip *chip;
	struct i2c_client *client = dual_role_get_drvdata(dual_role);
	uint8_t mode, target_state, fallback_state;
	int rc;
	struct device *cdev;
	long timeout;

	if (!client)
		return -EIO;

	chip = i2c_get_clientdata(client);
	cdev = &client->dev;

    dev_dbg(cdev, "%s: chip->state = 0x%02x, prop = %d\n",__func__,chip->state,prop);

	if (prop == DUAL_ROLE_PROP_MODE) {
		if (*val == DUAL_ROLE_PROP_MODE_DFP) {
			dev_dbg(cdev, "%s: Setting SRC mode\n", __func__);
			mode           = WUSB3801_DRP_PREFER_SRC;
			target_state   = WUSB3801_STATE_ATTACHED_SRC;
			fallback_state = WUSB3801_STATE_ATTACHED_SNK;
		} else if (*val == DUAL_ROLE_PROP_MODE_UFP) {
			dev_dbg(cdev, "%s: Setting SNK mode\n", __func__);
			mode           = WUSB3801_DRP_PREFER_SNK;
			target_state   = WUSB3801_STATE_ATTACHED_SNK;
			fallback_state = WUSB3801_STATE_ATTACHED_SRC;
		} else {
			dev_err(cdev, "%s: Trying to set invalid mode\n",
								__func__);
			return -EINVAL;
		}
	} else if (prop == DUAL_ROLE_PROP_PR) {
		if (*val == DUAL_ROLE_PROP_PR_SRC) {
			dev_dbg(cdev, "%s: Setting SRC mode\n", __func__);
			mode		   = WUSB3801_DRP_PREFER_SRC;
			target_state   = WUSB3801_STATE_ATTACHED_SRC;
			fallback_state = WUSB3801_STATE_ATTACHED_SNK;
		} else if (*val == DUAL_ROLE_PROP_PR_SNK) {
			dev_dbg(cdev, "%s: Setting SNK mode\n", __func__);
			mode		   = WUSB3801_DRP_PREFER_SNK;
			target_state   = WUSB3801_STATE_ATTACHED_SNK;
			fallback_state = WUSB3801_STATE_ATTACHED_SRC;
		} else {
			dev_err(cdev, "%s: Trying to set invalid mode\n",
								__func__);
			return -EINVAL;
		}
	} else if (prop == DUAL_ROLE_PROP_DR) {
		if (*val == DUAL_ROLE_PROP_DR_HOST) {
			dev_dbg(cdev, "%s: Setting HOST mode\n", __func__);
			mode		   = WUSB3801_DRP_PREFER_SRC;
			target_state   = WUSB3801_STATE_ATTACHED_SRC;
			fallback_state = WUSB3801_STATE_ATTACHED_SNK;
		} else if (*val == DUAL_ROLE_PROP_DR_DEVICE) {
			dev_dbg(cdev, "%s: Setting DEVICE mode\n", __func__);
			mode		   = WUSB3801_DRP_PREFER_SNK;
			target_state   = WUSB3801_STATE_ATTACHED_SNK;
			fallback_state = WUSB3801_STATE_ATTACHED_SRC;
		} else {
			dev_err(cdev, "%s: Trying to set invalid mode\n",
								__func__);
			return -EINVAL;
		}
	} else {
		dev_err(cdev, "%s: Property cannot be set\n", __func__);
		return -EINVAL;
	}

	if (chip->state == target_state)
		return 0;

	mutex_lock(&chip->mlock);
    dev_info(cdev, "%s: enter wusb3801_set_mode mode = 0x%02x\n",__func__,mode);
	rc = wusb3801_set_mode(chip, (uint8_t)mode);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		if (IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		mutex_unlock(&chip->mlock);
		return rc;
	}

	rc = wusb3801_set_chip_state(chip, WUSB3801_STATE_ERROR_RECOVERY);
	if (IS_ERR_VALUE((unsigned long)rc)) {
		if (IS_ERR_VALUE((unsigned long)wusb3801_reset_device(chip)))
			dev_err(cdev, "%s: failed to reset\n", __func__);
		mutex_unlock(&chip->mlock);
		return rc;
	}

	wusb3801_detach(chip);

	mutex_unlock(&chip->mlock);

	timeout = wait_event_interruptible_timeout(mode_switch,
			((chip->state == target_state) || (chip->state == fallback_state)),
			msecs_to_jiffies(ROLE_SWITCH_TIMEOUT));

	if (timeout > 0 )
	{
		if(chip->state == target_state)
		    return 0;
	}
	else
	{
		if(wusb3801_is_waiting_for_vbus(chip))
		{
			dev_err(cdev, "%s: Device is blocking on waiting for VBus.\n", __func__);
		}	
	}
	mutex_lock(&chip->mlock);
	if (IS_ERR_VALUE((unsigned long)wusb3801_set_mode(chip,	chip->pdata->init_mode)))
		dev_err(cdev, "%s: failed to set init mode\n", __func__);
	mutex_unlock(&chip->mlock);

	return -EIO;
#endif /*__WITH_POWER_BANK_MODE__*/
}

#endif /*__MEDIATEK_PLATFORM__*/

#if defined(__QCOM_PLATFORM__)
static int wusb3801_pinctrl_init(struct wusb3801_chip *chip)//(struct mxt_data *data)
{
	int error;

	/* Get pinctrl if target uses pinctrl */
	chip->wusb3801_pinctrl = devm_pinctrl_get((&chip->client->dev));
	if (IS_ERR_OR_NULL(chip->wusb3801_pinctrl)) {
		printk(
			"Device does not use pinctrl\n");
		error = PTR_ERR(chip->wusb3801_pinctrl);
		chip->wusb3801_pinctrl = NULL;
		return error;
	}

	chip->gpio_state_active
		= pinctrl_lookup_state(chip->wusb3801_pinctrl, "pmx_wusb3801_active");
	if (IS_ERR_OR_NULL(chip->gpio_state_active)) {
		printk(
			"Can not get wusb3801 default pinstate\n");
		error = PTR_ERR(chip->gpio_state_active);
		chip->wusb3801_pinctrl = NULL;
		return error;
	}

	chip->gpio_state_suspend
		= pinctrl_lookup_state(chip->wusb3801_pinctrl, "pmx_wusb3801_suspend");
	if (IS_ERR_OR_NULL(chip->gpio_state_suspend)) {
		printk(
			"Can not get wusb3801 sleep pinstate\n");
		error = PTR_ERR(chip->gpio_state_suspend);
		chip->wusb3801_pinctrl = NULL;
		return error;
	}

	return 0;
}

#if 0
static int wusb3801_pinctrl_select(struct wusb3801_chip *chip, bool on)
{
	struct pinctrl_state *pins_state;
	int error;

	pins_state = on ? chip->gpio_state_active
		: chip->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		error = pinctrl_select_state(chip->wusb3801_pinctrl, pins_state);
		if (error) {
			printk(
				"can not set %s pins\n",
				on ? "pmx_wusb3801_active" : "pmx_wusb3801_suspend");
			return error;
		}
	} else {
		printk(
			"not a valid '%s' pinstate\n",
				on ? "pmx_wusb3801_active" : "pmx_wusb3801_suspend");
	}

	return 0;
}
#endif
#endif




/* dingjun 2017.12.04 start */
#ifdef CONFIG_OF
static int wusb3801x_parse_dt(struct device *dev, struct wusb3801_chip *chip)
{
	int r = 0;
	struct device_node *np = dev->of_node;

		printk("%s,%d:\n",__func__,__LINE__);
	np = of_find_compatible_node(NULL, NULL, "mediatek,wusb3801-gpio-v2");
		printk("%s,%d:\n",__func__,__LINE__);

	if (np) {
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
		r = of_get_named_gpio(np, "gpio-irq-std", 0);
		if (r < 0)
			printk("%s: get NFC IRQ GPIO failed (%d)", __FILE__, r);
		else
			chip->irq_gpio = r;
		r = 0;
#else
		of_property_read_u32_array(np, "gpio-irq", &(chip->irq_gpio),
					   1);
#endif
	} else {
			printk("%s : get gpio num err.\n", __func__);
		return -1;
	}


	printk("[wusb3801x] %s : irq_gpio[%d]\n", __func__, chip->irq_gpio);
	return r;
}
#else
static int wusb3801x_parse_dt(struct device *dev, struct wusb3801_chip *chip)
{
	return 0;
}
#endif
/* dingjun 2017.12.04 end */


/* dingjun add 20171211 start */
int register_typec_switch_callback(struct typec_switch_data *new_driver)
{

       printk("%s,%d-Register driver %s %d\n", new_driver->name, new_driver->type,__func__,__LINE__);
       if( DEVICE_TYPE == new_driver->type )
       {
           g_typec.device_driver = new_driver;
	    g_typec.device_driver->on = 0;
          if (g_device_connected)
          {
		    g_typec.device_driver->enable(g_typec.device_driver->priv_data);		
          }
           return 0;
       }

       if( HOST_TYPE == new_driver->type )
       {
           g_typec.host_driver = new_driver;
	    g_typec.host_driver->on = 0;
	    if(g_host_connected)
	    {
	           g_typec.host_driver->enable(g_typec.host_driver->priv_data);
	    }
           return 0;
       }
   
	return -1;
}
EXPORT_SYMBOL_GPL(register_typec_switch_callback);

int unregister_typec_switch_callback(struct typec_switch_data *new_driver)
{
	if ((DEVICE_TYPE == new_driver->type) && (g_typec.device_driver == new_driver))
		g_typec.device_driver = NULL;

	if ((HOST_TYPE == new_driver->type) && (g_typec.host_driver == new_driver))
		g_typec.host_driver = NULL;

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_typec_switch_callback);
/* dingjun add 20171211 end */	

static int wusb3801_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct wusb3801_chip *chip;
	struct device *cdev = &client->dev;
	struct device_node *node;
//	struct power_supply *usb_psy;
#if 1//!defined(__MEDIATEK_PLATFORM__)
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
#else /*__MEDIATEK_PLATFORM*/
//	struct power_supply *usb_psy;
#endif /*__MEDIATEK_PLATFORM__*/
	int ret = 0;

		printk("%s,%d:\n",__func__,__LINE__);
	
	dev_info(cdev,"wusb3801_probe start \n");
#if !defined(__MEDIATEK_PLATFORM__)
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(cdev, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}
#endif /*__MEDIATEK_PLATFORM__*/

	printk("%s,%d:\n",__func__,__LINE__);
	
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA |
				I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(cdev, "smbus data not supported!\n");
		return -EIO;
	}

	
		printk("%s,%d:\n",__func__,__LINE__);
		
	chip = devm_kzalloc(cdev, sizeof(struct wusb3801_chip), GFP_KERNEL);
	if (!chip) {
		dev_err(cdev, "can't alloc wusb3801_chip\n");
		return -ENOMEM;
	}
	
	printk("%s,%d:\n",__func__,__LINE__);
	
	/* dingjun add 2017.12.04 start */	
		ret = wusb3801x_parse_dt(&client->dev, chip);
		if (ret) {
			printk("%s,%d : can't parse dts ret =%d\n", __func__, __LINE__,ret);
			return ret;
		}		
		printk("%s,%d:\n",__func__,__LINE__);
	ret = gpio_request(chip->irq_gpio,
#if (!defined(CONFIG_MTK_GPIO) || defined(CONFIG_MTK_GPIOLIB_STAND))
			   "gpio-irq-std"
#else
			   "gpio-irq"
#endif
			   );
	if (ret) {
		printk("%s,%d : gpio_request failed\n", __func__,__LINE__);
		goto err1;
	}
	printk("%s,%d : IRQ GPIO = %d\n", __func__, __LINE__, chip->irq_gpio);
	ret = gpio_direction_input(chip->irq_gpio);
	if (ret) {
		printk("%s,%d : gpio_direction_input failed\n", __func__,__LINE__);
		ret = -ENODEV;
		goto err1;
	}	

		printk("%s,%d:\n",__func__,__LINE__);
		node = of_find_compatible_node(NULL, NULL, "mediatek,wusb3801x-eint");
		printk("%s,%d:\n",__func__,__LINE__);

	if (node) {

		wusb3801x_irq = irq_of_parse_and_map(node, 0);

		//client->irq = wusb3801x_irq;
		chip->irq = wusb3801x_irq;

		printk("%s : client->irq=%d, wusb3801x_irq=%d\n", __func__, client->irq, wusb3801x_irq);

		//enable_irq_wake(client->irq);

	} else {
		printk("%s,%d : can not find wusb3801x eint compatible node\n", __func__,__LINE__);
	}
	/* dingjun add 2017.12.04 end */
	
		printk("%s,%d:\n",__func__,__LINE__);
	

	chip->client = client;
	chip->client->addr = 0x60; /* add for test 2017.12.04 */
    printk("[wusb3801_probe]: client->addr=0x%x\n",client->addr);
	i2c_set_clientdata(client, chip);

	ret = wusb3801_read_device_id(chip);
	if (ret){
		dev_err(cdev, "wusb3801 doesn't find, try again\n");
		ret = wusb3801_read_device_id(chip);
		if(ret){
			dev_err(cdev, "wusb3801 doesn't find, stop find\n");
			goto err1;
		}
	}
	if (1) { //(&client->dev.of_node){
		struct wusb3801_data *data = devm_kzalloc(cdev,
				sizeof(struct wusb3801_data), GFP_KERNEL);

		if (!data) {
			dev_err(cdev, "can't alloc wusb3801_data\n");
			ret = -ENOMEM;
			goto err1;
		}

		chip->pdata = data;

		#if 0
		ret = wusb3801_parse_dt(chip);
		if (ret) {
			dev_err(cdev, "can't parse dt\n");
			goto err2;
		}
		#endif
		
		
	} else {
		chip->pdata = client->dev.platform_data;
	}

	//ret = wusb3801_init_gpio(chip);
	//if (ret) {
	//	dev_err(cdev, "fail to init gpio\n");
	//	goto err2;
	//}

	chip->type      = WUSB3801_TYPE_INVALID;
	chip->state     = WUSB3801_STATE_ERROR_RECOVERY;
	chip->attached  = 0;
	chip->bc_lvl    = WUSB3801_SNK_3000MA;
	chip->pdata->init_mode = WUSB3801_DRP_PREFER_SRC_ACC;
	chip->ufp_power = 0;
	chip->defer_init = 0;	// Do the mode init when chip init
	//chip->usb_psy   = usb_psy;
#ifdef __TEST_CC_PATCH__	
	chip->reg4_val = 0;
	chip->cc_sts = 0xFF;
	chip->cc_test_flag = 0;
#endif

	chip->cc_wq = alloc_ordered_workqueue("wusb3801-wq", WQ_HIGHPRI);
	if (!chip->cc_wq) {
		dev_err(cdev, "unable to create workqueue wusb3801-wq\n");
		goto err2;
	}
	INIT_WORK(&chip->dwork, wusb3801_work_handler);
	//wake_lock_init(&chip->wlock, WAKE_LOCK_SUSPEND, "wusb3801_wake");
	mutex_init(&chip->mlock);

	ret = wusb3801_create_devices(cdev);
	if (IS_ERR_VALUE((unsigned long)ret)) {
		dev_err(cdev, "could not create devices\n");
		goto err3;
	}

#if defined(__MEDIATEK_PLATFORM__)

	//dual_role_service.drv_data = client;
//	chip->irq_gpio = gpio_to_irq(WUSB3801X_INT_PIN);
	dev_info(cdev, "wusb3801 chip->irq_gpio =%d \n",wusb3801x_irq);
	//chip->dual_role = &dual_role_service;
	
	//i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
	
	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		desc = devm_kzalloc(cdev, sizeof(struct dual_role_phy_desc),
					GFP_KERNEL);
		if (!desc) {
			dev_err(cdev, "unable to allocate dual role descriptor\n");
			goto err4;
		}
	
		dev_info(cdev, "%s: devm_dual_role_instance_register\n", __func__);
		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_local_prop;
		desc->set_property = dual_role_set_prop;
		desc->properties = wusb3801_drp_properties;
		desc->num_properties = ARRAY_SIZE(wusb3801_drp_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		dual_role = devm_dual_role_instance_register(cdev, desc);
		dual_role->drv_data = client;
		chip->dual_role = dual_role;
		chip->desc = desc;
	}

	ret = devm_request_irq(cdev, wusb3801x_irq,
				wusb3801_interrupt,
				IRQF_TRIGGER_LOW,/*IRQF_TRIGGER_HIGH,*/
				"wusb3801_int_irq", chip);
	if (ret) {
		dev_err(cdev, "failed to reqeust IRQ\n");
		goto err4;
	}	
	 
	 ret = wusb3801_reset_device(chip);
	 if (ret) {
		dev_err(cdev, "failed to initialize\n");
		goto err5;
	}
#else /*__MEDIATEK_PLATFORM__*/
   	chip->usb_psy = usb_psy;
	chip->irq_gpio = gpio_to_irq(chip->pdata->int_gpio);
	gpio_direction_input(chip->pdata->int_gpio);

	if (chip->irq_gpio < 0) {
		dev_err(cdev, "could not register int_gpio\n");
		ret = -ENXIO;
		goto err4;
	}
	
	//i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_INTERRUPT);
#ifdef __QCOM_PLATFORM__
	ret = wusb3801_pinctrl_init(chip);
	if (ret)
		dev_err(cdev, "No pinctrl support\n");
	//ret = wusb3801_pinctrl_select(chip, 1);
	ret = pinctrl_select_state(chip->wusb3801_pinctrl, chip->gpio_state_active);
	if (ret)
		dev_err(cdev, "select failedt\n");
#endif
	ret = devm_request_irq(cdev, chip->irq_gpio,
				wusb3801_interrupt,
				IRQF_TRIGGER_LOW,
				"wusb3801_int_irq", chip);
	if (ret) {
		dev_err(cdev, "failed to reqeust IRQ\n");
		goto err4;
	}

	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		desc = devm_kzalloc(cdev, sizeof(struct dual_role_phy_desc),
					GFP_KERNEL);
		if (!desc) {
			dev_err(cdev, "unable to allocate dual role descriptor\n");
			goto err4;
		}
	
		desc->name = "otg_default";
		desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
		desc->get_property = dual_role_get_local_prop;
		desc->set_property = dual_role_set_prop;
		desc->properties = wusb3801_drp_properties;
		desc->num_properties = ARRAY_SIZE(wusb3801_drp_properties);
		desc->property_is_writeable = dual_role_is_writeable;
		dual_role = devm_dual_role_instance_register(cdev, desc);
		dual_role->drv_data = client;
		chip->dual_role = dual_role;
		chip->desc = desc;
	}
	ret = wusb3801_reset_device(chip);
	if (ret) {
		dev_err(cdev, "failed to initialize\n");
		goto err5;
	}
#endif /*__MEDIATEK_PLATFORM__*/
//	printk("%s,%d: enable irq=%x\n",__func__,__LINE__,chip->irq_gpio);
//	enable_irq_wake(chip->irq_gpio);
	printk("%s,%d: enable irq=%x\n",__func__,__LINE__,wusb3801x_irq);
	enable_irq_wake(wusb3801x_irq);
	
	//huanglei add for debug init reg status begin	
	ret = i2c_smbus_read_byte_data(chip->client, WUSB3801_REG_STATUS);
	if (IS_ERR_VALUE((unsigned long)ret)) {
		dev_err(cdev, "%s: failed to read reg status\n", __func__);
	}
	dev_err(cdev, "%s WUSB3801_REG_STATUS : 0x%02x\n", __func__, ret);	
	//huanglei add for debug init reg status end
		
#ifdef __TEST_CC_PATCH__	
	if(BITS_GET(ret, 0x80) == 1 && BITS_GET(ret, 0x60) != 0){
		printk("wusb3801_probe end no sm rst\n");
		return 0;	
	}
#endif	
	
	i2c_smbus_write_byte_data(chip->client,
			WUSB3801_REG_CONTROL1, BIT_REG_CTRL1_SM_RST);
			
	dev_info(cdev,"wusb3801_probe end \n");
	return 0;

#if 1//!defined(__MEDIATEK_PLATFORM__)
err5:
	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF))
		devm_kfree(cdev, chip->desc);
#endif /*__MEDIATEK_PLATFORM__*/
err4:
	wusb3801_destory_device(cdev);
err3:
	destroy_workqueue(chip->cc_wq);
	mutex_destroy(&chip->mlock);
	//wake_lock_destroy(&chip->wlock);
	wusb3801_free_gpio(chip);
err2:
	if (1) //(&client->dev.of_node)
		devm_kfree(cdev, chip->pdata);
err1:
	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, chip);

	return ret;
}

static int wusb3801_remove(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;

	if (!chip) {
		dev_err(cdev, "%s : chip is null\n", __func__);
		return -ENODEV;
	}

//	if (chip->irq_gpio > 0)
//		devm_free_irq(cdev, chip->irq_gpio, chip);
	if (wusb3801x_irq > 0)
		devm_free_irq(cdev, wusb3801x_irq, chip);

#if 1//!defined(__MEDIATEK_PLATFORM__)
	if (IS_ENABLED(CONFIG_DUAL_ROLE_USB_INTF)) {
		devm_dual_role_instance_unregister(cdev, chip->dual_role);
		devm_kfree(cdev, chip->desc);
	}
#endif /*__MEDIATEK_PLATFORM__*/

	wusb3801_destory_device(cdev);
	destroy_workqueue(chip->cc_wq);
	mutex_destroy(&chip->mlock);
	//wake_lock_destroy(&chip->wlock);
	wusb3801_free_gpio(chip);

	if (1) //(&client->dev.of_node)
		devm_kfree(cdev, chip->pdata);

	i2c_set_clientdata(client, NULL);
	devm_kfree(cdev, chip);

	return 0;
}

static void wusb3801_shutdown(struct i2c_client *client)
{
	struct wusb3801_chip *chip = i2c_get_clientdata(client);
	struct device *cdev = &client->dev;

	if (IS_ERR_VALUE((unsigned long)wusb3801_set_mode(chip, WUSB3801_SNK)) ||
			IS_ERR_VALUE((unsigned long)wusb3801_set_chip_state(chip,
					WUSB3801_STATE_ERROR_RECOVERY)))
		dev_err(cdev, "%s: failed to set sink mode\n", __func__);
}

#ifdef CONFIG_PM
static int wusb3801_suspend(struct device *dev)
{
	return 0;
}

static int wusb3801_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops wusb3801_dev_pm_ops = {
	.suspend = wusb3801_suspend,
	.resume  = wusb3801_resume,
};
#endif

static const struct i2c_device_id wusb3801_id_table[] = {
	{"wusb3801", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, wusb3801_id_table);

#ifdef CONFIG_OF

static struct of_device_id wusb3801_match_table[] = {
#if defined(__MEDIATEK_PLATFORM__)
	{ .compatible = "mediatek,wusb3801x",},
	//{ .compatible = "mediatek,wusb3801-gpio-v2",},
#else /*__MEDIATEK_PLATFORM__*/
	{ .compatible = "qcom,wusb3801x",},
#endif /*__MEDIATEK_PLATFORM__*/
	{ },
};
#else /*!CONFIG_OF*/
#define wusb3801_match_table NULL
#endif /*!CONFIG_OF*/

static const unsigned short normal_i2c[] = {WUSB3801_SLAVE_ADDR0, WUSB3801_SLAVE_ADDR1, I2C_CLIENT_END};
static struct i2c_driver wusb3801_i2c_driver = {
	.driver = {
		.name = "wusb3801",
		.owner = THIS_MODULE,
		.of_match_table = wusb3801_match_table,
#ifdef CONFIG_PM
		.pm = &wusb3801_dev_pm_ops,
#endif
	},
	.probe = wusb3801_probe,
	.remove = wusb3801_remove,
	.shutdown = wusb3801_shutdown,
	.id_table = wusb3801_id_table,
	.address_list = (const unsigned short *) normal_i2c,
};


/*dingjun 20171204 start*/
#ifndef KERNEL318MTK

static int wusb3801_platform_probe(struct platform_device *pdev)
{
	printk("%s,%d:\n",__func__,__LINE__);
	return 0;
}

static int wusb3801_platform_remove(struct platform_device *pdev)
{
	printk("%s,%d:\n",__func__,__LINE__);
	return 0;
}

/*  platform driver */
static const struct of_device_id wusb3801_dev_of_match[] = {
	{
	.compatible = "mediatek,wusb3801-gpio-v2",
	},
	{},
};

static struct platform_driver wusb3801_platform_driver = {
	.probe = wusb3801_platform_probe,
	.remove = wusb3801_platform_remove,
	.driver = {
		.name =  "wusb3801",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = wusb3801_dev_of_match,
#endif
	},
};
#endif /* KERNEL318MTK */
/*dingjun 20171204 end*/


static __init int wusb3801_i2c_init(void)
{
	printk("%s,%d:\n",__func__,__LINE__);
	dual_role_class_init();
	platform_driver_register(&wusb3801_platform_driver);
	return i2c_add_driver(&wusb3801_i2c_driver);
}

static __exit void wusb3801_i2c_exit(void)
{
	printk("%s,%d:\n",__func__,__LINE__);
	dual_role_class_exit();
	i2c_del_driver(&wusb3801_i2c_driver);
}

late_initcall(wusb3801_i2c_init);//wuxiwen for type-c headset default in power on dump
module_exit(wusb3801_i2c_exit);

MODULE_AUTHOR("lhuang@sh-willsemi.com");
MODULE_DESCRIPTION("WUSB3801x USB Type-C driver for MSM and Qcom/Mediatek/Others Linux/Android Platform");
MODULE_LICENSE("GPL v2");
