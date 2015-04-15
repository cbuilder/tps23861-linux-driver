/*
 * TPS23861 IEEE 802.3at Quad Port Power-over-Ethernet PSE Controller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TPS23861_H__
#define __TPS23861_H__

#define TPS23861_PORTS_NUM	4

#define INT_OCCURED_REG		0x00 /* Events occured */
#define INT_ENABLE_REG		0x01 /* Interrupt enable */
#define DET_EVENT_COR_REG	0x05 /* Detect & Class event per port (CoR) */
#define FAULT_EVENT_COR_REG	0x07 /* Disconnect or Ilim event (CoR) */
#define PT_STATUS_BASE		0x0C /* Detection & Class status */
#define PT_MODE_REG		0x12 /* Ports operating mode */
#define PT_DIS_EN_REG		0x13 /* Disconnect enable */
#define PT_DET_CLAS_EN_REG	0x14 /* Detect and class enable */
#define PT_POWER_EN_REG		0x19 /* Ports power control */
#define RESET_REG		0x1A /* Clear different port registers */
#define TEMPERATURE_REG		0x2C /* Chip temperature */
#define INPUT_V_REG		0x2E /* 2 bytes, little endian */
#define FW_REV_REG		0x41 /* device firmware revision */
#define DID_SIL_REV_REG		0x43 /* device ID, silicon revision */

/* Interrupts */
#define PESEN 	1 << 0
#define PGSEN	1 << 1
#define DISEN 	1 << 2
#define DEEN	1 << 3
#define CLCEN	1 << 4
#define IFEN	1 << 5
#define STRTEN	1 << 6
#define SUPEN	1 << 7
#define ALLEN	0xFF
#define CLRAIN	1 << 7 /* clear all */
/* Events */
#define PEC	1 << 0
#define PGC	1 << 1
#define DISF	1 << 2
#define DETC	1 << 3
#define CLASC	1 << 4
#define IFAULT	1 << 5
#define STRTF	1 << 6
#define SUPF	1 << 7

/* Input voltage */
#define INPUT_V_SZ		2
/* Power enable register */
#define P_ON			1
#define P_OFF			0
#define PT_PWR_ON_SHIFT		0
#define PT_PWR_OFF_SHIFT	4

/* Port current and voltage info */
#define PT_DC_REGS		0x30
#define PT_DC_INFO_SZ		16

/* Operating mode register */
#define PORT_MODE_OFF		0
#define PORT_MODE_MANUAL	1
#define PORT_MODE_SEMIAUTO	2
#define PORT_MODE_AUTO		3
#define PT_MODE_FIELD_WIDTH	2
#define PT_MODE_MASK		0x3
/* Disconnect, detect and class enable */
#define DIS_EN			0x1
#define DET_CLAS_EN		0x11

/* Detection statuses */
#define CLASS_UNKNOWN		0
#define CLASS1			0x1
#define CLASS2			0x2
#define CLASS3			0x3
#define CLASS4			0x4
#define CLASS0			0x6
#define CLASS_OVERCURRENT	0x7
#define CLASS_MISMATCH		0x8
/* Classification statuses */
#define DETECT_R_VALID		0x4

#define PORT_SELECT( port_i ) (0x1 << (port_i))
#define ASCII_TO_DIGIT( ascii_digit ) (ascii_digit - 0x30) 

#define TEMP_LSB		652 /* 0.652 C */
#define VOLT_LSB		3662 /* 3.662 mV */
#define CURR_S250_LSB		62260 /* 62.260 uA */
#define CURR_S255_LSB		61039 /* 61.039 uA */

struct pt_dc_parm {
	s16 i;
	s16 v;
};

struct pt_dflt {
	int enable;
	int mode;
	int pwr;
};

struct tps23861_platform_data {
	u32 irq;
	struct pt_dflt pt_df[TPS23861_PORTS_NUM];
};

struct tps23861_data {
	struct i2c_client *client;
	struct tps23861_platform_data pdata;
	struct pt_dc_parm pt_dc[TPS23861_PORTS_NUM];
};

#endif /* __TPS23861_H__ */
