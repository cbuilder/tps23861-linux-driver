/*
 * TPS23861 IEEE 802.3at Quad Port Power-over-Ethernet PSE Controller
 *
 * Version 1.0 Release notes:
 * 1. Semiautomatic mode is untested.
 * 2. Legacy detect is untested.
 * 3. All condition values (periods, limits etc.) left as hardware default.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "tps23861.h"

static int tps23861_read_reg(struct tps23861_data *ddata, u8 reg)
{
	int res;
	res  = i2c_smbus_read_byte_data(ddata->client, reg);
	if (res < 0)
		dev_err(&ddata->client->dev, "read byte error %d\n", res);
	return res;
}

static int tps23861_write_reg(struct tps23861_data *ddata, u8 reg, u8 val)
{
	int res;
	res  = i2c_smbus_write_byte_data(ddata->client, reg, val);
	if (res < 0)
		dev_err(&ddata->client->dev, "write byte error %d\n", res);
	return res;
}

static int tps23861_read(struct tps23861_data *ddata,
			u8 reg, int len, void * values) {
	int res;
	u8 l;
	u8 *data = (char *)values;
	for (l = 0; l < len; l++) {
		res = i2c_smbus_read_byte_data(ddata->client, reg + l);
		if (res < 0) {
			dev_err(&ddata->client->dev, "read error %d\n", res);
			return res;
		}
		data[l] = (u8)res;
	}
	return l;
}

static int tps23861_irq_enable(struct tps23861_data *ddata, u8 interrupts) {
	return tps23861_write_reg(ddata, INT_ENABLE_REG, interrupts);
}

static int tps23861_irq_clear(struct tps23861_data *ddata, int irqn) {
	return tps23861_write_reg(ddata, RESET_REG, irqn);
}

static irqreturn_t tps23861_irq_handler(int irq, void *device_data)
{
	int i;
	u8 ports;
	u8 irq_flags;
	struct tps23861_data *ddata = (struct tps23861_data *)device_data;
	irq_flags = tps23861_read_reg(ddata, INT_OCCURED_REG);
	/* Critical faults */
	if (IFAULT & irq_flags) {
		/* Icut */
		ports = tps23861_read_reg(ddata, DISC_ICUT_COR_REG);
		for (i = 0; i < TPS23861_PORTS_NUM; i++) {
			if (ports & (1 << i))
				dev_info(&ddata->client->dev,
				    "port%d: overload occured", i);
		}
		/* Ilim */
		ports = tps23861_read_reg(ddata, ILIM_STRT_COR_REG);
		for (i = 0; i < TPS23861_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: exceeded current limit", i);
		}
	}
	if (SUPF & irq_flags)
		dev_info(&ddata->client->dev, "power supply fault\n");
	/* Deferrable events: PEC, PGC, DISF, DETC, CLASC, STRTF */
	if (STRTF & irq_flags) {
		ports = tps23861_read_reg(ddata, ILIM_STRT_COR_REG);
		for (i = 0; i < TPS23861_PORTS_NUM; i++) {
			if (ports & (1 << i))
				dev_info(&ddata->client->dev,
				    "port%d: start fault", i);
		}
	}
	if (DISF & irq_flags) {
		ports = tps23861_read_reg(ddata, DISC_ICUT_COR_REG);
		for (i = 0; i < TPS23861_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: powered device disconnected", i);
		}
	}
	if (CLASC & irq_flags) {
		ports = tps23861_read_reg(ddata, DET_EVENT_COR_REG);
		for (i = 0; i < TPS23861_PORTS_NUM; i++) {
			if (ports & (1 << (i + 4)))
				dev_info(&ddata->client->dev,
				    "port%d: powered device connected", i);
		}
	}
	tps23861_irq_clear(ddata, CLRAIN);
	return IRQ_HANDLED;
}

int pt_is_valid(struct device *dev, int pt_number) {
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	if (ddata->pdata.pt_df[pt_number].enable == 1)
		return 1;
	else
		return 0;
}

static u8 append_pt_mode_reg(u8 old_mode_reg, int pt_number, int mode) {
	u8 new_mode_reg;
	u8 pt_mask;
	pt_mask = ~(PT_MODE_MASK << (pt_number * PT_MODE_FIELD_WIDTH));
	new_mode_reg = mode << (pt_number * PT_MODE_FIELD_WIDTH);
	new_mode_reg = new_mode_reg | (pt_mask & old_mode_reg);
	return new_mode_reg;
}

static int tps23861_get_pt_mode(struct device *dev, int porti) {
	u8 mode;
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	mode = tps23861_read_reg(ddata, PT_MODE_REG);
	mode = (mode >> porti * PT_MODE_FIELD_WIDTH) & PT_MODE_MASK;
	return (int)mode;
}

static int tps23861_set_pt_mode(struct device *dev, int porti, int mode) {
	u8 mcfg;
	struct tps23861_data *ddata = dev_get_drvdata(dev);

	mcfg = tps23861_read_reg(ddata, PT_MODE_REG);
	mcfg = append_pt_mode_reg(mcfg, porti, mode);
	tps23861_write_reg(ddata, PT_MODE_REG, mcfg);

	if (mode == PORT_MODE_MANUAL) {
		/* Disable connection monitoring */
		mcfg = tps23861_read_reg(ddata, PT_DET_CLAS_EN_REG);
		mcfg &= ~(DET_CLAS_EN << porti);
		tps23861_write_reg(ddata, PT_DET_CLAS_EN_REG, mcfg);
		mcfg = tps23861_read_reg(ddata, PT_DIS_EN_REG);
		mcfg &= ~(DIS_EN << porti);
		tps23861_write_reg(ddata, PT_DIS_EN_REG, mcfg);
	} else if (mode == PORT_MODE_AUTO) {
		/* Enable connect detection */
		mcfg = tps23861_read_reg(ddata, PT_DET_CLAS_EN_REG);
		mcfg |= DET_CLAS_EN << porti;
		tps23861_write_reg(ddata, PT_DET_CLAS_EN_REG, mcfg);
		mcfg = tps23861_read_reg(ddata, PT_DIS_EN_REG);
		mcfg |= DIS_EN << porti;
		tps23861_write_reg(ddata, PT_DIS_EN_REG, mcfg);
	}
	return 0;
}

static int tps23861_switch_pt_power(struct device *dev, int porti, int state) {
	int res;
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	if (porti >= 0 && porti < TPS23861_PORTS_NUM) {
		if (PORT_MODE_AUTO == tps23861_get_pt_mode(dev, porti))
			return -EINVAL;
		if (state == 1)
			state = PORT_SELECT(porti) << PT_PWR_ON_SHIFT;
		else
			state = PORT_SELECT(porti) << PT_PWR_OFF_SHIFT;
	}
	else
		return (res = -EINVAL);
	return tps23861_write_reg(ddata, PT_POWER_EN_REG, state);
}


static int tps23861_apply_dt_defaults(struct device *dev) {
	int i;
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	for (i = 0; i < TPS23861_PORTS_NUM; i++) {
		if (!pt_is_valid(dev, i))
			tps23861_set_pt_mode(dev, i, PORT_MODE_OFF);
		else {
			tps23861_set_pt_mode(dev, i, ddata->pdata.pt_df[i].mode);
			if (ddata->pdata.pt_df[i].mode == PORT_MODE_MANUAL)
				tps23861_switch_pt_power(dev, i,
				    ddata->pdata.pt_df[i].pwr);
		}
	}
	return 0;
}

static ssize_t tps23861_show_temp(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int val;
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	val = tps23861_read_reg(ddata, TEMPERATURE_REG);
	val = val * TEMP_LSB - 20000;
	return snprintf(buf, 6, "%d.%01d\n", val/1000, (val%1000)/100);
}

static ssize_t tps23861_show_vin(struct device *dev,
		struct device_attribute *attr, char *buf) {
	u16 val;
	long vin;
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	tps23861_read(ddata, INPUT_V_REG, INPUT_V_SZ, &val);
	vin = VOLT_LSB * __le16_to_cpu(val);
	return snprintf(buf, 6, "%ld.%01ld\n",
			vin/1000000, (vin%1000000)/100000);
}

static ssize_t tps23861_store_pt_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	u8 mode;
	int port_idx = ASCII_TO_DIGIT(buf[0]);
	if (!pt_is_valid(dev, port_idx))
		return -EINVAL;
	if (!strncmp(buf + 1, "off", 3))
		mode = PORT_MODE_OFF;
	else if (!strncmp(buf + 1, "manual", 6))
		mode = PORT_MODE_MANUAL;
	else if (!strncmp(buf + 1, "semiauto", 8))
		mode = PORT_MODE_SEMIAUTO;
	else if (!strncmp(buf + 1, "auto", 4))
		mode = PORT_MODE_AUTO;
	else
		return -EINVAL;
	tps23861_set_pt_mode(dev, port_idx, mode);
	return count;
}

static ssize_t tps23861_store_pt_power(struct device *dev,
		const char *buf, size_t count, int state) {
	int res;
	long port_idx;
	res = kstrtol(buf, 10, &port_idx);
	if (res != 0)
		return res;
	if (!pt_is_valid(dev, port_idx))
		return -EINVAL;
	res = tps23861_switch_pt_power(dev, port_idx, state);
	if (res == 0)
		return count;
	else
		return res;
}

static ssize_t tps23861_store_pt_power_on(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	return tps23861_store_pt_power(dev, buf, count, P_ON);
}

static ssize_t tps23861_store_pt_power_off(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	return tps23861_store_pt_power(dev, buf, count, P_OFF);
}

static ssize_t tps23861_show_pt_info(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int i;
	int len;
	unsigned long val = 0;
	struct tps23861_data *ddata = dev_get_drvdata(dev);
	tps23861_read(ddata, PT_DC_REGS, PT_DC_INFO_SZ, ddata->pt_dc);
	len = sprintf(buf, "# mode voltage current\n"); /* column names */
	for (i = 0; i < TPS23861_PORTS_NUM; i++) {
		/* Port number */
		len += sprintf(buf+len, "%d ", i);
		/* Port mode of operation */
		val = tps23861_get_pt_mode(dev, i);
		if (val == PORT_MODE_OFF)
			len += sprintf(buf+len, "off ");
		else if (val == PORT_MODE_MANUAL)
			len += sprintf(buf+len, "manual ");
		else if (val == PORT_MODE_SEMIAUTO)
			len += sprintf(buf+len, "semiauto ");
		else if (val == PORT_MODE_AUTO)
			len += sprintf(buf+len, "auto ");
		else
			len += sprintf(buf+len, "unknown ");
		/* Port voltage */
		if (!pt_is_valid(dev, i))
			len += sprintf(buf+len,"* ");
		else {
			val = VOLT_LSB * __le16_to_cpu(ddata->pt_dc[i].v);
			len += sprintf(buf+len,"%lu.%01lu ",
					val/1000000, (val%1000000)/100000);
		}
		/* Port current intensity */
		if (!pt_is_valid(dev, i))
			len += sprintf(buf+len,"*\n");
		else {
			val = CURR_S250_LSB * __le16_to_cpu(ddata->pt_dc[i].i);
			len += sprintf(buf+len, "%lu.%03lu\n",
					val/1000000000, val/1000000);
		}
	}
	return len;
}

static DEVICE_ATTR(temp, S_IRUGO, tps23861_show_temp, NULL);
static DEVICE_ATTR(vin, S_IRUGO, tps23861_show_vin, NULL);
static DEVICE_ATTR(port_mode, S_IWUSR|S_IWGRP,
		NULL, tps23861_store_pt_mode);
static DEVICE_ATTR(port_power_on, S_IWUSR|S_IWUSR,
		NULL, tps23861_store_pt_power_on);
static DEVICE_ATTR(port_power_off, S_IWUSR|S_IWUSR,
		NULL, tps23861_store_pt_power_off);
static DEVICE_ATTR(port_info, S_IRUGO,
		tps23861_show_pt_info, NULL);

static struct attribute *tps23861_attributes[] = {
	&dev_attr_temp.attr,
	&dev_attr_vin.attr,
	&dev_attr_port_mode.attr,
	&dev_attr_port_power_on.attr,
	&dev_attr_port_power_off.attr,
	&dev_attr_port_info.attr,
	NULL,
};

static const struct attribute_group tps23861_attr_group = {
	.attrs = tps23861_attributes,
};

static const struct of_device_id tps23861_dt_id[] = {
	{ .compatible = "ti,tps23861"},
	{ }
};

static int tps23861_of_probe(struct device *dev,
		struct tps23861_platform_data *pdata) {
	int i;
	const char *pmode;
	struct device_node *np = dev->of_node;
	struct device_node *child;

	if (!of_match_device(tps23861_dt_id, dev))
		return -ENODEV;

	if ((!np) || !of_get_next_child(np, NULL))
		return -EINVAL;
	
	of_property_read_u32(np, "irq-gpio", &pdata->irq);
	i = 0;
	for_each_child_of_node(np, child) {
		of_property_read_u32(child, "enable", &pdata->pt_df[i].enable);

		if (0 == of_property_read_string(child, "mode", &pmode)) {
			if (!strncmp(pmode, "manual", 6))
				pdata->pt_df[i].mode = PORT_MODE_MANUAL;
			else if (!strncmp(pmode, "semiauto", 8))
				pdata->pt_df[i].mode = PORT_MODE_SEMIAUTO;
			else if (!strncmp(pmode, "auto", 4))
				pdata->pt_df[i].mode = PORT_MODE_AUTO;
			else
				pdata->pt_df[i].mode = PORT_MODE_OFF;
		}
		of_property_read_u32(child, "power", &pdata->pt_df[i].pwr);
		i++;
	}
	return 0;
}

static int tps23861_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{	
	int res;
	static struct tps23861_data *ddata;
	struct tps23861_platform_data *pdata;
	struct device *dev = &client->dev;

	ddata = devm_kzalloc(dev, sizeof(struct tps23861_data), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;
	res = sysfs_create_group(&dev->kobj, &tps23861_attr_group);
	if (res) {
		dev_err(dev, "failed to create sysfs attributes\n");
		devm_kfree(&client->dev, ddata);
		return res;	
	}
	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	dev_set_drvdata(dev, ddata);
	pdata = &ddata->pdata;
	res = tps23861_of_probe(dev, pdata);
	tps23861_apply_dt_defaults(dev);

	dev_info(dev, "TPS23861 ID 0x%x, silicon rev. %u, fw %u",
			tps23861_read_reg(ddata, DID_SIL_REV_REG) >> 5 & 0x7,
			tps23861_read_reg(ddata, DID_SIL_REV_REG) & 0x1F,
			tps23861_read_reg(ddata, FW_REV_REG) & 0x7);

	tps23861_irq_enable(ddata, SUPEN | STRTEN | IFEN | CLCEN | DISEN);
	res = request_threaded_irq(gpio_to_irq(pdata->irq), NULL,
			tps23861_irq_handler,
			IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
			"tps23861", ddata);
	if (res) {
		dev_err(dev, "failed to register interrupt\n");
		sysfs_remove_group(&client->dev.kobj, &tps23861_attr_group);
		devm_kfree(&client->dev, ddata);
		return res;
	}
	tps23861_irq_clear(ddata, SUPEN);
	tps23861_irq_handler(gpio_to_irq(pdata->irq), ddata);
	return 0;
}

static int tps23861_remove(struct i2c_client *client)
{
	struct tps23861_data *ddata = i2c_get_clientdata(client);
	free_irq(gpio_to_irq(ddata->pdata.irq), ddata);
	sysfs_remove_group(&client->dev.kobj, &tps23861_attr_group);
	devm_kfree(&client->dev, ddata);
	return 0;
}

static const struct i2c_device_id tps23861_id[] = {
	{ "tps23861", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tps23861_id);

static struct i2c_driver tps23861_driver = {
	.driver = {
		.name   = "tps23861",
		.owner  = THIS_MODULE,
		.of_match_table = tps23861_dt_id,
	},
	.probe          = tps23861_probe,
	.remove         = tps23861_remove,
	.id_table	= tps23861_id,
};

static int __init tps23861_init( void ) {
	return i2c_add_driver(&tps23861_driver);
}

static void __exit tps23861_exit( void ) {
	i2c_del_driver(&tps23861_driver);
}

module_init( tps23861_init );
module_exit( tps23861_exit );

MODULE_AUTHOR("Dmitry Alekseev <alexeev6@yahoo.com>");
MODULE_DESCRIPTION("TPS23861 Quad Port POE PSE Controller Driver");
MODULE_LICENSE("GPL v2");
