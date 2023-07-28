/*
 * Copyright (c) 2023 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/shell/shell.h>
// #include <zephyr/usb_c/usbc.h>
#include <zephyr/drivers/usb_c/usbc_ppc.h>

#include "nxp_nx20p348x_priv.h"

#define DT_DRV_COMPAT nxp_nx20p348x
LOG_MODULE_REGISTER(nxp_nx20p348x, CONFIG_USBC_PPC_LOG_LEVEL);

/************************************************************ Structs ************************************************************/

struct nx20p348x_cfg {
	const struct i2c_dt_spec bus;
	const struct gpio_dt_spec irq_gpio;
};

struct nx20p348x_data {
	const struct device *dev;
	struct gpio_callback irq_cb;
	struct k_work irq_work;
};

/************************************************************ Helpers ************************************************************/

static int read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	const struct nx20p348x_cfg *cfg = dev->config;
	struct nx20p348x_data *data = dev->data;
	int ret;

	ret = i2c_reg_read_byte(cfg->bus.bus, cfg->bus.addr, reg, value);
	if (ret < 0) {
		LOG_ERR("Error reading reg %02x: %d", reg, ret);
		return ret;
	}

	return 0;
}

static int write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct nx20p348x_cfg *cfg = dev->config;
	struct nx20p348x_data *data = dev->data;
	int ret;

	ret = i2c_reg_write_byte(cfg->bus.bus, cfg->bus.addr, reg, value);
	if (ret < 0) {
		LOG_ERR("Error writing reg %02x: %d", reg, ret);
		return ret;
	}

	return 0;
}

/************************************************************ API ************************************************************/

int nx20p348x_is_dead_battery_mode(const struct device *dev)
{
	uint8_t sts_reg;
	int ret;

	ret = read_reg(dev, NX20P348X_DEVICE_STATUS_REG, &sts_reg);
	if (ret < 0) {
		return ret;
	}

	// TODO: 3481
	return ((sts_reg & NX20P3483_DEVICE_MODE_MASK) == NX20P348X_MODE_DEAD_BATTERY);
}

int nx20p348x_exit_dead_battery_mode(const struct device *dev)
{
	uint8_t ctrl_reg;
	int ret;

	ret = read_reg(dev, NX20P348X_DEVICE_CONTROL_REG, &ctrl_reg);
	if (ret < 0) {
		return ret;
	}

	ctrl_reg |= NX20P348X_CTRL_DB_EXIT;
	ret = write_reg(dev, NX20P348X_DEVICE_CONTROL_REG, ctrl_reg);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int nx20p348x_is_sourcing_vbus(const struct device *dev)
{
	// const struct nx20p348x_cfg *cfg = dev->config;
	// struct nx20p348x_data *data = dev->data;
	uint8_t sts_reg;
	int ret;

	ret = read_reg(dev, NX20P348X_SWITCH_STATUS_REG, &sts_reg);
	if (ret < 0) {
		return ret;
	}

	// TODO: 3481 or not...
	return !!(sts_reg & (NX20P348X_SWITCH_STATUS_5VSRC | NX20P348X_SWITCH_STATUS_HVSRC));
}

static int nx20p348x_is_sinking_vbus(const struct device *dev)
{
	uint8_t sts_reg;
	int ret;

	ret = read_reg(dev, NX20P348X_SWITCH_STATUS_REG, &sts_reg);
	if (ret < 0) {
		return ret;
	}

	// TODO: 3481 or not...
	return !!(sts_reg & NX20P348X_SWITCH_STATUS_HVSNK);
}

static int nx20p348x_vbus_sink_enable(const struct device *dev, bool enable)
{
	// TODO: 3481 ->
	// const struct nx20p348x_cfg *cfg = dev->config;
	// struct nx20p348x_data *data = dev->data;
	// uint8_t ctrl_reg = (enable) ? NX20P3481_SWITCH_CONTROL_HVSNK : 0;
	// int ret;

	// ret = write_reg(dev, NX20P348X_SWITCH_CONTROL_REG, ctrl_reg);
	// if (ret < 0) {
	// 	return ret;
	// }

	// return 0;

	// 3483
	return -ENOSYS;
}

static int nx20p348x_vbus_source_enable(const struct device *dev, bool enable)
{
	// TODO: 3481 ->
	// const struct nx20p348x_cfg *cfg = dev->config;
	// struct nx20p348x_data *data = dev->data;
	// uint8_t ctrl_reg = (enable) ? NX20P3481_SWITCH_CONTROL_5VSRC : 0; // TODO: dts HV vs 5V
	// int ret;

	// ret = write_reg(dev, NX20P348X_SWITCH_CONTROL_REG, ctrl_reg);
	// if (ret < 0) {
	// 	return ret;
	// }

	// return 0;

	// 3483
	return -ENOSYS;
}

// static int nx20p348x_set_polarity(const struct device *dev, enum tc_cc_polarity polarity)
// {
// 	return -ENO;
// }

static int nx20p348x_set_vbus_source_current_limit(const struct device *dev, enum tc_rp_value rp)
{
	uint8_t thr_reg;
	int ret;

	ret = read_reg(dev, NX20P348X_5V_SRC_OCP_THRESHOLD_REG, &thr_reg);
	if (ret < 0) {
		return ret;
	}

	thr_reg &= ~NX20P348X_ILIM_MASK;

	switch (rp) {
	case TC_RP_USB:
		thr_reg |= NX20P348X_ILIM_1_000;
		break;
	case TC_RP_1A5:
		thr_reg |= NX20P348X_ILIM_1_600;
		break;
	case TC_RP_3A0:
		thr_reg |= NX20P348X_ILIM_3_200;
		break;
	default:
		return -EINVAL;
	}

	ret = write_reg(dev, NX20P348X_5V_SRC_OCP_THRESHOLD_REG, thr_reg);
	return ret;
}

static int nx20p348x_discharge_vbus(const struct device *dev, bool enable)
{
	uint8_t ctrl_reg;
	int ret;

	ret = read_reg(dev, NX20P348X_DEVICE_CONTROL_REG, &ctrl_reg);
	if (ret < 0) {
		return ret;
	}

	if (enable) {
		ctrl_reg |= NX20P348X_CTRL_VBUSDIS_EN;
	} else {
		ctrl_reg &= ~NX20P348X_CTRL_VBUSDIS_EN;
	}

	ret = write_reg(dev, NX20P348X_DEVICE_CONTROL_REG, ctrl_reg);

	return ret;
}

// static int nx20p348x_notify_device_connection(const struct device *dev, enum usbc_ppc_device_role role)
// {
// 	return -EIO;
// }

// static int nx20p348x_set_sbu(const struct device *dev, bool enable)
// {
// 	return -EIO;
// }

// static int nx20p348x_set_vconn(const struct device *dev, bool enable)
// {
// 	return -EIO;
// }

// static int nx20p348x_set_frs_enable(const struct device *dev, bool enable)
// {
// 	return -EIO;
// }

// static int nx20p348x_is_vbus_present(const struct device *dev)
// {
// 	return -EIO;
// }

static int nx20p348x_dump_regs(const struct device *dev)
{
	const struct nx20p348x_cfg *cfg = dev->config;
	uint8_t val;

	LOG_INF("PPC %s:%s registers:", cfg->bus.bus->name, dev->name);
	for(unsigned a = 0; a <= 0xB; a++) {
		i2c_reg_read_byte(cfg->bus.bus, cfg->bus.addr, a, &val);

		LOG_INF("- [%02x] = 0x%02x", a, val);
	}

	return 0;
}

static struct usbc_ppc_drv nx20p348x_driver_api = {
	.is_dead_battery_mode = nx20p348x_is_dead_battery_mode,
	.exit_dead_battery_mode = nx20p348x_exit_dead_battery_mode,
	.is_sourcing_vbus = nx20p348x_is_sourcing_vbus,
	.is_sinking_vbus = nx20p348x_is_sinking_vbus,
	.vbus_sink_enable = nx20p348x_vbus_sink_enable,
	.vbus_source_enable = nx20p348x_vbus_source_enable,
	// .set_polarity = nx20p348x_set_polarity,
	.set_vbus_source_current_limit = nx20p348x_set_vbus_source_current_limit,
	.discharge_vbus = nx20p348x_discharge_vbus,
	// .notify_device_connection = nx20p348x_notify_device_connection,
	// .set_sbu = nx20p348x_set_sbu,
	// .set_vconn = nx20p348x_set_vconn,
	// .set_frs_enable = nx20p348x_set_frs_enable,
	// .is_vbus_present = nx20p348x_is_vbus_present,
	.dump_regs = nx20p348x_dump_regs,
};

static void nx20p348x_irq_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct nx20p348x_data *data = CONTAINER_OF(cb, struct nx20p348x_data, irq_cb);

	LOG_INF("NX20P348X irq handler");

	k_work_submit(&data->irq_work);
}

static void nx20p348x_irq_worker(struct k_work *work)
{
	struct nx20p348x_data *data = CONTAINER_OF(work, struct nx20p348x_data, irq_work);
	uint8_t irq1, irq2;

	LOG_INF("NX20P348X irq worker");

	read_reg(data->dev, NX20P348X_INTERRUPT1_REG, &irq1);
	read_reg(data->dev, NX20P348X_INTERRUPT2_REG, &irq2);

	LOG_INF("Irq1: %02x, Irq2: %02x", irq1, irq2);
}

static int nx20p348x_dev_init(const struct device *dev)
{
	const struct nx20p348x_cfg *cfg = dev->config;
	struct nx20p348x_data *data = dev->data;
	int ret;

	LOG_INF("Initializing PPC");

	/* Initialize irq */
	ret = gpio_pin_configure(cfg->irq_gpio.port, cfg->irq_gpio.pin, GPIO_INPUT);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure(cfg->irq_gpio.port, cfg->irq_gpio.pin, GPIO_INT_EDGE_FALLING);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->irq_cb, nx20p348x_irq_handler, BIT(cfg->irq_gpio.pin));
	ret = gpio_add_callback(cfg->irq_gpio.port, &data->irq_cb);
	if (ret < 0) {
		return ret;
	}

	/* Initialize work_q */
	k_work_init(&data->irq_work, nx20p348x_irq_worker);

	// nx20p348x_vbus_source_enable(dev, 1);

	return 0;
}

#define NX20P348X_DRIVER_CFG_INIT(node)                                        \
	{                                                                      \
		.bus = I2C_DT_SPEC_GET(node),                                  \
		.irq_gpio = GPIO_DT_SPEC_GET(node, irq_gpios),                 \
	}

#define NX20P348X_DRIVER_DATA_INIT(node)                                                           \
	{                                                                                          \
		.dev = DEVICE_DT_GET(node), \
	}

#define NX20P348X_DRIVER_INIT(inst)                                                                \
	static struct nx20p348x_data drv_data_nx20p348x##inst = NX20P348X_DRIVER_DATA_INIT(DT_DRV_INST(inst));                                        \
	static struct nx20p348x_cfg drv_cfg_nx20p348x##inst = NX20P348X_DRIVER_CFG_INIT(DT_DRV_INST(inst)); \
	DEVICE_DT_INST_DEFINE(inst, &nx20p348x_dev_init, NULL, &drv_data_nx20p348x##inst,                \
			      &drv_cfg_nx20p348x##inst, POST_KERNEL, CONFIG_USBC_PPC_INIT_PRIORITY,       \
			      &nx20p348x_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NX20P348X_DRIVER_INIT)
