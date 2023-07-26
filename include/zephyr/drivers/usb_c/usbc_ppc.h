/*
 * Copyright 2023 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USBC_USBC_PPC_H_
#define ZEPHYR_INCLUDE_DRIVERS_USBC_USBC_PPC_H_

/**
 * @brief USB Type-C Port Controller API
 * @defgroup usb_type_c_port_controller_api USB Type-C Port Controller API
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <errno.h>

#include "usbc_tcpc.h"

#ifdef __cplusplus
extern "C" {
#endif

enum usbc_ppc_device_role {
	USBC_PPC_DEV_DISCONNECTED = 0,
	USBC_PPC_DEV_SNK = 1,
	USBC_PPC_DEV_SRC = 2,
};

enum usbc_ppc_event {
	USBC_PPC_EVENT_DEAD_BATTERY = 0, // exit from dead battery failed

	USBC_PPC_EVENT_SRC_OVERVOLTAGE,
	USBC_PPC_EVENT_SRC_REVERSE_CURRENT,
	USBC_PPC_EVENT_SRC_OVERCURRENT,
	USBC_PPC_EVENT_SRC_SHORT,

	USBC_PPC_EVENT_OVER_TEMPERATURE,
	USBC_PPC_EVENT_BOTH_SNKSRC_ENABLED, // both sink and source enabled

	USBC_PPC_EVENT_SNK_REVERSE_CURRENT,
	USBC_PPC_EVENT_SNK_SHORT,
	USBC_PPC_EVENT_SNK_OVERVOLTAGE,
};

enum

__subsystem struct usbc_ppc_drv {
	int (*is_dead_battery_mode)(const struct device *dev);
	int (*exit_dead_battery_mode)(const struct device *dev);
	int (*is_sourcing_vbus)(const struct device *dev);
	int (*is_sinking_vbus)(const struct device *dev);
	int (*vbus_sink_enable)(const struct device *dev, bool enable);
	int (*vbus_source_enable)(const struct device *dev, bool enable);
	int (*set_polarity)(const struct device *dev, enum tc_cc_polarity polarity);
	int (*set_vbus_source_current_limit)(const struct device *dev, enum tc_rp_value rp);
	int (*discharge_vbus)(const struct device *dev, bool enable);
	int (*notify_device_connection)(const struct device *dev, enum usbc_ppc_device_role role);
	int (*set_sbu)(const struct device *dev, bool enable);
	int (*set_vconn)(const struct device *dev, bool enable);
	int (*set_frs_enable)(const struct device *dev, bool enable);
	int (*is_vbus_present)(const struct device *dev);
	int (*dump_regs)(const struct device *dev);
};

/*
 * API functions
 */

/* < 0 - error
 * 0 - no DB
 * 1 - dead battery
 */
static inline int ppc_is_dead_battery_mode(const struct device *dev)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->is_dead_battery_mode) {
		return -ENOSYS;
	}

	return api->is_dead_battery_mode(dev);
}

/*
 * 0 - means success sending request, need to check if still in dead battery in case of error
 */
static inline int ppc_exit_dead_battery_mode(const struct device *dev)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->exit_dead_battery_mode) {
		return -ENOSYS;
	}

	return api->exit_dead_battery_mode(dev);
}

static inline int ppc_is_sourcing_vbus(const struct device *dev)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->is_sourcing_vbus) {
		return -ENOSYS;
	}

	return api->is_sourcing_vbus(dev);
}

static inline int ppc_is_sinking_vbus(const struct device *dev)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->is_sinking_vbus) {
		return -ENOSYS;
	}

	return api->is_sinking_vbus(dev);
}

static inline int ppc_vbus_sink_enable(const struct device *dev, bool enable)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->vbus_sink_enable) {
		return -ENOSYS;
	}

	return api->vbus_sink_enable(dev, enable);
}

static inline int ppc_vbus_source_enable(const struct device *dev, bool enable)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->vbus_source_enable) {
		return -ENOSYS;
	}

	return api->vbus_source_enable(dev, enable);
}

static inline int ppc_set_polarity(const struct device *dev, enum tc_cc_polarity polarity)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->set_polarity) {
		return -ENOSYS;
	}

	return api->set_polarity(dev, polarity);
}

static inline int ppc_set_vbus_source_current_limit(const struct device *dev, enum tc_rp_value rp)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->set_vbus_source_current_limit) {
		return -ENOSYS;
	}

	return api->set_vbus_source_current_limit(dev, rp);
}

static inline int ppc_discharge_vbus(const struct device *dev, bool enable)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->discharge_vbus) {
		return -ENOSYS;
	}

	return api->discharge_vbus(dev, enable);
}

static inline int ppc_notify_device_connection(const struct device *dev, enum ppc_device_role role)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->notify_device_connection) {
		return -ENOSYS;
	}

	return api->notify_device_connection(dev, role);
}

static inline int ppc_set_sbu(const struct device *dev, bool enable)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->set_sbu) {
		return -ENOSYS;
	}

	return api->set_sbu(dev, enable);
}

static inline int ppc_set_vconn(const struct device *dev, bool enable)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->set_vconn) {
		return -ENOSYS;
	}

	return api->set_vconn(dev, enable);
}

static inline int ppc_set_frs_enable(const struct device *dev, bool enable)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->set_frs_enable) {
		return -ENOSYS;
	}

	return api->set_frs_enable(dev, enable);
}

static inline int ppc_is_vbus_present(const struct device *dev)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->is_vbus_present) {
		return -ENOSYS;
	}

	return api->is_vbus_present(dev);
}

static inline int ppc_dump_regs(const struct device *dev)
{
	const struct usbc_ppc_drv *api = (const struct usbc_ppc_drv *)dev->api;

	if (!api->dump_regs) {
		return -ENOSYS;
	}

	return api->dump_regs(dev);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_USBC_USBC_PPC_H_ */
