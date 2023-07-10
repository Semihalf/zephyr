/*
 * Copyright (c) 2023 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_USBC_TCPCI_PRIV_H_
#define ZEPHYR_INCLUDE_DRIVERS_USBC_TCPCI_PRIV_H_

#include <stdint.h>
#include <zephyr/drivers/i2c.h>

struct tcpci_reg_dump_map {
	uint8_t addr;
	const char *name;
	uint8_t size;
};

#define TCPCI_STD_REGS_SIZE 37
extern const struct tcpci_reg_dump_map tcpci_std_regs[TCPCI_STD_REGS_SIZE];

int tcpci_read_reg8(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *value);
int tcpci_write_reg8(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t value);
int tcpci_update_reg8(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t mask, uint8_t value);
int tcpci_read_reg16(const struct i2c_dt_spec *bus, uint8_t reg, uint16_t *value);
int tcpci_write_reg16(const struct i2c_dt_spec *bus, uint8_t reg, uint16_t value);
enum tcpc_alert tcpci_alert_reg_to_enum(uint16_t reg);

#endif /* ZEPHYR_INCLUDE_DRIVERS_USBC_TCPCI_PRIV_H_ */
