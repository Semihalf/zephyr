/*
 * Copyright (c) 2023 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/usb_c/tcpci_priv.h>
#include <zephyr/usb_c/usbc.h>
#include <zephyr/usb_c/tcpci.h>
#include <zephyr/shell/shell.h>

#define DUMP_TCPC_REGS(node) ret |= tcpc_dump_std_reg(DEVICE_DT_GET(DT_PROP(node, tcpc)));

static int cmd_tcpci_dump(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;

	DT_FOREACH_STATUS_OKAY(usb_c_connector, DUMP_TCPC_REGS);

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tcpci_cmds,
			       SHELL_CMD(dump, NULL, "Dump TCPC registers", cmd_tcpci_dump),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(tcpci, &sub_tcpci_cmds, "TCPCI (USB-C PD) diagnostics", NULL);
