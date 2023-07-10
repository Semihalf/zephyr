/*
 * Copyright (c) 2023 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/usb_c/tcpci_priv.h>
#include <zephyr/usb_c/usbc.h>
#include <zephyr/usb_c/tcpci.h>
#include <zephyr/shell/shell.h>

#define TCPCI_DUMP_NODE(node) ret |= tcpc_dump_std_reg(DEVICE_DT_GET(DT_PROP(node, tcpc)));

#define TCPCI_VBUS_NODE(node)                                                                      \
	{                                                                                          \
		int val;                                                                           \
		ret |= usbc_vbus_measure(DEVICE_DT_GET(DT_PROP(node, vbus)), &val);                \
		shell_print(sh, "%s vbus: %d mV", DEVICE_DT_GET(node)->name, val);                 \
	}

static int cmd_tcpci_dump(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;

	DT_FOREACH_STATUS_OKAY(usb_c_connector, TCPCI_DUMP_NODE);

	return ret;
}

static int cmd_tcpci_vbus(const struct shell *sh, size_t argc, char **argv)
{
	int ret = 0;

	DT_FOREACH_STATUS_OKAY(usb_c_connector, TCPCI_VBUS_NODE);

	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tcpci_cmds,
			       SHELL_CMD(dump, NULL, "Dump TCPC registers", cmd_tcpci_dump),
			       SHELL_CMD(vbus, NULL, "Display all vbus voltages", cmd_tcpci_vbus),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(tcpci, &sub_tcpci_cmds, "TCPCI (USB-C PD) diagnostics", NULL);
