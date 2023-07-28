/*
 * Copyright (c) 2023 The Chromium OS Authors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/usb_c/usbc_ppc.h>

#define DT_DRV_COMPAT nxp_nx20p348x

// #define PPC_DUMP(inst) ppc_dump_regs(DEVICE_DT_GET(DT_DRV_INST(inst)));

#define CALL_IF_HAS_PPC(usb_node, func) COND_CODE_1(DT_NODE_HAS_PROP(usb_node, ppc), (func(DEVICE_DT_GET(DT_PHANDLE_BY_IDX(usb_node, ppc, 0)));), ())
// #define CALL_IF_HAS_PPC(usb_node, func) func(DEVICE_DT_GET()

#define X(node) ppc_dump_regs(DEVICE_DT_GET(node))

static int cmd_ppc_dump(const struct shell *sh, size_t argc, char **argv)
{
	//usb_c_connector
	DT_FOREACH_STATUS_OKAY_VARGS(usb_c_connector, CALL_IF_HAS_PPC, ppc_dump_regs);
	// DT_FOREACH_STATUS_OKAY(nxp_nx20p348x, X);
	return 0;
}

static void print_status(const struct device *dev)
{
	printk("PPC %s:\n", dev->name);
	printk("  Dead battery:    %d\n", ppc_is_dead_battery_mode(dev));
	printk("  Is sourcing:     %d\n", ppc_is_sourcing_vbus(dev));
	printk("  Is sinking:      %d\n", ppc_is_sinking_vbus(dev));
	printk("  Is vbus present: %d\n", ppc_is_vbus_present(dev));
}

#define Y(node) print_status(DEVICE_DT_GET(node))

static int cmd_ppc_status(const struct shell *sh, size_t argc, char **argv)
{
	DT_FOREACH_STATUS_OKAY_VARGS(usb_c_connector, CALL_IF_HAS_PPC, print_status);
	// DT_FOREACH_STATUS_OKAY(nxp_nx20p348x, Y);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ppc_cmds,
			       SHELL_CMD(dump, NULL, "Dump PPC registers", cmd_ppc_dump),
			       SHELL_CMD(status, NULL, "Write PPC status", cmd_ppc_status),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ppc, &sub_ppc_cmds, "PPC (USB-C PD) diagnostics", NULL);
