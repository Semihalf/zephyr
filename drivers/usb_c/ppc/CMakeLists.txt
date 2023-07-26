# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_USBC_PPC_SHELL ppc_shell.c)
zephyr_library_sources_ifdef(CONFIG_USBC_PPC_NX20P348X nxp_nx20p348x.c)
