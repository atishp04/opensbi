#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2019 SiFive Corporation.
#
# Authors:
#   Zong Li <zong.li@sifive.com>
#

# Blobs to build
FW_TEXT_START=0x80000000
FW_DYNAMIC=y
FW_JUMP=y
FW_JUMP_ADDR=0x80200000
FW_JUMP_FDT_ADDR=0x88000000
FW_PAYLOAD=y
FW_PAYLOAD_OFFSET=0x200000
FW_PAYLOAD_FDT_ADDR=0x88000000
FW_PAYLOAD_FDT=hawksbill.dtb
