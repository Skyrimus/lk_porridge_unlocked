/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <reg.h>
#include <debug.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <platform.h>
#include <platform/mt_typedefs.h>
#include <platform/boot_mode.h>
#include <platform/mt_reg_base.h>
#include <platform/env.h>
#include <libfdt.h>

extern BOOT_ARGUMENT *g_boot_arg;
static char dram_dummy_read_str[][40]={"reserve-memory-dram_r0_dummy_read", "reserve-memory-dram_r1_dummy_read"};

int target_fdt_dram_dummy_read(void *fdt, unsigned int rank_num)
{
    u32 i;
    int nodeoffset, parentoffset, ret = 0;
    dt_dram_info rsv_mem_reg_property[2];
    dt_size_info rsv_mem_size_property[2];
    mblock_info_t *mblock_info = &g_boot_arg->mblock_info;

    parentoffset = fdt_path_offset(fdt, "/reserved-memory");
    if (parentoffset < 0) {
        dprintf(CRITICAL, "Warning: can't find reserved-memory node in device tree\n");
        ret = -1;
    }

    for (i = 0; i < rank_num; i++) {
        rsv_mem_reg_property[i].start_hi = cpu_to_fdt32((mblock_info->mblock[i].start)>>32);
        rsv_mem_reg_property[i].start_lo = cpu_to_fdt32(mblock_info->mblock[i].start);
        rsv_mem_reg_property[i].size_hi = cpu_to_fdt32((mblock_info->mblock[i].size)>>32);
        rsv_mem_reg_property[i].size_lo = cpu_to_fdt32(mblock_info->mblock[i].size);
        rsv_mem_size_property[i].size_hi = cpu_to_fdt32(0);
        rsv_mem_size_property[i].size_lo = cpu_to_fdt32(0x00001000);

        nodeoffset = fdt_add_subnode(fdt, parentoffset, dram_dummy_read_str[i]);
        if (nodeoffset < 0) {
	    ret = -1;
            dprintf(CRITICAL, "Warning: can't find reserved dram rank%d dummy read node in device tree\n", i);
        }

        ret = fdt_setprop_string(fdt, nodeoffset, "compatible", dram_dummy_read_str[i]);
        if (ret) {
            ret = -1;
            dprintf(CRITICAL, "Warning: can't add compatible  in device tree\n");
        }

        ret = fdt_setprop(fdt, nodeoffset, "size", &rsv_mem_size_property[i], sizeof(dt_size_info));
        ret |= fdt_setprop(fdt, nodeoffset, "alignment", &rsv_mem_size_property[i], sizeof(dt_size_info));
        dprintf(INFO," rsv mem rsv_mem_reg_property[%d].start_hi = 0x%08X\n", i, rsv_mem_reg_property[i].start_hi);
        dprintf(INFO," rsv mem rsv_mem_reg_property[%d].start_lo = 0x%08X\n", i, rsv_mem_reg_property[i].start_lo);
        dprintf(INFO," rsv mem rsv_mem_reg_property[%d].size_hi = 0x%08X\n", i, rsv_mem_reg_property[i].size_hi);
        dprintf(INFO," rsv mem rsv_mem_reg_property[%d].size_lo = 0x%08X\n", i, rsv_mem_reg_property[i].size_lo);
        ret |= fdt_setprop(fdt, nodeoffset, "alloc-ranges", &rsv_mem_reg_property[i], sizeof(dt_dram_info));
        if (ret) {
            ret = -1;
            dprintf(CRITICAL, "Warning: can't setprop size, alignment and alloc-ranges in device tree\n");
        }
    }

    return ret;
}

