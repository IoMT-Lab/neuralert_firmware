/**
 ****************************************************************************************
 *
 * @file command_mem.h
 *
 * @brief Header file for Console Commands to handle and to test memory
 *
 * Copyright (c) 2016-2022 Renesas Electronics. All rights reserved.
 *
 * This software ("Software") is owned by Renesas Electronics.
 *
 * By using this Software you agree that Renesas Electronics retains all
 * intellectual property and proprietary rights in and to this Software and any
 * use, reproduction, disclosure or distribution of the Software without express
 * written permission or a license agreement from Renesas Electronics is
 * strictly prohibited. This Software is solely for use on or in conjunction
 * with Renesas Electronics products.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE
 * SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. EXCEPT AS OTHERWISE
 * PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * RENESAS ELECTRONICS BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL,
 * OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
 * USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 ****************************************************************************************
 */
 
#ifndef	__COMMAND_MEM_H__
#define	__COMMAND_MEM_H__

#include "command.h"

//-----------------------------------------------------------------------
// Command MEM-List
//-----------------------------------------------------------------------

extern const COMMAND_TREE_TYPE	cmd_mem_list[] ;

//-----------------------------------------------------------------------
// Command Functions
//-----------------------------------------------------------------------

extern void cmd_mem_read(int argc, char *argv[]);
extern void cmd_mem_write(int argc, char *argv[]);
extern void cmd_mem_wread(int argc, char *argv[]);
extern void cmd_mem_wwrite(int argc, char *argv[]);
extern void cmd_mem_lread(int argc, char *argv[]);
extern void cmd_mem_lwrite(int argc, char *argv[]);
extern void cmd_mem_memcpy(int argc, char *argv[]);

extern void cmd_nor_func(int argc, char *argv[]);
extern void cmd_sflash_func(int argc, char *argv[]);
extern void cmd_sfcopy_func(int argc, char *argv[]);
extern void cmd_setsfl_func(int argc, char *argv[]);
extern void cmd_whdis_func(int argc, char *argv[]);

#endif /*__COMMAND_MEM_H__*/

/* EOF */
