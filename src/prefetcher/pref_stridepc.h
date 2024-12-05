/* Copyright 2020 HPS/SAFARI Research Groups
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/***************************************************************************************
 * File         : pref_stridepc.h
 * Author       : HPS Research Group
 * Date         : 1/23/2005
 * Description  : Stride Prefetcher - Based on load's PC address
 ***************************************************************************************/
#ifndef __PREF_STRIDEPC_H__
#define __PREF_STRIDEPC_H__

#include "pref_common.h"
#include <stdbool.h>

typedef struct StridePC_Table_Entry_Struct {
  Flag trained;
  Flag valid;

  Addr last_addr;
  Addr load_addr;
  Addr start_index;
  Addr pref_last_index;
  int  stride;

  Counter train_num;
  Counter pref_sent;
  Counter last_access;  // for lru
} StridePC_Table_Entry;

typedef struct Pref_StridePC_Struct {
  HWP_Info*             hwp_info;
  StridePC_Table_Entry* stride_table;
  CacheLevel        type;
} Pref_StridePC;

typedef struct{
  Pref_StridePC* stridepc_hwp_core_ul1;
  Pref_StridePC* stridepc_hwp_core_umlc;
} stridepc_prefetchers;

/*************************************************************/
/* HWP Interface */
void pref_stridepc_init(HWP* hwp);

void pref_stridepc_ul1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC,
                            uns32 global_hist);
void pref_stridepc_ul1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC,
                           uns32 global_hist);
void pref_stridepc_umlc_miss(uns8 proc_id, Addr lineAddr, Addr loadPC,
                            uns32 global_hist);
void pref_stridepc_umlc_hit(uns8 proc_id, Addr lineAddr, Addr loadPC,
                           uns32 global_hist);


/*************************************************************/
/* Internal Function */
void init_stridepc(HWP* hwp, Pref_StridePC* stridepc_hwp_core);
void pref_stridepc_train(Pref_StridePC* stridepc_hwp, uns8 proc_id, Addr lineAddr, Addr loadPC,
                             Flag is_hit);
/*************************************************************/
/* Misc functions */

typedef struct Footprint_struct {
    bool accessed[64];  // Size of page devided by block size
                        // 4096/64
} Footprint;

typedef struct Aux_Entry_Struct {
  Footprint    footprint; // Bit vector for accessed blocks
  Addr      trigger_addr; // The address that triggered this entry
  Addr      pc;           // The PC of the trigger instruction
} Aux_Entry;

// This will be put into a dictionary with key as Page address


typedef struct Bingo_History_Table_Struct {
  // table hashed by PC + Offset and each entry is PC+Address, footprint
  Addr      pc_plus_address;
  Addr      pc_plus_offset;
  Aux_Entry entry; // Holds the original data from the Aux data  
} Bingo_History_Table; // this is one entry in the Bingo table will also be in a dictionary

typedef struct Bingo_Table_Line_Struct {
    Bingo_History_Table line[16];  // The entries
    int usage_order[16];           // Tracks the order of usage for LRU
    int current_size;              // Keeps track of the number of valid entries in the line
} Bingo_Table_Line;


#endif /*  __PREF_STRIDEPC_H__*/
