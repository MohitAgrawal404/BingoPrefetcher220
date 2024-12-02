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
 * File         : stream_pref.c
 * Author       : HPS Research Group
 * Date         : 10/24/2002
 * Description  :
 ***************************************************************************************/

#include "debug/debug_macros.h"
#include "debug/debug_print.h"
#include "globals/global_defs.h"
#include "globals/global_types.h"
#include "globals/global_vars.h"

#include "globals/assert.h"
#include "globals/utils.h"
#include "op.h"

#include "core.param.h"
#include "dcache_stage.h"
#include "debug/debug.param.h"
#include "general.param.h"
#include "libs/cache_lib.h"
#include "libs/hash_lib.h"
#include "libs/list_lib.h"
#include "memory/memory.h"
#include "memory/memory.param.h"
#include "prefetcher//bingo_pref.h"
#include "prefetcher//bingo_pref.param.h"
#include "prefetcher/l2l1pref.h"
#include "prefetcher/pref_common.h"
#include "prefetcher/bingo_pref.h"
#include "statistics.h"

/**************************************************************************************/
/* Macros */
#define DEBUG(args...) _DEBUG(DEBUG_PREF_Bingo, ##args)

/**************************************************************************************/
Hash_Table History_Table; 
Hash_Table Aux_Storage; 

// Method for detecting when a block is being accessed (will just be PC trigger)
//   

// Method for detecting when a block has been evicted or made invalid 
//   cache_invalidate    pref_ul1evict  pref_ul1evictOnPF


void pref_bingo_init(HWP* hwp){
  init_hash_table(&History_Table, "History Table", 32, sizeof(Bingo_Table_Line));
  init_hash_table(&Aux_Storage, "Auxiliary Storage", 64, sizeof(Aux_Entry));
}


void pref_bingo_ul1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist) {
  // On miss you will first check for if the address exists in the bingo, if not then
  //  add it to the Aux data and start recording 
  //  if already in the Aux then flip a bit in the bitmap (don't change anything else)
  //  continue
  Addr block_address = lineAddr >> 64;
  Addr pc_plus_offset = loadPC + block_address;
  Addr pc_plus_address = loadPC + lineAddr;
  Addr page_address = lineAddr >> 4096;


  Bingo_Table_Line line = hash_table_access(History_Table, pc_plus_offset);
  Bingo_History_Table hash_entry =  pref_bingo_find_event_to_fetch_addr(line, pc_plus_address);
  if (hash_entry == NULL){
    hash_entry =  pref_bingo_find_event_to_fetch(line, pc_plus_offset);
  }
  Aux_Entry aux_entry = hash_table_access(Aux_Storage, page_address);

  int block = (block_address - page_address) / 64;
  if (hash_entry){
    // Push the prefetch stuff
    pref_bingo_prefetch(hash_entry, proc_id, page_address);
    return;
  }
  else if (aux_entry){
    aux_entry -> footprint -> accessed[block] = TRUE;
  }
  else{
    Aux_Entry aux_entry_temp = malloc(sizeof(Aux_Entry));
    aux_entry_temp -> trigger_addr = lineAddr;
    aux_entry_temp -> pc = loadPC;
    aux_entry_temp -> footprint -> accessed[block] = TRUE;
    hash_table_access_replace(Aux_Storage, page_address, aux_entry_temp);
  }

}

// Finds the most recently used entry with a matching pc_plus_offset
Bingo_History_Table* pref_bingo_find_event_to_fetch(Bingo_Table_Line* table_line, Addr pc_plus_offset) {
    for (int i = 0; i < table_line->current_size; i++) {
        int index = table_line->usage_order[i];
        if (table_line->line[index].pc_plus_offset == pc_plus_offset) {
            return &table_line->line[index];
        }
    }
    return NULL;  // Not found
}

Bingo_History_Table* pref_bingo_find_event_to_fetch_addr(Bingo_Table_Line* table_line, Addr pc_plus_address) {
    for (int i = 0; i < table_line->current_size; i++) {
        int index = table_line->usage_order[i];
        if (table_line->line[index].pc_plus_address == pc_plus_address) {
            return &table_line->line[index];
        }
    }
    return NULL;  // Not found
}


void pref_bingo_prefetch(Bingo_History_Table History_Entry, uns8 proc_id, Addr page_address){
  Addr temp_line_addr = 0;
  for (int i = 0; i < 64; i ++){
    if (History_Entry -> entry -> footprint -> accessed[i] == TRUE) {
      temp_line_addr = page_address + (64 * i); 
      for (int i = 0; i < 64; i ++){
        temp_line_addr = temp_line_addr + i;
        pref_addto_ul1req_queue(proc_id, temp_line_addr, "Bingo");
      }
    }
  }
  return 
}


// Adds a new entry to the table, treats it as the most recently used, and pops the least recently used
void add_entry(Bingo_Table_Line* table_line, Bingo_History_Table new_entry) {
    int index_to_replace;

    if (table_line->current_size < 16) {
        // If there's space, add to the next available spot
        index_to_replace = table_line->current_size;
        table_line->current_size++;
    } else {
        // Replace the least recently used (the last in the usage_order array)
        index_to_replace = table_line->usage_order[15]; // Pop the LRU
    }
    
    // Replace the entry at the chosen index
    table_line->line[index_to_replace] = new_entry;

    // Shift the usage_order to the right to make room for the new most recently used entry
    for (int i = 15; i > 0; i--) {
        table_line->usage_order[i] = table_line->usage_order[i - 1];
    }
    
    // Place the new entry's index at the front (most recently used)
    table_line->usage_order[0] = index_to_replace;
}

// Marks an entry as recently used by address (i.e., pc_plus_address)
void mark_used_by_address(Bingo_Table_Line* table_line, Addr pc_plus_address) {
    int found_index = -1;
    // Find the entry by address
    for (int i = 0; i < table_line->current_size; i++) {
        if (table_line->line[i].pc_plus_address == pc_plus_address) {
            found_index = i;
            break;
        }
    }
    
    if (found_index == -1) return; // Entry not found

    // Move the found entry to the front of the usage_order array (most recently used)
    for (int i = 0; i < table_line->current_size; i++) {
        if (table_line->usage_order[i] == found_index) {
            // Shift entries to the right
            for (int j = i; j > 0; j--) {
                table_line->usage_order[j] = table_line->usage_order[j - 1];
            }
            table_line->usage_order[0] = found_index; // Place at the front
            break;
        }
    }
}



