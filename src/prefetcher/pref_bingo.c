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
 * File         : pref_bingo.c
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
#include "debug/debug.param.h"
#include "general.param.h"
#include "libs/cache_lib.h"
#include "libs/hash_lib.h"
#include "libs/list_lib.h"
#include "memory/memory.param.h"
#include "prefetcher//pref_bingo.h"
#include "prefetcher//pref_bingo.param.h"
#include "prefetcher/pref.param.h"
#include "prefetcher/pref_common.h"
#include "statistics.h"

/**************************************************************************************/
/* Macros */
#define DEBUG(args...) _DEBUG(DEBUG_PREF_BINGO, ##args)

/**************************************************************************************/
Hash_Table History_Table; 
Hash_Table Aux_Storage; 
HWP_Info hwp_in;

// Method for detecting when a block is being accessed (will just be PC trigger)
//   

// Method for detecting when a block has been evicted or made invalid 
//   cache_invalidate    pref_ul1evict  pref_ul1evictOnPF


void pref_bingo_init(HWP* hwp) {
  printf("im hereeeebingo\n");
  //DEBUG("Your message, int: 0\n");
  printf("im hereeee\n");
  if(!PREF_BINGO_ON)
    return;
  hwp->hwp_info->enabled = TRUE;
  hwp_in = *(hwp->hwp_info);
  init_hash_table(&History_Table, "History Table", 32, sizeof(Bingo_Table_Line));
  init_hash_table(&Aux_Storage, "Auxiliary Storage", 64, sizeof(Aux_Entry));
  return;
}

void pref_bingo_ul1_hit(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist){
    // On miss you will first check for if the address exists in the bingo, if not then
  //  add it to the Aux data and start recording 
  //  if already in the Aux then flip a bit in the bitmap (don't change anything else)
  //  continue
  //printf("hit");
  Addr block_address = (lineAddr >> 6) << 6; // Clear the lower 6 bits 
  Addr pc_plus_offset = loadPC + block_address;
  Addr pc_plus_address = loadPC + lineAddr;
  Addr page_address = lineAddr >> 12;
  Addr page_offset = lineAddr & (4096 - 1);

  Bingo_Table_Line* line = hash_table_access(&History_Table, pc_plus_offset);
  Bingo_History_Table* hash_entry = NULL;
  hash_entry = pref_bingo_find_event_to_fetch_addr(line, pc_plus_address);
  if (hash_entry == NULL){
    hash_entry =  pref_bingo_find_event_to_fetch(line, pc_plus_offset);
    printf("Pointer address: %p\n", (void*)hash_entry);
  }
  int block_index = page_offset / 64;
  return;
  if (hash_entry == NULL){
    printf("hola\n");
    return;
    Aux_Entry* aux_entry = hash_table_access(&Aux_Storage, page_address);
    return;
    if (aux_entry){
      aux_entry->footprint.accessed[block_index] = TRUE;
       return;
    }
    else{
      Aux_Entry* aux_entry_temp = (Aux_Entry*)malloc(sizeof(Aux_Entry));
      aux_entry_temp->trigger_addr = lineAddr;
      aux_entry_temp->pc = loadPC;
      memset(aux_entry_temp->footprint.accessed, 0, sizeof(aux_entry_temp->footprint.accessed));
      aux_entry_temp->footprint.accessed[block_index] = TRUE;

      // Store the new auxiliary entry in the aux table
      hash_table_access_replace(&Aux_Storage, page_address, aux_entry_temp);
    }
  }

}

void pref_bingo_ul1_cache_evict(uns8 proc_id, Addr lineAddr) {
    //printf("evict");
    return;
    Addr page_address = lineAddr >> 12;

    // Access the auxiliary entry from the Aux_Storage table
    Aux_Entry* aux_entry = hash_table_access(&Aux_Storage, page_address);
    
    // Check if the auxiliary entry exists
    if (!aux_entry) {
        // If the auxiliary entry doesn't exist, we can't evict, so just return
        return;
    }

    Addr block_address = lineAddr >> 6;
    Addr pc_plus_offset = aux_entry->pc + block_address;
    Addr pc_plus_address = aux_entry->pc + lineAddr;

    // Access the corresponding history table entry, or allocate if it doesn't exist
    Bingo_History_Table* hist_entry = (Bingo_History_Table*)malloc(sizeof(Bingo_History_Table));
    
    // Populate the history table entry
    hist_entry->pc_plus_address = pc_plus_address;
    hist_entry->pc_plus_offset = pc_plus_offset;
    hist_entry->entry = *aux_entry;  // Copy the aux_entry into the history table entry

    Bingo_Table_Line* table_line = hash_table_access(&History_Table, pc_plus_offset);

    if (table_line == NULL){
      table_line = (Bingo_Table_Line*)malloc(sizeof(Bingo_Table_Line));
      table_line->current_size = 0;
    }

    add_entry(table_line, *hist_entry);


    // Replace the entry in the history table
    hash_table_access_replace(&History_Table, pc_plus_offset, &table_line);

    // Now free the aux_entry from Aux_Storage since we no longer need it
    hash_table_access_delete(&Aux_Storage, page_address);
}




void pref_bingo_ul1_miss(uns8 proc_id, Addr lineAddr, Addr loadPC, uns32 global_hist) {
  // On miss you will first check for if the address exists in the bingo, if not then
  //  add it to the Aux data and start recording 
  //  if already in the Aux then flip a bit in the bitmap (don't change anything else)
  //  continue
  
  return;
  Addr block_address = lineAddr >> 6; 
  Addr pc_plus_offset = loadPC + block_address;
  Addr pc_plus_address = loadPC + lineAddr;
  Addr page_address = lineAddr >> 12;


  Bingo_Table_Line* line = hash_table_access(&History_Table, pc_plus_offset);
  Bingo_History_Table* hash_entry = pref_bingo_find_event_to_fetch_addr(line, pc_plus_address);

  if (hash_entry == NULL){
    hash_entry =  pref_bingo_find_event_to_fetch(line, pc_plus_offset);
  }
  Aux_Entry* aux_entry = hash_table_access(&Aux_Storage, page_address);

  int block = (block_address & 0x3F);

  if (hash_entry){
    // Push the prefetch stuff
    pref_bingo_prefetch(*hash_entry, proc_id, page_address);
    mark_used_by_address(line, pc_plus_address);
    return;
  }
  else if (aux_entry){
    aux_entry->footprint.accessed[block] = TRUE;
  }
  else{
    Aux_Entry* aux_entry_temp = (Aux_Entry*)malloc(sizeof(Aux_Entry));
    aux_entry_temp->trigger_addr = lineAddr;
    aux_entry_temp->pc = loadPC;
    memset(aux_entry_temp->footprint.accessed, 0, sizeof(aux_entry_temp->footprint.accessed));
    aux_entry_temp->footprint.accessed[block] = TRUE;

    // Store the new auxiliary entry in the hash table
    hash_table_access_replace(&Aux_Storage, page_address, aux_entry_temp);
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
  for (int i = 0; i < 64; i++) {
    if (History_Entry.entry.footprint.accessed[i] == TRUE) {
        temp_line_addr = page_address + (64 * i);  // Reset here
        for (int x = 0; x < 64; x++) {
            Addr addr_to_prefetch = temp_line_addr + x;  // Use a new variable to avoid overwriting
            //uns8 bingo[] = {'b', 'i', 'n', 'g', 'o', '\0'};
            pref_addto_ul1req_queue(proc_id, addr_to_prefetch, hwp_in.id);
        }
    }
  }
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



