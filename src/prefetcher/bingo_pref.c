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
#include "prefetcher//pref_stream.h"
#include "prefetcher//stream.param.h"
#include "prefetcher/l2l1pref.h"
#include "prefetcher/pref_common.h"
#include "prefetcher/stream_pref.h"
#include "statistics.h"

/**************************************************************************************/
/* Macros */
#define DEBUG(proc_id, args...) _DEBUG(proc_id, DEBUG_STREAM, ##args)

/**************************************************************************************/
/* Global Variables */

extern Dcache_Stage* dc;

/***************************************************************************************/
/* Local Prototypes */

static inline void addto_train_stream_filter(Addr line_index);
static inline void remove_redundant_stream(int hit_index);

/**************************************************************************************/
/* stream prefetcher  */
/* prefetch is initiated by dcache miss but the request fills the l1 cache
 * (second level) cache   */
/* each stream has the starting pointer and the ending pointer and those
 * pointers tell whether the dl0 miss is within the boundary (buffer) */
/* stream buffer fetches one by one (end point requests the fetch address  */
/* stream buffer just holds the stream boundary not the data itself and data is
 * stored in the second level cache  */
/* At the beginning we will wait until we see 2 miss addresses */
/* using 2 miss addresses we decide the direction of the stream ( upward or
 * downward) and in the begining fill the half of buffer ( so many request
 * !!!)*/
/* Reference : IBM POWER 4 White paper */

#define DELTA_HISTORY_SIZE 16
#define PATTERN_SIZE 4
#define MAX_PATTERN_ENTRIES 32

typedef struct Bingo_History_Entry {
  Addr last_address;
  int delta_history[DELTA_HISTORY_SIZE]; // Stores deltas for each stream
  int delta_index; // Current index in the delta history array
} Bingo_History_Entry;

typedef struct Bingo_Pattern_Entry {
  int pattern[PATTERN_SIZE]; // Stores a sequence of deltas
  int next_delta; // The predicted delta after the pattern
  int occurrence_count; // Count of how often this pattern has been seen
} Bingo_Pattern_Entry;

Bingo_History_Entry* delta_history_table; // Table to store delta history for each stream
Bingo_Pattern_Entry pattern_table[MAX_PATTERN_ENTRIES]; // Table to store detected patterns


// Stream_HWP* stream_hwp;
// Addr*       train_filter;
// Addr*       train_l2hit_filter;
// static int  train_filter_no;
// static int  stream_pref_req_no;  // L1 prefetcher
// static int  stream_pref_send_no;

// static int l2hit_stream_pref_req_no;  // dcache prefetcher
// static int l2hit_stream_pref_send_no;
// static int train_l2hit_filter_no;

// static int l2hit_l2access_req_no;
// static int l2hit_l2access_send_no;

void init_stream_HWP(void) {
  stream_hwp = (Stream_HWP*)malloc(sizeof(Stream_HWP));
  
  // Initialize the delta history table
  delta_history_table = (Bingo_History_Entry*)calloc(STREAM_BUFFER_N, sizeof(Bingo_History_Entry));

  // Initialize pattern table
  for (int i = 0; i < MAX_PATTERN_ENTRIES; i++) {
    memset(&pattern_table[i], 0, sizeof(Bingo_Pattern_Entry));
  }
  
  // need to keep initializing
}

void stream_dl0_miss(Addr line_addr) {
  int hit_index = -1;
  Addr line_index = line_addr >> LOG2(DCACHE_LINE_SIZE);
  uns proc_id = get_proc_id_from_cmp_addr(line_addr);
  
  // Calculate delta (difference between current address and last address)
  Bingo_History_Entry* history_entry = &delta_history_table[proc_id];
  int delta = (line_addr - history_entry->last_address) >> LOG2(DCACHE_LINE_SIZE);

  // Update delta history
  history_entry->delta_history[history_entry->delta_index % DELTA_HISTORY_SIZE] = delta;
  history_entry->delta_index++;

  // Update last address
  history_entry->last_address = line_addr;

  // Call the function to detect patterns
  bingo_pattern_detection(proc_id);

  // Continue with the original stream detection logic...
}

void bingo_pattern_detection(uns proc_id) {
  Bingo_History_Entry* history_entry = &delta_history_table[proc_id];
  
  // Extract the last PATTERN_SIZE deltas from the delta history
  int recent_pattern[PATTERN_SIZE];
  for (int i = 0; i < PATTERN_SIZE; i++) {
    recent_pattern[i] = history_entry->delta_history[(history_entry->delta_index - 1 - i) % DELTA_HISTORY_SIZE];
  }

  // Search the pattern table for a match
  for (int i = 0; i < MAX_PATTERN_ENTRIES; i++) {
    if (memcmp(recent_pattern, pattern_table[i].pattern, sizeof(recent_pattern)) == 0) {
      // Pattern found, use it to prefetch the next delta
      bingo_prefetch(history_entry->last_address, pattern_table[i].next_delta);
      return;
    }
  }

  // If no pattern was found, add the recent pattern to the table
  for (int i = 0; i < MAX_PATTERN_ENTRIES; i++) {
    if (pattern_table[i].occurrence_count == 0) {
      memcpy(pattern_table[i].pattern, recent_pattern, sizeof(recent_pattern));
      pattern_table[i].next_delta = recent_pattern[0]; // Next delta can be the most recent one
      pattern_table[i].occurrence_count++;
      break;
    }
  }
}

void bingo_prefetch(Addr line_addr, int predicted_delta) {
  Addr prefetch_addr = line_addr + (predicted_delta << LOG2(DCACHE_LINE_SIZE));
  
  // Check if the prefetch address is valid and within bounds
  if (is_valid_address(prefetch_addr)) {
    Pref_Mem_Req new_req = {0};
    new_req.line_addr = prefetch_addr;
    new_req.line_index = prefetch_addr >> LOG2(DCACHE_LINE_SIZE);
    new_req.valid = TRUE;

    // Add the prefetch request to the queue
    if (!stream_hwp->pref_req_queue[stream_pref_req_no % PREF_REQ_Q_SIZE].valid) {
      stream_hwp->pref_req_queue[stream_pref_req_no++ % PREF_REQ_Q_SIZE] = new_req;
    }
  }
}


// void stream_dl0_miss(Addr line_addr) /* line_addr: the first address of the
//                                         cache block */{// search the stream buffer
//   int          hit_index = -1;
//   int          ii;
//   int          dis;
//   Pref_Mem_Req new_req    = {0};
//   Addr         line_index = line_addr >> LOG2(DCACHE_LINE_SIZE);
//   uns          proc_id    = get_proc_id_from_cmp_addr(line_addr);
//   /* training filter */

//   DEBUG(0, "[DL0MISS:0x%s]ma:0x%7s mi:0x%7s\n", "L1", hexstr64(line_addr),
//         hexstr64(line_index));

//   if(!train_stream_filter(line_index)) {
//     /* search for stream buffer */
//     hit_index = train_create_stream_buffer(
//       proc_id, line_index, TRUE, STREAM_CREATE_ON_DC_MISS);  // so we create on
//                                                              // dcache misses
//                                                              // also? -
//                                                              // confusing...

//     if(hit_index ==
//        -1) /* we do not have a trained buffer, nor did we create it */
//       return;

//     addto_train_stream_filter(line_index);

//     if(stream_hwp->stream[hit_index].trained) {
//       stream_hwp->stream[hit_index].lru = cycle_count;  // update lru
//       STAT_EVENT(proc_id, HIT_TRAIN_STREAM);
//       /* hit the stream_buffer, request the prefetch */


//       for(ii = 0; ii < STREAM_PREFETCH_N; ii++) {
//         if((stream_hwp->stream[hit_index].sp == line_index) &&
//            (stream_hwp->stream[hit_index].buffer_full)) {  // when is
//                                                            // buffer_full set to
//                                                            // FALSE except for
//                                                            // buffer creation?
//           // stream prefetch is requesting  enough far ahead
//           // stop prefetch and wait until miss address is within the buffer area
//           return;
//         }

//         new_req.line_index = stream_hwp->stream[hit_index].ep +
//                              stream_hwp->stream[hit_index].dir;
//         new_req.line_addr = (new_req.line_index) << LOG2(DCACHE_LINE_SIZE);
//         // check whether prefetch addr escaped the original demand's address
//         // space
//         if(proc_id != get_proc_id_from_cmp_addr(new_req.line_addr))
//           return;
//         new_req.valid = TRUE;

//         if(stream_hwp->pref_req_queue[stream_pref_req_no % PREF_REQ_Q_SIZE]
//              .valid) {
//           DEBUG(proc_id, "[PREF_QUEU] overlap!!\n");
//           STAT_EVENT(proc_id, PREF_REQ_QUE_FULL);
//           if(STREAM_STALL_ON_QUEUE_FULL)
//             return;
//         }


//         stream_hwp->pref_req_queue[stream_pref_req_no++ % PREF_REQ_Q_SIZE] =
//           new_req;

//         stream_hwp->stream[hit_index].ep = stream_hwp->stream[hit_index].ep +
//                                            stream_hwp->stream[hit_index].dir;
//         dis = stream_hwp->stream[hit_index].ep -
//               stream_hwp->stream[hit_index].sp;
//         if(((stream_hwp->stream[hit_index].dir == 1) &&
//             (dis > STREAM_LENGTH)) ||
//            ((stream_hwp->stream[hit_index].dir == -1) &&
//             (dis < -STREAM_LENGTH))) {
//           stream_hwp->stream[hit_index].buffer_full = TRUE;
//           stream_hwp->stream[hit_index].sp = stream_hwp->stream[hit_index].sp +
//                                              stream_hwp->stream[hit_index].dir;
//         }
//         STAT_EVENT(proc_id, STREAM_BUFFER_REQ);

//         if(REMOVE_REDUNDANT_STREAM)
//           remove_redundant_stream(hit_index);

//         DEBUG(proc_id,
//               "[InQ:0x%s]ma:0x%7s mi:0x%7s d:%2d ri:0x%7s, ra:0x%7s b:%2d "
//               "sp:0x%7s ep:0x%7s send_no:%d req_no:%d\n",
//               "L1", hexstr64(line_addr), hexstr64(line_index),
//               stream_hwp->stream[hit_index].dir, hexstr64(new_req.line_index),
//               hexstr64(new_req.line_addr), hit_index,
//               hexstr64(stream_hwp->stream[hit_index].sp),
//               hexstr64(stream_hwp->stream[hit_index].ep), stream_pref_send_no,
//               stream_pref_req_no);
//       }
//     } else
//       STAT_EVENT(proc_id, MISS_TRAIN_STREAM);
//   }
// }


// void stream_dl0_hit_train(Addr line_addr) {
//   Addr         line_index = line_addr >> LOG2(DCACHE_LINE_SIZE);
//   int          ii;
//   int          dis;
//   Pref_Mem_Req new_req = {0};
//   uns          proc_id = get_proc_id_from_cmp_addr(line_addr);

//   if(!train_stream_filter(line_index)) {
//     int hit_index = train_create_stream_buffer(proc_id, line_index, TRUE, TRUE);
//     if(hit_index == -1)
//       return;

//     addto_train_stream_filter(line_index);

//     if(stream_hwp->stream[hit_index].trained) {
//       stream_hwp->stream[hit_index].lru = cycle_count;  // update lru
//       STAT_EVENT(proc_id, HIT_TRAIN_STREAM);
//       /* hit the stream_buffer, request the prefetch */

//       for(ii = 0; ii < STREAM_PREFETCH_N; ii++) {
//         if((stream_hwp->stream[hit_index].sp == line_index) &&
//            (stream_hwp->stream[hit_index].buffer_full)) {  // when is
//                                                            // buffer_full set to
//                                                            // FALSE except for
//                                                            // buffer creation?
//           // stream prefetch is requesting  enough far ahead
//           // stop prefetch and wait until miss address is within the buffer area
//           return;
//         }

//         new_req.line_index = stream_hwp->stream[hit_index].ep +
//                              stream_hwp->stream[hit_index].dir;
//         new_req.line_addr = (new_req.line_index) << LOG2(DCACHE_LINE_SIZE);
//         // check whether prefetch addr escaped the original demand's address
//         // space
//         if(proc_id != get_proc_id_from_cmp_addr(new_req.line_addr))
//           return;
//         new_req.valid = TRUE;

//         if(stream_hwp->pref_req_queue[stream_pref_req_no % PREF_REQ_Q_SIZE]
//              .valid) {
//           DEBUG(proc_id, "[PREF_QUEU] overlap!!\n");
//           STAT_EVENT(proc_id, PREF_REQ_QUE_FULL);
//           if(STREAM_STALL_ON_QUEUE_FULL)
//             return;
//         }


//         stream_hwp->pref_req_queue[stream_pref_req_no++ % PREF_REQ_Q_SIZE] =
//           new_req;

//         stream_hwp->stream[hit_index].ep = stream_hwp->stream[hit_index].ep +
//                                            stream_hwp->stream[hit_index].dir;
//         dis = stream_hwp->stream[hit_index].ep -
//               stream_hwp->stream[hit_index].sp;
//         if(((stream_hwp->stream[hit_index].dir == 1) &&
//             (dis > STREAM_LENGTH)) ||
//            ((stream_hwp->stream[hit_index].dir == -1) &&
//             (dis < -STREAM_LENGTH))) {
//           stream_hwp->stream[hit_index].buffer_full = TRUE;
//           stream_hwp->stream[hit_index].sp = stream_hwp->stream[hit_index].sp +
//                                              stream_hwp->stream[hit_index].dir;
//         }
//         STAT_EVENT(proc_id, STREAM_BUFFER_REQ);

//         if(REMOVE_REDUNDANT_STREAM)
//           remove_redundant_stream(hit_index);

//         DEBUG(proc_id,
//               "[InQ:0x%s]ma:0x%7s mi:0x%7s d:%2d ri:0x%7s, ra:0x%7s b:%2d "
//               "sp:0x%7s ep:0x%7s send_no:%d req_no:%d\n",
//               "L1", hexstr64(line_addr), hexstr64(line_index),
//               stream_hwp->stream[hit_index].dir, hexstr64(new_req.line_index),
//               hexstr64(new_req.line_addr), hit_index,
//               hexstr64(stream_hwp->stream[hit_index].sp),
//               hexstr64(stream_hwp->stream[hit_index].ep), stream_pref_send_no,
//               stream_pref_req_no);
//       }
//     } else
//       STAT_EVENT(proc_id, MISS_TRAIN_STREAM);
//   }
// }


// void stream_ul1_miss(Mem_Req* req) {
//   if(!STREAM_CREATE_ON_WRONGPATH && req->off_path)
//     return;

//   if((req->type == MRT_DFETCH) || (req->type == MRT_DSTORE)) {
//     Addr line_index = req->addr >> LOG2(DCACHE_LINE_SIZE);
//     // do not train, but create a stream buffer
//     if(!train_stream_filter(line_index)) {
//       int hit_index = train_create_stream_buffer(
//         get_proc_id_from_cmp_addr(req->addr), line_index, FALSE,
//         STREAM_CREATE_ON_L1_MISS);
//       if(hit_index != -1)
//         addto_train_stream_filter(line_index);
//     }
//   }
// }

void update_pref_queue(void) {
  // how many prefetch request will send to buffer
  // read from queue
  int  ii = 0;
  Flag immediate_update;

  for(ii = 0; ii < PREF_SCHEDULE_NUM; ii++) {
    int q_index = stream_pref_send_no % PREF_REQ_Q_SIZE;
    if(stream_hwp->pref_req_queue[q_index].valid) {
      uns proc_id = get_proc_id_from_cmp_addr(
        stream_hwp->pref_req_queue[q_index].line_addr);
      Addr line_addr = stream_hwp->pref_req_queue[q_index].line_addr;
      if(((model->mem == MODEL_MEM) &&
          new_mem_req(MRT_DPRF, get_proc_id_from_cmp_addr(line_addr), line_addr,
                      L1_LINE_SIZE, 1, NULL,
                      STREAM_PREF_INTO_DCACHE ? dcache_fill_line : NULL,
                      unique_count,
                      0))) {  // CMP maybe unique_count_per_core[proc_id]?
        DEBUG(proc_id, "[MissQ]line_addr[%d]:0x%s q_index:%d q_no:%d \n",
              q_index, hexstr64(stream_hwp->pref_req_queue[q_index].line_addr),
              q_index, stream_pref_send_no);
        stream_hwp->pref_req_queue[q_index].valid = FALSE;
        stream_pref_send_no++;
      } else {
        STAT_EVENT(proc_id, REQ_SEND_QUEUE_STALL);
        DEBUG(proc_id, "[MISS_FULL]\n");
        break;  // buffer is full. wait!!
      }
    }
  }


  immediate_update = L2L1_IMMEDIATE_PREF_CACHE;  // more conditions for the
                                                 // future

  if(L2HIT_STREAM_PREF_ON && !immediate_update) {
    ASSERTM(0, NUM_CORES == 0,
            "L2HIT_STREAM_PREF_ON code looks like it would not work with CMP "
            "model (how does it know which DCache to check?)");

    Flag dcache_read_port_full = FALSE;
    Flag buffer_empty          = FALSE;

    // access the dcache first  && if the dcache is miss then send to L2 access
    // queue

    while(!(dcache_read_port_full || buffer_empty)) {
      int  q_index = l2hit_stream_pref_send_no % L2HIT_PREF_REQ_Q_SIZE;
      Addr req_va  = stream_hwp->l2hit_pref_req_queue[q_index].line_addr;
      Addr line_addr;

      if(stream_hwp->l2hit_pref_req_queue[q_index].valid) {
        uns bank = req_va >> dc->dcache.shift_bits &
                   N_BIT_MASK(LOG2(DCACHE_BANKS));

        if(get_read_port(&dc->ports[bank])) {
          Dcache_Data* data = (Dcache_Data*)cache_access(&dc->dcache, req_va,
                                                         &line_addr, FALSE);

          if(data) {
            // L1 hit
            stream_hwp->l2hit_pref_req_queue[q_index].valid = FALSE;
            l2hit_stream_pref_send_no++;
            DEBUG(
              0,
              "[L2HITDCACHEHIT]li:0x%s, line_addr[%d]:0x%s q_index:%d q_no:%d "
              "\n",
              hexstr64(stream_hwp->l2hit_pref_req_queue[q_index].line_index),
              q_index, hexstr64(req_va), q_index, l2hit_stream_pref_send_no);
            STAT_EVENT(0, L2HIT_PREF_REQ_DCACHE_HIT);
          } else {
            STAT_EVENT(
              0, L2HIT_L2SEND_Q_FULL +
                   (stream_hwp
                        ->l2hit_l2send_req_queue[l2hit_l2access_req_no %
                                                 L2HIT_L2ACCESS_REQ_Q_SIZE]
                        .valid ?
                      0 :
                      1));

            stream_hwp
              ->l2hit_l2send_req_queue[l2hit_l2access_req_no %
                                       L2HIT_L2ACCESS_REQ_Q_SIZE]
              .line_addr = stream_hwp->l2hit_pref_req_queue[q_index].line_addr;
            stream_hwp
              ->l2hit_l2send_req_queue[l2hit_l2access_req_no %
                                       L2HIT_L2ACCESS_REQ_Q_SIZE]
              .line_index =
              stream_hwp->l2hit_pref_req_queue[q_index].line_index;
            stream_hwp
              ->l2hit_l2send_req_queue[l2hit_l2access_req_no %
                                       L2HIT_L2ACCESS_REQ_Q_SIZE]
              .valid = stream_hwp->l2hit_pref_req_queue[q_index].valid;
            stream_hwp
              ->l2hit_l2send_req_queue[l2hit_l2access_req_no++ %
                                       L2HIT_L2ACCESS_REQ_Q_SIZE]
              .rdy_cycle = cycle_count +
                           DCACHE_CYCLES;  // dcache_access time model
            stream_hwp->l2hit_pref_req_queue[q_index].valid = FALSE;
            l2hit_stream_pref_send_no++;

            DEBUG(
              0,
              "[L2HITL2QENTERli:0x%s, line_addr[%d]:0x%s q_index:%d q_no:%d "
              "l2q_enter_no:%d\n",
              hexstr64(stream_hwp->l2hit_pref_req_queue[q_index].line_index),
              q_index, hexstr64(req_va), q_index, l2hit_stream_pref_send_no,
              l2hit_l2access_req_no);
            STAT_EVENT(0, L2HIT_PREF_REQ_DCACHE_MISS);
          }
        } else
          dcache_read_port_full = TRUE;  // at this point we just stop here.
      } else
        buffer_empty = TRUE;
    }

    // request the prefetcher into the l2 cache
    for(ii = 0; ii < L2HIT_STREAM_SCHEDULE_NUM; ii++) {
      uns q_index = l2hit_l2access_send_no % L2HIT_L2ACCESS_REQ_Q_SIZE;
      if(stream_hwp->l2hit_l2send_req_queue[q_index].valid &&
         (cycle_count >=
          stream_hwp->l2hit_l2send_req_queue[q_index].rdy_cycle)) {
        if(((model->mem == MODEL_MEM) &&
            // cmp FIXME
            new_mem_req(MRT_DPRF, 0,
                        stream_hwp->l2hit_l2send_req_queue[q_index].line_addr,
                        L1_LINE_SIZE, 1, NULL,
                        (L2L1_FILL_PREF_CACHE ? dc_pref_cache_fill_line :
                                                dcache_fill_line),
                        unique_count,
                        0)))  // FIXME CMP maybe unique_count_per_core[proc_id]
        {
          STAT_EVENT(0, L2HIT_MEM_REQ);
          DEBUG(
            0, "[L2HITL2ACCQ]line_addr[%d]:0x%s li:0x%s q_no:%d \n", q_index,
            hexstr64(stream_hwp->l2hit_l2send_req_queue[q_index].line_addr),
            hexstr64(stream_hwp->l2hit_l2send_req_queue[q_index].line_index),
            l2hit_l2access_send_no);
          stream_hwp->l2hit_l2send_req_queue[q_index].valid = FALSE;
          l2hit_l2access_send_no++;
        }
      }
    }
  }
}


int train_create_stream_buffer(uns proc_id, Addr line_index, Flag train,
                               Flag create) {
  int ii;
  int dir;
  int lru_index = -1;

  if(train || create) {
    for(ii = 0; ii < STREAM_BUFFER_N; ii++) {
      if(stream_hwp->stream[ii].valid && stream_hwp->stream[ii].trained) {
        if(((stream_hwp->stream[ii].sp <= line_index) &&
            (stream_hwp->stream[ii].ep >= line_index) &&
            (stream_hwp->stream[ii].dir == 1)) ||
           ((stream_hwp->stream[ii].sp >= line_index) &&
            (stream_hwp->stream[ii].ep <= line_index) &&
            (stream_hwp->stream[ii].dir == -1))) {
          // found a trained buffer
          return ii;
        }
      }
    }

    for(ii = 0; ii < STREAM_BUFFER_N; ii++) {
      if(stream_hwp->stream[ii].valid && (!stream_hwp->stream[ii].trained)) {
        if((stream_hwp->stream[ii].sp <= (line_index + STREAM_TRAIN_LENGTH)) &&
           (stream_hwp->stream[ii].sp >=
            (line_index - STREAM_TRAIN_LENGTH))) {  // FIXME: should creation be
                                                    // done based on
                                                    // STREAM_LENGTH?

          if(train) {  // do these only if we are training
            // decide the train dir
            if(stream_hwp->stream[ii].sp > line_index)
              dir = -1;
            else
              dir = 1;
            stream_hwp->stream[ii].trained = TRUE;
            stream_hwp->stream[ii].ep      = (dir > 0) ?
                                          line_index + STREAM_START_DIS :
                                          line_index -
                                            STREAM_START_DIS;  // BUG 57
            stream_hwp->stream[ii].dir = dir;
            DEBUG(proc_id,
                  "stream  trained stream_index:%3d sp %7s ep %7s dir %2d "
                  "miss_index %7d\n",
                  ii, hexstr64(stream_hwp->stream[ii].sp),
                  hexstr64(stream_hwp->stream[ii].ep),
                  stream_hwp->stream[ii].dir, (int)line_index);
          }

          return ii;
        }
      }
    }

    if(!create)
      return -1;
  }

  if(create) {
    // search for invalid buffer
    for(ii = 0; ii < STREAM_BUFFER_N; ii++) {
      if(!stream_hwp->stream[ii].valid) {
        lru_index = ii;
        break;
      }
    }

    // search for oldest buffer

    if(lru_index == -1) {
      lru_index = 0;
      for(ii = 0; ii < STREAM_BUFFER_N; ii++) {
        if(stream_hwp->stream[ii].lru < stream_hwp->stream[lru_index].lru) {
          lru_index = ii;
        }
      }
      STAT_EVENT(proc_id, REPLACE_OLD_STREAM);
    }

    // create new train buffer

    stream_hwp->stream[lru_index].lru         = cycle_count;
    stream_hwp->stream[lru_index].valid       = TRUE;
    stream_hwp->stream[lru_index].sp          = line_index;
    stream_hwp->stream[lru_index].ep          = line_index;
    stream_hwp->stream[lru_index].train_hit   = 1;
    stream_hwp->stream[lru_index].trained     = FALSE;
    stream_hwp->stream[lru_index].buffer_full = FALSE;

    STAT_EVENT(proc_id, STREAM_TRAIN_CREATE);
    DEBUG(proc_id,
          "create new stream : stream_no :%3d, line_index %7s sp = %7s\n",
          lru_index, hexstr64(line_index),
          hexstr64(stream_hwp->stream[lru_index].sp));
    return lru_index;
  }

  return -1;
}


Flag train_stream_filter(Addr line_index) {
  int ii;
  for(ii = 0; ii < TRAIN_FILTER_SIZE; ii++) {
    if(train_filter[ii] == line_index) {
      return TRUE;
    }
  }
  return FALSE;
}

static inline void addto_train_stream_filter(Addr line_index) {
  train_filter[(train_filter_no++) % TRAIN_FILTER_SIZE] = line_index;
}


Flag pref_req_queue_filter(Addr addr) {
  int ii;
  if(!PREF_REQ_QUEUE_FILTER_ON)
    return FALSE;
  for(ii = 0; ii < PREF_REQ_Q_SIZE; ii++) {
    if((stream_hwp->pref_req_queue[ii].line_addr >> LOG2(DCACHE_LINE_SIZE)) ==
       (addr >> LOG2(DCACHE_LINE_SIZE))) {
      stream_hwp->pref_req_queue[ii].valid = FALSE;
      STAT_EVENT(get_proc_id_from_cmp_addr(addr),
                 STREAM_REQ_QUEUE_HIT_BY_DEMAND);
      return TRUE;
      break;
    }
  }
  return FALSE;
}

/************************************************************************************/

/* dcache miss but l2 hit */

void l2_hit_stream_pref(Addr line_addr, Flag hit) {
  uns  proc_id    = get_proc_id_from_cmp_addr(line_addr);
  Addr line_index = line_addr >> LOG2(DCACHE_LINE_SIZE);
  if(!train_l2hit_stream_filter(line_index)) {
    l2hit_stream_req(line_index, hit);
    STAT_EVENT(proc_id, L2HIT_TRAIN_HIT_DEMAND + (hit ? 0 : 1));
    STAT_EVENT(proc_id, L2HIT_TRAIN_FILTER_MISS);
  } else
    STAT_EVENT(proc_id, L2HIT_TRAIN_FILTER_HIT);
}

Flag train_l2hit_stream_filter(Addr line_index) {
  int ii;
  for(ii = 0; ii < TRAIN_FILTER_SIZE; ii++) {
    if(train_l2hit_filter[ii] == line_index) {
      return TRUE;
    }
  }
  train_l2hit_filter[(train_l2hit_filter_no++) % TRAIN_FILTER_SIZE] =
    line_index;
  return FALSE;
}

void l2hit_stream_req(Addr line_index, Flag hit) {
  uns          ii;
  int          hit_index = 1;
  Pref_Mem_Req new_req   = {0};
  int          dis;
  /* search for the stream buffer */
  hit_index = train_l2hit_stream_buffer(line_index, hit);
  if(!stream_hwp->l2hit_stream[hit_index].trained) {
    STAT_EVENT(0, L2HIT_MISS_TRAIN_STREAM);
    return;
  }
  stream_hwp->l2hit_stream[hit_index].lru = cycle_count;  // update lru
  STAT_EVENT(0, L2HIT_HIT_TRAIN_STREAM);


  for(ii = 0; ii < L2HIT_STREAM_PREFETCH_N; ii++) {
    if((stream_hwp->l2hit_stream[hit_index].sp == line_index) &&
       (stream_hwp->l2hit_stream[hit_index].buffer_full)) {
      // stream prefetch is requesting  enough far ahead
      // stop prefetch and wait until miss address is within the buffer area
      return;
    }

    new_req.line_index = stream_hwp->l2hit_stream[hit_index].ep +
                         stream_hwp->l2hit_stream[hit_index].dir;
    new_req.line_addr = (new_req.line_index) << LOG2(DCACHE_LINE_SIZE);
    new_req.valid     = TRUE;

    if(stream_hwp
         ->l2hit_pref_req_queue[l2hit_stream_pref_req_no %
                                L2HIT_PREF_REQ_Q_SIZE]
         .valid) {
      DEBUG(0, "[l2HITP] PREF_QUEU overlap!!\n");
      STAT_EVENT(0, L2HIT_STREAM_PREF_REQ_QUE_FULL);
      if(STREAM_STALL_ON_QUEUE_FULL)
        return;
    }

    if(L2L1_IMMEDIATE_PREF_CACHE && DC_PREF_CACHE_ENABLE) {
      dc_pref_cache_insert(new_req.line_addr);
    } else {
      stream_hwp->l2hit_pref_req_queue[l2hit_stream_pref_req_no++ %
                                       L2HIT_PREF_REQ_Q_SIZE] = new_req;
    }

    stream_hwp->l2hit_stream[hit_index].ep =
      stream_hwp->l2hit_stream[hit_index].ep +
      stream_hwp->l2hit_stream[hit_index].dir;
    dis = stream_hwp->l2hit_stream[hit_index].ep -
          stream_hwp->l2hit_stream[hit_index].sp;
    if(((stream_hwp->l2hit_stream[hit_index].dir == 1) &&
        (dis > L2HIT_STREAM_LENGTH)) ||
       ((stream_hwp->l2hit_stream[hit_index].dir == -1) &&
        (dis < -L2HIT_STREAM_LENGTH))) {
      stream_hwp->l2hit_stream[hit_index].buffer_full = TRUE;
      stream_hwp->l2hit_stream[hit_index].sp =
        stream_hwp->l2hit_stream[hit_index].sp +
        stream_hwp->l2hit_stream[hit_index].dir;
    }
    STAT_EVENT(0, L2HIT_STREAM_BUFFER_REQ);

    if(REMOVE_REDUNDANT_STREAM)
      remove_redundant_stream(hit_index);

    DEBUG(0,
          "[L2HITPInQ**%s**]ma:0x%7s mi:0x%7s d:%2d ri:0x%7s, ra:0x%7s b:%2d "
          "sp:0x%7s ep:0x%7s send_no:%d req_no:%d\n",
          hit ? "H" : "M", hexstr64(line_index << LOG2(DCACHE_LINE_SIZE)),
          hexstr64(line_index), stream_hwp->l2hit_stream[hit_index].dir,
          hexstr64(new_req.line_index), hexstr64(new_req.line_addr), hit_index,
          hexstr64(stream_hwp->l2hit_stream[hit_index].sp),
          hexstr64(stream_hwp->l2hit_stream[hit_index].ep),
          l2hit_stream_pref_send_no, l2hit_stream_pref_req_no);
  }
}


int train_l2hit_stream_buffer(Addr line_index, Flag hit) {
  int ii;
  int dir;
  int lru_index = -1;
  for(ii = 0; ii < L2HIT_STREAM_BUFFER_N; ii++) {
    if(stream_hwp->l2hit_stream[ii].valid &&
       stream_hwp->l2hit_stream[ii].trained) {
      if(((stream_hwp->l2hit_stream[ii].sp <= line_index) &&
          (stream_hwp->l2hit_stream[ii].ep >= line_index) &&
          (stream_hwp->l2hit_stream[ii].dir == 1)) ||
         ((stream_hwp->l2hit_stream[ii].sp >= line_index) &&
          (stream_hwp->l2hit_stream[ii].ep <= line_index) &&
          (stream_hwp->l2hit_stream[ii].dir == -1))) {
        // found a trained buffer
        return ii;
      }
    }
  }

  for(ii = 0; ii < L2HIT_STREAM_BUFFER_N; ii++) {
    if(stream_hwp->l2hit_stream[ii].valid &&
       (!stream_hwp->l2hit_stream[ii].trained)) {
      if((stream_hwp->l2hit_stream[ii].sp <=
          (line_index + L2HIT_STREAM_LENGTH)) &&
         (stream_hwp->l2hit_stream[ii].sp >=
          (line_index - L2HIT_STREAM_LENGTH))) {
        // decide the train dir
        if(stream_hwp->l2hit_stream[ii].sp > line_index)
          dir = -1;
        else
          dir = 1;
        stream_hwp->l2hit_stream[ii].trained = TRUE;
        stream_hwp->l2hit_stream[ii].ep      = (dir > 0) ?
                                            line_index +
                                              L2HIT_STREAM_START_DIS :
                                            line_index - L2HIT_STREAM_START_DIS;
        stream_hwp->l2hit_stream[ii].dir = dir;
        DEBUG(0,
              "[l2HITP**%s**]stream  trained stream_index:%3d sp 0x%7s ep "
              "0x%7s dir %2d miss_index %7s\n",
              hit ? "H" : "M", ii, hexstr64(stream_hwp->l2hit_stream[ii].sp),
              hexstr64(stream_hwp->l2hit_stream[ii].ep),
              stream_hwp->l2hit_stream[ii].dir, hexstr64(line_index));
        return ii;
      }
    }
  }

  // search for invalid buffer
  for(ii = 0; ii < L2HIT_STREAM_BUFFER_N; ii++) {
    if(!stream_hwp->l2hit_stream[ii].valid) {
      lru_index = ii;
      break;
    }
  }

  // search for oldest buffer

  if(lru_index == -1) {
    lru_index = 0;
    for(ii = 0; ii < L2HIT_STREAM_BUFFER_N; ii++) {
      if(stream_hwp->l2hit_stream[ii].lru <
         stream_hwp->l2hit_stream[lru_index].lru) {
        lru_index = ii;
      }
    }
    STAT_EVENT(0, REPLACE_OLD_STREAM);
  }

  // create new train buffer

  stream_hwp->l2hit_stream[lru_index].lru         = cycle_count;
  stream_hwp->l2hit_stream[lru_index].valid       = TRUE;
  stream_hwp->l2hit_stream[lru_index].sp          = line_index;
  stream_hwp->l2hit_stream[lru_index].ep          = line_index;
  stream_hwp->l2hit_stream[lru_index].train_hit   = 1;
  stream_hwp->l2hit_stream[lru_index].trained     = FALSE;
  stream_hwp->l2hit_stream[lru_index].buffer_full = FALSE;
  stream_hwp->l2hit_stream[lru_index].dir         = 0;

  STAT_EVENT(0, L2HIT_STREAM_TRAIN_CREATE);

  DEBUG(0,
        "[L2HITP]create new l2hit stream : stream_no :%3d, line_index %7s sp = "
        "%7s\n",
        lru_index, hexstr64(line_index),
        hexstr64(stream_hwp->l2hit_stream[lru_index].sp));
  return lru_index;
}

void remove_redundant_stream(int hit_index) {
  int ii;

  for(ii = 0; ii < STREAM_BUFFER_N; ii++) {
    if((ii == hit_index) || (!stream_hwp->stream[ii].valid))
      continue;
    if(((stream_hwp->stream[ii].ep < stream_hwp->stream[hit_index].ep) &&
        (stream_hwp->stream[ii].ep > stream_hwp->stream[hit_index].sp)) ||
       ((stream_hwp->stream[ii].sp < stream_hwp->stream[hit_index].ep) &&
        (stream_hwp->stream[ii].sp > stream_hwp->stream[hit_index].sp))) {
      stream_hwp->stream[ii].valid = FALSE;
      STAT_EVENT(0, REMOVE_REDUNDANT_STREAM_STAT);
      DEBUG(
        0,
        "stream[%d] sp:0x%s ep:0x%s is removed by stream[%d] sp:0x%s ep:0x%s\n",
        ii, hexstr64(stream_hwp->stream[ii].sp),
        hexstr64(stream_hwp->stream[ii].ep), hit_index,
        hexstr64(stream_hwp->stream[hit_index].sp),
        hexstr64(stream_hwp->stream[hit_index].ep));
    }
  }
}
