#ifndef __STREAM_PREF_H__
#define __STREAM_PREF_H__

#include "globals/global_types.h"

/**************************************************************************************/
/* Forward Declarations */

struct Mem_Req_struct;
struct Pref_Mem_Req_struct;
struct Stream_Buffer_struct;

/**************************************************************************************/
/* Types */

typedef struct Bingo_History_Entry_Struct {
  Addr last_address; // The last address accessed in this stream
  int delta_history[DELTA_HISTORY_SIZE]; // Array to store the delta history for the stream
  int current_delta_index; // Points to the next free index in the delta history array
} Bingo_History_Entry;

typedef struct Bingo_Pattern_Detection_Entry_Struct {
  int pattern[MAX_PATTERN_LENGTH]; // A detected delta pattern
  int next_delta; // The predicted next delta after the pattern
} Bingo_Pattern_Detection_Entry;

typedef struct Stream_HWP_Struct {
  // Stream HWP (Hardware Prefetcher)
  Stream_Buffer* stream;
  Stream_Buffer* l2hit_stream;

  /* Prefetch request queues */
  Pref_Mem_Req* pref_req_queue;
  Pref_Mem_Req* l2hit_pref_req_queue;
  Pref_Mem_Req* l2hit_l2send_req_queue;

  /* Bingo-specific data structures */
  Bingo_History_Entry* delta_history_table; // Table for tracking delta history of streams
  Bingo_Pattern_Detection_Entry* pattern_detection_table; // Table for storing detected patterns
  Pref_Mem_Req* bingo_prefetch_candidates; // Queue of prefetch candidates based on pattern detection
} Stream_HWP;

/**************************************************************************************/
/* Function Prototypes */

/* Core prefetcher functions */
void stream_dl0_miss(Addr line_addr);
void stream_ul1_miss(Mem_Req* req);
void update_pref_queue(void);
void init_stream_HWP(void);
int  train_create_stream_buffer(uns proc_id, Addr line_index, Flag train,
                                Flag create);
Flag train_stream_filter(Addr line_index);
Flag pref_req_queue_filter(Addr line_addr);

/* Bingo-specific functions */
void bingo_dl0_miss(Addr line_addr); // Triggered on DL0 cache misses
void bingo_pattern_detection(Addr line_addr, int delta); // Detect patterns based on deltas
void bingo_prefetch(Addr line_addr, int predicted_delta); // Issue prefetch requests based on predicted deltas
void update_delta_history(Addr line_addr, int delta); // Update the delta history for a given stream

/* Functions for handling L2 hits (optional, depending on Bingo prefetcher design) */
void l2_hit_stream_pref(Addr line_addr, Flag hit);
Flag train_l2hit_stream_filter(Addr line_index);
void l2hit_stream_req(Addr line_index, Flag hit);
int  train_l2hit_stream_buffer(Addr line_index, Flag hit);
void stream_dl0_hit_train(Addr line_addr);

#endif /*  __STREAM_PREF_H__ */
