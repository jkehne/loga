/**************************************************************************/
/*!
  \author  Konstantinos Menychtas --- kmenycht@cs.rochester.edu
  \brief   Event-based Fair-Queueing scheduling algorithm; interface
*/
/**************************************************************************/

#ifndef __NEON_EVENT_H__
#define __NEON_EVENT_H__

#include <linux/sysctl.h>  // sysctl
#include <linux/list.h>
#include <asm/atomic.h>

/**************************************************************************/
// sysctl/proc managed options

//#define NEON_EVENT_COMP0_ONLY          //   account only compute requests on dev0
#define NEON_EVENT_T_MAX         1000  //   1  Sec
#define NEON_EVENT_T_DEFAULT        1  //   1 mSec (per task)
#define NEON_EVENT_X_DEFAULT        5  //   5 x event season time
// exported sysctl/proc knob
extern ctl_table neon_knob_event_options [];

#define NEON_POLICY_EVENT_KNOB {                \
    .procname = "event",                        \
      .mode = 0555,                             \
      .child = neon_knob_event_options       \
      }

/**************************************************************************/
// epoch and seasons
typedef enum {
  DFQ_EVENT_SAMPLING,
  DFQ_EVENT_FREERUN,
  DFQ_TASK_NOFSEASONS_EVENT
} event_season_t;

// policy-specific work, task and dev sched struct entries
typedef struct {
  // reference counter of last realized work (used for idleness detection)
  unsigned int last_seen;
  // engage-control
  unsigned int engage;
  // flag marking work as ignored for accounting purposes
  unsigned int heed;
} event_work_t;

typedef struct _sched_work_t_ sched_work_t;

typedef struct {
  // number of channels occupied with works
  unsigned int occ_chans;
  // number of channels with managed works (used when ignoring DMA for sched)
  unsigned int mng_chans;
  // task virtual time
  unsigned long vtime;
  // # of dma/compute requests sampled in the last sample period
  atomic64_t ntimes_run;
  // cumulative runtime of dma/compute rqsts sampled in last period
  atomic64_t exe_dt_sampled;
  // flag suggesting task did not run in last freerun period
  unsigned int held_back;
  // semaphore state counter
  int sem_count;
  // block at this semaphore
  struct semaphore sem;
} event_task_t;

typedef struct _sched_task_t_ sched_task_t;

typedef struct {
  // DFQ status
  event_season_t season;
  // device virtual time
  unsigned long vtime;
  // total number of occupied channels
  unsigned long occ_chans;
  // idleness flag (not set if pending requests exist)
  unsigned int active;
  // channel draining countdown
  unsigned int countdown;
  // timestamp marking block-till-completion-update
  unsigned long update_ts;
  // channel to task mapping
  sched_task_t **chan_to_task;
  // season change event flag (for event handler thread)
  atomic_t action;
  // epoch counter (full cycle)
  struct hrtimer season_timer;
} event_dev_t;

/**************************************************************************/
// EVENT policy's scheduling interface
struct _neon_policy_face_t_; // early declaration; in neon_policy.h
extern struct _neon_policy_face_t_ neon_policy_eventp;

#endif // __NEON_EVENT_H__
