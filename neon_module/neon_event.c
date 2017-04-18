/**************************************************************************/
/*!
  \author  Kontantinos Menychtas --- kmenycht@cs.rochester.edu
  \brief   Sampling-based Fair-Queueing scheduling algorithm; routines
*/
/**************************************************************************/

#include <linux/kernel.h>   // BUILD_BUG_ON_ZERO
#include <linux/slab.h>     // kmalloc/kzalloc
#include <linux/sysctl.h>   // sysctl
#include <linux/ktime.h>    // ktime
#include <linux/delay.h>
#if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG)
#include <linux/sched.h>    // touch_softlockup_watchdog
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <linux/string.h>   // memset
#include <linux/export.h>   // THIS_MODULE
#include <asm/uaccess.h>    // copy_to_user
#endif /* LOGA_TRACE || LOGA_TRACE_DEBUG */
#include <asm/processor.h>  // cpu_relax
#include "neon_policy.h"    // policy interface
#include "neon_help.h"      // print wrappers
//===========================================================
// GPU (sub)state definitions and accessors
#include "loga_p.h"
#include "loga_phase_stats.h"

/***************************************************************************/
#define DFQ(x) ps.evnt.x

static char season_name[DFQ_TASK_NOFSEASONS_EVENT+1][NAME_LEN] = {
  {'S', 'A', 'M', 'P', 'L', 'I', 'N', 'G',  '\0',  0 },
  {'F', 'R', 'E', 'E', 'R', 'U', 'N', '\0', '\0',  0 }
};

//===========================================================
// Acconting data structures
static int last_known_running_cid;
static u64 last_timestamp; // When did we first notice the "last known state/cid" declared above
static phase_stats_t stats;

// sampling period
static unsigned int _event_T_ = NEON_EVENT_T_DEFAULT;
static unsigned int  event_T  = NEON_EVENT_T_DEFAULT;
ktime_t event_interval;

// all samples should be collected by cut-off time
static unsigned int _event_X_ = NEON_EVENT_X_DEFAULT;
static unsigned int  event_X  = NEON_EVENT_X_DEFAULT;

// Event queue for suspending/waking up the sampling thread
DECLARE_WAIT_QUEUE_HEAD(sampling_thread_queue);
DECLARE_WAIT_QUEUE_HEAD(sampling_thread_awake_queue);
atomic_t sampling_threads_awake;

// sysctl/proc options
ctl_table neon_knob_event_options [] = {
  {
    .procname = "event_T",
    .data = &_event_T_,
    .maxlen = sizeof(int),
    .mode = 0666,
    .proc_handler = &proc_dointvec,
  },
  {
    .procname = "event_X",
    .data = &_event_X_,
    .maxlen = sizeof(int),
    .mode = 0666,
    .proc_handler = &proc_dointvec,
  },
  { 0 }
};

/**************************************************************************/
// no-interference (event) policy interface
static int  init_event(void);
static void fini_event(void);
static void reset_event(unsigned int onoff);
static int  create_event(sched_task_t *sched_task);
static void destroy_event(sched_task_t *sched_task);
static void start_event(sched_dev_t  * const sched_dev,
                           sched_work_t * const sched_work,
                           sched_task_t * const sched_task);
static void stop_event(sched_dev_t  * const sched_dev,
                          sched_work_t * const sched_work,
                          sched_task_t * const sched_task);
static void submit_event(sched_dev_t  * const sched_dev,
                            sched_work_t * const sched_work,
                            sched_task_t * const sched_task);
static void issue_event(sched_dev_t  * const sched_dev,
                           sched_work_t * const sched_work,
                           sched_task_t * const sched_task,
                           unsigned int had_blocked);
static void complete_event(sched_dev_t  * const sched_dev,
                              sched_work_t * const sched_work,
                              sched_task_t * const sched_task);
static void event_event(void);
static int  reengage_map_event(const neon_map_t * const neon_map);

neon_policy_face_t neon_policy_eventp = {
  .init = init_event,
  .fini = fini_event,
  .create = create_event,
  .destroy = destroy_event,
  .reset = reset_event,
  .start = start_event,
  .stop = stop_event,
  .submit = submit_event,
  .issue = issue_event,
  .complete = complete_event,
  .event = event_event,
  .reengage_map = reengage_map_event
};

#if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG)
typedef struct {
  short ctx;
  unsigned short duration;
} loga_trace_entry_t;

static struct dentry *debugfs_dir;
static struct dentry *debugfs_tracebuffer;
static struct dentry *debugfs_stats;
static loga_trace_entry_t *tracebuffer;
static unsigned int tb_pointer = 0;
struct debugfs_blob_wrapper tb_wrapper;
static atomic64_t idle_time;
static atomic64_t ntimes_idle;

#define TRACEBUFFER_SIZE (10000000 * sizeof(loga_trace_entry_t))

static void reset_task_stats(sched_task_t *sched_task) {
  atomic64_set(&sched_task->DFQ(exe_dt_sampled), 0);
  atomic64_set(&sched_task->DFQ(ntimes_run), 0);
}

static void reset_device_stats(sched_dev_t *sched_dev) {
  sched_task_t *stask;

  list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
    reset_task_stats(stask);
  }
}

static ssize_t debugfs_stats_write(struct file *filep, char __user *buffer, size_t count, loff_t *ppos) {
  int i;

  for (i = 0; i < neon_global.ndev; ++i)
    reset_device_stats(&sched_dev_array[i]);

  return count;
}

static const char *find_task_name_by_pid(pid_t pid) {
  struct task_struct *task;
  task = pid_task(find_vpid(pid), PIDTYPE_PID);
  return task->comm;
}

static u32 get_device_stats(sched_dev_t *sched_dev, char *buf, u32 pos, u32 len) {
  sched_task_t *stask;

  list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
    pos += snprintf(buf + pos,
		    len - pos,
		    "Total time consumed by %s: %ld\n",
		    find_task_name_by_pid(stask->pid),
		    atomic64_read(&stask->DFQ(exe_dt_sampled))
		    );
    if (pos >= len)
      return len;
  }
  return pos;
}

static ssize_t debugfs_stats_read(struct file *filep, char __user *buffer, size_t count, loff_t *ppos) {
  int i;
  char *buf;
  u32 pos = 0;

  buf = kzalloc(count, GFP_KERNEL);

  for (i = 0; i < neon_global.ndev; ++i)
    pos = get_device_stats(&sched_dev_array[i], buf, pos, count);

  copy_to_user(buffer, buf, strlen(buf) + 1);

  kfree(buf);

  return strlen(buf) + 1;
}

struct file_operations debugfs_stats_fops = {
  .owner = THIS_MODULE,
  .write = debugfs_stats_write,
  .read = debugfs_stats_read,
};

static void make_debugfs_entries(void) {
  tracebuffer = vmalloc(TRACEBUFFER_SIZE);
  BUG_ON(!tracebuffer);
  memset(tracebuffer, 0, TRACEBUFFER_SIZE);

  debugfs_dir = debugfs_create_dir("neon", NULL);
  if (!debugfs_dir) {
    WARN(1, "Failed to allocate debugfs dir");
    return;
  }

  tb_wrapper.data = (void *)tracebuffer;
  tb_wrapper.size = TRACEBUFFER_SIZE;

  debugfs_tracebuffer = debugfs_create_blob("trace", 0444, debugfs_dir, &tb_wrapper);
  if (!debugfs_tracebuffer) {
    WARN(1, "Failed to allocate debugfs trace entry");
    return;
  }

  debugfs_stats = debugfs_create_file("stats", 0666, debugfs_dir, NULL, &debugfs_stats_fops);
  if (!debugfs_stats) {
    WARN(1, "Failed to allocate debugfs reset entry");
    return;
  }
}

static void destroy_debugfs_entries(void) {
  if (debugfs_dir)
    debugfs_remove_recursive(debugfs_dir);
  vfree(tracebuffer);
}
#endif /* LOGA_TRACE || LOGA_TRACE_DEBUG */

/* Accounts the given duration (in nanoseconds) to the given channel.
   Does not check if "cid" is non-negative. */
static inline void account_time_unchecked(int cid, u64 ns, sched_task_t *sched_task)
{
  /*
   * Note: No critical section is necessary here. Since we always wait
   * for all LoGA-Threads to finish before reading these values, it does
   * not matter how these operations interleave in different threads, as
   * long as none of them are lost completely.
   * Also, there is no atomic operation for exe_dt since changing exe_dt
   * to atomic_t would mess up the other policies, and in ours, it serves
   * informational purposes (debug output) only.
   */
  u64 us = ns / NSEC_PER_USEC;
  sched_task->exe_dt += us;
  atomic64_add(us, &sched_task->DFQ(exe_dt_sampled));
  atomic64_inc(&sched_task->DFQ(ntimes_run));
}

/* Accounts the given duration (in nanoseconds) to the given channel.
   Pass a negative cid to do nothing (no one was running) */
static inline void account_time(int cid, u64 ns, sched_dev_t *sched_dev)
{
  if (cid >= 0) {
    sched_task_t *sched_task = sched_dev->DFQ(chan_to_task)[cid];
    if(sched_task) {
      account_time_unchecked(cid, ns, sched_task);
    }
  }
#ifdef LOGA_TRACE
  else {
    atomic64_add(ns / NSEC_PER_USEC, &idle_time);
    atomic64_inc(&ntimes_idle);
  }
#endif

/* #if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG) */
/*   tracebuffer[tb_pointer].ctx = cid; */
/*   tracebuffer[tb_pointer].duration = ns / NSEC_PER_USEC; */
/*   tb_pointer++; */
/* #endif /\* LOGA_TRACE || LOGA_TRACE_DEBUG *\/ */
}

/* Account for the time between the last noticed state change and
   the end of the polling phase.
   If a changelog is used, go through it and perform accounting. */
static
inline void loga_pollphase_post(u64 id, phase_stats_t stats, sched_dev_t *sched_dev)
{
  account_time(last_known_running_cid, stats.end_poll_ns - last_timestamp, sched_dev);
}

/* Perform an initial (for this active phase) GPU state polling round.
 * Estimate and account GPU usage during the preceding sleep phase
 */
static
inline void loga_pollphase_pre(void __iomem *bar0, const phase_stats_t stats, sched_dev_t *sched_dev)
{
	// Determine currently running channel
	const int cid_cur  = get_running_cid(bar0);

	// Update "last known" variables and timestamp with the new values
	last_known_running_cid = cid_cur;

#ifdef LOGA_TRACE_DEBUG
	const u64 new_timestamp = ktime_to_ns(ktime_get());
	account_time(-4200, USHRT_MAX, sched_dev);
	last_timestamp = new_timestamp;
#else
	last_timestamp = ktime_to_ns(ktime_get());
#endif // LOGA_TRACE_DEBUG

}

/* Performs a single GPU state polling round. */
static inline void loga_do_poll(void __iomem *bar0, sched_dev_t *sched_dev)
{
  const int old_cid = last_known_running_cid;
  const int cur_cid = get_running_cid(bar0);

  if(old_cid != cur_cid) {
    const u64 now = ktime_to_ns(ktime_get());

    account_time(old_cid, now - last_timestamp, sched_dev);
    last_known_running_cid = cur_cid;
    last_timestamp = now;
  }
}

/* Continously performs polling rounds until "duration_jifs" jiffies have elapsed */
static
inline u64 loga_poll_phase(void __iomem *bar0, sched_dev_t *sched_dev)
{
  u64 num_iters = 0; // #Iterations in this phase (1 iteration = 1 full poll of all subfifos)
  sched_task_t  *stask = NULL;

  while(likely(sched_dev->DFQ(season) == DFQ_EVENT_SAMPLING))
    {
      loga_do_poll(bar0, sched_dev);
      ++num_iters; // Count iterations for profiling
      cpu_relax();
#ifdef LOGA_TRACE
      if (unlikely(num_iters > 1000000)) {
	touch_softlokup_watchdog();
	num_iters = 0;
      }
#endif /* LOGA_TRACE */
    }

  return num_iters;
}


/* Thread main function: until kthread_stop() is called,
   cycles between active phases (preparation, polling, post-evaluation) and
   relaxation phases: [information dump, and] either sleep or schedule() */
static int loga_thread(void *dev_id_ptr)
{
  u64 dev_id = (u64)dev_id_ptr;
  int ret;
  u64 poll_phase_id = 0; // A unique identifier for each polling phase (a simple counter)
  u64 sleepphase_begin = 0, sleepphase_end = 0;

  sched_dev_t *sched_dev = &sched_dev_array[dev_id];
  void __iomem *bar0 = neon_global.dev[dev_id].bar0;

  atomic_inc(&sampling_threads_awake);

  neon_report("LoGA Thread started for device %u!", dev_id);

  // ===================
  // MAIN LOOP

  while(likely(!kthread_should_stop()))
    {
      switch (sched_dev->DFQ(season)) {
      case DFQ_EVENT_FREERUN:
	if (!sleepphase_begin) //somehow we never entered polling between freerun phases
	  sleepphase_begin = ktime_to_ns(ktime_get());

	neon_report("Entering freerun season");

	atomic_dec(&sampling_threads_awake);
	wake_up_interruptible(&sampling_thread_awake_queue);

	ret = wait_event_interruptible(sampling_thread_queue, (sched_dev->DFQ(season) == DFQ_EVENT_SAMPLING) || unlikely(kthread_should_stop()));
	if (ret)
	  neon_error("wait_event_interruptible returned %d", ret);

	atomic_inc(&sampling_threads_awake);

	break;
      case DFQ_EVENT_SAMPLING:
	neon_report("Entering sampling season");
	if (sleepphase_begin) {
	  sleepphase_end = ktime_to_ns(ktime_get());
	  stats.sleep_duration_ns = sleepphase_end - sleepphase_begin;
	  neon_report("Freerun season lasted %u us", (sleepphase_end - sleepphase_begin) / NSEC_PER_USEC);
	  sleepphase_begin = 0;
	} 

	// Timestamps for both profiling purposes and for proper accounting
	// See: http://stackoverflow.com/questions/15995295/measuring-time-in-linux-kernel-space-with-sub-microsecond-precision
	loga_pollphase_pre(bar0, stats, sched_dev);

	stats.begin_poll_ns = ktime_to_ns(ktime_get());
	stats.num_pollphase_iters = loga_poll_phase(bar0, sched_dev);

	stats.end_poll_ns = ktime_to_ns(ktime_get());
	loga_pollphase_post(poll_phase_id, stats, sched_dev);

	neon_report("Poll phase %u did %u iterations in %u us",
		    poll_phase_id,
		    stats.num_pollphase_iters,
		    (stats.end_poll_ns - stats.begin_poll_ns) / NSEC_PER_USEC);

	poll_phase_id++;
	break;
      default:
	neon_error("Unknown season");
	break;
      }

    }

  neon_report( "loga_thread() returning...\n");
  return 0;
}

static struct task_struct **loga_task;

/**************************************************************************/
// loga_init/fini
/**************************************************************************/
// Setup and teardown for loga thread
void loga_init(void)
{
  u64 i;

  neon_urgent("Initializing Low-overhead GPU Accounting...\n");

  loga_task = kzalloc(neon_global.ndev * sizeof(struct task_struct *), GFP_KERNEL);
  atomic_set(&sampling_threads_awake, 0);

  for (i = 0; i < neon_global.ndev; ++i)
    loga_task[i] = kthread_run(&loga_thread, (void *)i, "LoGA Thread %llu", i);
}

void loga_fini(void)
{
  u32 i;
  int loga_ret;

  neon_urgent("Terminating Low-overhead GPU Accounting...\n");

  for (i = 0; i < neon_global.ndev; ++i) {
    if (!loga_task[i])
      continue;
    loga_ret = kthread_stop(loga_task[i]);
    neon_report("LoGA thread %u was stopped. Return value: %d\n", i, loga_ret);
  }

  kfree(loga_task);
}

/**************************************************************************/
// update_vtimes
/**************************************************************************/
// update virtual times for all tasks in device
// CAREFUL : sched_dev write lock held
static void
update_vtimes(sched_dev_t  * const sched_dev)
{
  sched_task_t  *stask             = NULL;
  unsigned long  min_vtime         = (unsigned long) (-1);
  unsigned long  total_avg_exe_dt  = 0;
  unsigned long  epoch_dt          = 0;
  unsigned int   activity          = 0;

  list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
    unsigned long avg_exe_dt = 0;
    // adjust average execution time, as expected by event,
    // to ensure that blocked tasks appear as not having run at all
    activity = 1;
    if(atomic64_read(&stask->DFQ(ntimes_run)) > 0) {
      avg_exe_dt = atomic64_read(&stask->DFQ(exe_dt_sampled)) / atomic64_read(&stask->DFQ(ntimes_run));
    } else
      avg_exe_dt = atomic64_read(&stask->DFQ(exe_dt_sampled));
    total_avg_exe_dt += avg_exe_dt;
  }
  if((activity == 0) || (total_avg_exe_dt == 0))
    goto just_decide;

  epoch_dt = (stats.sleep_duration_ns + (stats.end_poll_ns - stats.begin_poll_ns)) / NSEC_PER_USEC;

  list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
    unsigned long avg_exe_dt = 0;
    unsigned long vt         = 0;
    if(atomic64_read(&stask->DFQ(ntimes_run)) == 0) { //TODO: Long-running kernels?
      // idle tasks are not considered as candidates to
      // set device vtime
      neon_report("DFQ : don't account pid %d (held_back %d)",
                  stask->pid, stask->DFQ(held_back));
      continue;
    }

    avg_exe_dt = atomic64_read(&stask->DFQ(exe_dt_sampled)) / atomic64_read(&stask->DFQ(ntimes_run));
    //avg_exe_dt = atomic64_read(&stask->DFQ(exe_dt_sampled));
    neon_report("DFQ : did %d : pid %d : exe_dt_sampled %ld : "
                "nrqst %ld : avg_exe_dt %lu ",
                sched_dev->id, stask->pid,  atomic64_read(&stask->DFQ(exe_dt_sampled)),
                atomic64_read(&stask->DFQ(ntimes_run)), avg_exe_dt);

    vt = (avg_exe_dt * epoch_dt) / total_avg_exe_dt;
    stask->DFQ(vtime) += vt;
    if(stask->DFQ(vtime) < min_vtime)
      min_vtime = stask->DFQ(vtime);
    neon_account("DFQ : did %d : pid %d : vtd = %ld : "
                 "vtp +=  %ld/%ld (exe avg/total) * "
                 "%ld (epoch_dt) = %ld -> vtp = %ld ",
                 sched_dev->id, stask->pid, sched_dev->DFQ(vtime),
                 avg_exe_dt, total_avg_exe_dt,
                 epoch_dt, vt, stask->DFQ(vtime));
  }
  if(min_vtime < (unsigned long) -1) {
    sched_dev->DFQ(vtime) = min_vtime;
    list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
      if(stask->DFQ(vtime) < sched_dev->DFQ(vtime)) {
        neon_report("DFQ : process %-6d : vtime %-15lu <  dev vtime %-15lu "
                    "---> ___MOVED fwd to match",
                    stask->pid, stask->DFQ(vtime), sched_dev->DFQ(vtime));
        stask->DFQ(vtime) = sched_dev->DFQ(vtime);
      } else {
        neon_report("DFQ : process %-6d : vtime %-15lu >= dev vtime %-15lu ---> "
                    "NOT_MOVED fwd",
                    stask->pid, stask->DFQ(vtime), sched_dev->DFQ(vtime));
      }
    }
  }

 just_decide:
  sched_dev->DFQ(active) = activity;

  list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
    // update vtime, decide to block advanced tasks if
    // the whole system is not idle
    if(stask->DFQ(vtime) > (sched_dev->DFQ(vtime) + epoch_dt)) {
      stask->DFQ(held_back) = 1;
    }
    else
      stask->DFQ(held_back) = 0;
    // reset/task counters
    atomic64_set(&stask->DFQ(exe_dt_sampled), 0);
    atomic64_set(&stask->DFQ(ntimes_run), 0);
    neon_report("DFQ : did %d : pid %d : vtp %ld : vtd %ld + epoch_dt %d : "
                "held_back %d : dev-activity %d : update_vtime",
                sched_dev->id, stask->pid, stask->DFQ(vtime),
                sched_dev->DFQ(vtime), epoch_dt, stask->DFQ(held_back),
                sched_dev->DFQ(active));
  }

  return;
}

/****************************************************************************/
// update_now
/****************************************************************************/
// Pick task to sample (if any) and update all tasks virtual times
// CAREFUL : sched_dev write lock held
static void
update_now(sched_dev_t *sched_dev)
{
  sched_task_t *stask        = NULL;
  sched_work_t *swork        = NULL;

  update_vtimes(sched_dev);

  // unblock those who should be unblocked
  list_for_each_entry(stask, &sched_dev->stask_list.entry, entry) {
      neon_report("DFQ : did %d : pid %d : held-back %d : sem %d : "
                  "unblock all not held back", sched_dev->id, stask->pid,
                  stask->DFQ(held_back), stask->DFQ(sem_count));
      if(stask->DFQ(sem_count) < 0 && stask->DFQ(held_back) == 0) {
        stask->DFQ(sem_count)++;
        up(&stask->DFQ(sem));
      } else if (stask->DFQ(held_back)) {
	list_for_each_entry(swork, &stask->sworks_list.entry, entry) {
	  if(swork->DFQ(engage) == 0) {
            swork->DFQ(engage) = 1;
            neon_track_restart(1, swork->neon_work->ir);
            neon_report("DFQ : did %d : cid %d : pid %d : re_-engaged",
                        sched_dev->id, swork->id, swork->pid);
          } else
            neon_report("DFQ : did %d : cid %d : pid %d : was-engaged",
                        sched_dev->id, swork->id, swork->pid);

	}
	neon_report("DFQ : did %d : pid %d : held-back %d : sem %d : "
		    "suspended", sched_dev->id, stask->pid,
		    stask->DFQ(held_back), stask->DFQ(sem_count));
      }
  }

  neon_report("DFQ : %s : did %d (everybody will proceed)",
              season_name[sched_dev->DFQ(season)],
              sched_dev->id);
}

/****************************************************************************/
// event_season_timer_callback
/****************************************************************************/
// alarm signifying standard season or event period transition
static enum hrtimer_restart
event_season_timer_callback(struct hrtimer *timer)
{
  event_dev_t *event_dev = container_of(timer, event_dev_t,
                                              season_timer);
  policy_dev_t *policy_dev    = (policy_dev_t *) event_dev;
  sched_dev_t  *sched_dev     = container_of(policy_dev, sched_dev_t, ps);

#ifdef NEON_DEBUG_LEVEL_3
  if(sched_dev->id == NEON_MAIN_GPU_DID) {
    struct timespec now_ts      = { 0 };
    unsigned long   ts          = 0;
    getnstimeofday(&now_ts);
    ts = (unsigned long) (timespec_to_ns(&now_ts) / NSEC_PER_USEC);
    neon_report("DFQ : did %d : nctx %d : alarm timer callback @ %ld",
                sched_dev->id, atomic_read(&neon_global.ctx_live), ts);
  }
#endif // NEON_DEBUG_LEVEL_3

  if(atomic_read(&neon_global.ctx_live) > 0) {
    read_lock(&sched_dev->lock);
    atomic_set(&event_dev->action, 1);
    wake_up_interruptible(&neon_kthread_event_wait_queue);
    read_unlock(&sched_dev->lock);
  }

  return HRTIMER_NORESTART;
}

/**************************************************************************/
// init_event
/**************************************************************************/
// initialize DFQ specific structs
static int
init_event(void)
{
  unsigned int i = 0;

  for(i = 0; i < neon_global.ndev; i++) {
    sched_dev_t *sched_dev = &sched_dev_array[i];
#ifdef NEON_EVENT_COMP0_ONLY
    if(i > 0)
      break;
#endif // NEON_EVENT_COMP0_ONLY
    sched_dev->DFQ(season) = DFQ_EVENT_FREERUN;
    atomic_set(&sched_dev->DFQ(action), 0);
    hrtimer_init(&sched_dev->DFQ(season_timer), CLOCK_MONOTONIC,
                 HRTIMER_MODE_REL);
    sched_dev->DFQ(season_timer).function = &event_season_timer_callback;
    sched_dev->DFQ(chan_to_task) = (sched_task_t **) kzalloc(neon_global.dev[i].nchan * sizeof(sched_task_t *), GFP_KERNEL);
  }

  loga_init();

#if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG)
  make_debugfs_entries();
#endif /* LOGA_TRACE || LOGA_TRACE_DEBUG */
  
  neon_info("DFQ : init");

  return 0;
}

/**************************************************************************/
// fini_event
/**************************************************************************/
// finalize and destroy DFQ-specific structs
static void
fini_event(void)
{
  unsigned int i = 0;

  loga_fini();

  // cancel any live season timers; reset(0) must have already stopped them
  // but force a stop otherwise
  for(i = 0; i < neon_global.ndev; i++) {
    sched_dev_t *sched_dev = &sched_dev_array[i];
#ifdef NEON_EVENT_COMP0_ONLY
    if(i > 0)
      break;
#endif // NEON_EVENT_COMP0_ONLY
    atomic_set(&sched_dev->DFQ(action), 0);
    if(hrtimer_try_to_cancel(&sched_dev->DFQ(season_timer)) != 0)
      neon_error("%s : did %d : Event timer was busy at fini",
                 __func__, i);

    kfree(sched_dev->DFQ(chan_to_task));
  }

#if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG)
  destroy_debugfs_entries();
#endif /* LOGA_TRACE || LOGA_TRACE_DEBUG */
  
  neon_info("DFQ : fini");

  return;
}

/**************************************************************************/
// reset_event
/**************************************************************************/
// Reset Event-based Fair Queuing scheduling structs (checkpoint)
// THIS FUNCTION WILL MOST LIKELY NOT WORK AS-IS!!!
static void
reset_event(unsigned int nctx)
{
  unsigned int i = 0 ;

#if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG)
  tb_pointer = 0;
#endif /* LOGA_TRACE || LOGA_TRACE_DEBUG */
  
  if(nctx == 1) {
    event_T = _event_T_;
    event_X = _event_X_;

    if(event_T < NEON_POLLING_T_MIN) {
      neon_warning("Adjusting event T %u to implicit min = "
                   "min_polling %d T",
                   event_T, NEON_POLLING_T_MIN);
      event_T = NEON_POLLING_T_MIN;
    }
    if(event_T > NEON_EVENT_T_MAX) {
      neon_warning("Adjusting event T %u to max default %d T",
                   event_T, NEON_EVENT_T_MAX);
      event_T = NEON_EVENT_T_MAX;
    }
    if(event_X == 0) {
      neon_warning("Adjusting free-run to default %d*event_T",
                   event_X, NEON_EVENT_X_DEFAULT);
      event_X = NEON_EVENT_X_DEFAULT;
    }
    event_interval = ktime_set(0, event_T * NSEC_PER_MSEC);

    for(i = 0; i < neon_global.ndev; i++) {
      sched_dev_t *sched_dev = &sched_dev_array[i];
#ifdef NEON_EVENT_COMP0_ONLY
      if(i > 0)
        break;
#endif // NEON_EVENT_COMP0_ONLY
      sched_dev->DFQ(season) = DFQ_EVENT_FREERUN;
      sched_dev->DFQ(vtime) = 0;
      sched_dev->DFQ(update_ts) = 0;
      atomic_set(&sched_dev->DFQ(action), 0);
    }
    neon_info("DFQ : Event reset; (re)start with T=%d mSec",
              event_T);
  }
  if(nctx == 0) {
    for(i = 0; i < neon_global.ndev; i++) {
      sched_dev_t *sched_dev = &sched_dev_array[i];
#ifdef NEON_EVENT_COMP0_ONLY
      if(i > 0)
        break;
#endif // NEON_EVENT_COMP0_ONLY
      // TODO: try to include sched_dev->DFQ(sampled_task) != NULL ||
      // in the check for status ? currently update_vtimes does
      // this and if it fails to run, i know reset won't happen
      if(atomic_cmpxchg(&sched_dev->DFQ(action), 1, 0) == 0) {
        //      if(atomic_read(&sched_dev->DFQ(action)) != 0) {
        // unclean status at nctx == 0 should be impossible
        // if work-complete/stop worked as they should have been handled
        neon_warning("%s : did %d : unclean status @ nctx == 0",
                     __func__, sched_dev->id);
        //        BUG();
        return;
      }
    }
    neon_info("DFQ : Event reset; stop");
  }

  neon_info("DFQ : (re)set");

  return;
}

/**************************************************************************/
// create_event
/**************************************************************************/
// build new GPU sched-task event scheduler entries
static int
create_event(sched_task_t *sched_task)
{
  // policy-specific struct initializer
  memset(&sched_task->ps.evnt, 0, sizeof(event_task_t));
  sema_init(&sched_task->DFQ(sem), 0);

  INIT_LIST_HEAD(&sched_task->sworks_list.entry);

#if defined(LOGA_TRACE) || defined(LOGA_TRACE_DEBUG)
  tb_pointer = 0;
#endif /* LOGA_TRACE || LOGA_TRACE_DEBUG */

  neon_debug("DFQ : pid %d : create sched-task", sched_task->pid);

  return 0;
}

/**************************************************************************/
// destroy_event
/**************************************************************************/
// destroy GPU sched-task event scheduler entries
// CAREFUL : sched-dev write lock held
static void
destroy_event(sched_task_t *sched_task)
{
  // safety-check; preceeding stop/complete must have cleaned up properly
  if(sched_task->DFQ(held_back) != 0) {
    neon_warning("%s : DFQ : pid %d : held back task @ destroy "
                 "unblocked @ destroy", __func__, sched_task->pid);
    if(sched_task->DFQ(sem_count) < 0) {
      sched_task->DFQ(sem_count)++;
      up(&sched_task->DFQ(sem));
    }
    // sched_task->DFQ(held_back) = 0;
  }

  neon_debug("DFQ - pid %d : destroy sched-task", sched_task->pid);

  return;
}

/**************************************************************************/
// start_event
/**************************************************************************/
// new work, channel occupied
// CAREFUL : sched-dev write lock held
static void
start_event(sched_dev_t  * const sched_dev,
               sched_work_t * const sched_work,
               sched_task_t * const sched_task)
{
  // incoming jobs start engaged; it is only upon a submit attempt
  // that task and system status may change
  ++sched_task->DFQ(occ_chans);
  ++sched_dev->DFQ(occ_chans);

#ifdef NEON_EVENT_COMP0_ONLY
  if(sched_task->DFQ(occ_chans) % 2 == 0 ||
     sched_dev->id != NEON_MAIN_GPU_DID) {
    sched_work->DFQ(heed) = 0;
    sched_work->DFQ(engage) = 0;
    goto just_start;
  }
#endif // NEON_EVENT_COMP0_ONLY

  sched_work->DFQ(heed) = 1;
  sched_work->DFQ(engage) = 0;
  if(sched_task->DFQ(mng_chans)++ == 0)
    sched_task->DFQ(vtime) = sched_dev->DFQ(vtime);

  sched_dev->DFQ(chan_to_task)[sched_work->id] = sched_task;
  list_add(&sched_work->entry, &sched_task->sworks_list.entry);

  // force a new event to occur
  hrtimer_try_to_cancel(&sched_dev->DFQ(season_timer));
  neon_report("%s : canceled timer, set wake up event", __func__);
  if(atomic_read(&neon_global.ctx_live) > 0) {
    atomic_set(&sched_dev->DFQ(action), 1);
    wake_up_interruptible(&neon_kthread_event_wait_queue);
  }

 just_start :

  if(sched_dev->id == NEON_MAIN_GPU_DID)
    neon_report("DFQ : %s : did %d : cid %d : pid %d : sem %d : "
                "heed %d : engage %d : "
                "mng_chan %d : dma %d : vtime %d : start",
                season_name[sched_dev->DFQ(season)],
                sched_dev->id, sched_work->id, sched_task->pid,
                sched_task->DFQ(sem_count),
                sched_work->DFQ(heed), sched_work->DFQ(engage),
                sched_task->DFQ(mng_chans),
                sched_work->DFQ(heed),  sched_task->DFQ(vtime));

  return;
}

/**************************************************************************/
// stop_event
/**************************************************************************/
// remove work from channel
// CAREFUL : sched-dev write lock held
static void
stop_event(sched_dev_t  * const sched_dev,
              sched_work_t * const sched_work,
              sched_task_t * const sched_task)
{
  --sched_task->DFQ(occ_chans);
  --sched_dev->DFQ(occ_chans);

  if(sched_work->DFQ(heed) != 0)
    --sched_task->DFQ(mng_chans);
  if (sched_task->DFQ(mng_chans) == 0)
    sched_dev->DFQ(season) = DFQ_EVENT_FREERUN;


  sched_dev->DFQ(chan_to_task)[sched_work->id] = NULL;
  if (sched_work->DFQ(heed))
    list_del(&sched_work->entry);

  if(sched_task->DFQ(occ_chans) != 0) {
    if(sched_dev->id == NEON_MAIN_GPU_DID)
      neon_report("DFQ : did %d : cid %d : heed %d : mng/occ %d/%d : "
                  "vtime %d : ignore-work : stop",
                  sched_dev->id, sched_work->id, sched_work->DFQ(heed),
                  sched_task->DFQ(mng_chans), sched_task->DFQ(occ_chans),
                  sched_task->DFQ(vtime));
    goto just_stop;
  } else {
    if(sched_dev->id == NEON_MAIN_GPU_DID)
      neon_report("DFQ : did %d : cid %d : heed %d : mng/occ %d/%d : "
                  "vtime %d : halt-work : stop",
                  sched_dev->id, sched_work->id, sched_work->DFQ(heed),
                  sched_task->DFQ(mng_chans), sched_task->DFQ(occ_chans),
                  sched_task->DFQ(vtime));
  }

  // if exiting task is blocked, for whatever reason, unblock it
  while(sched_task->DFQ(sem_count) < 0) {
    sched_task->DFQ(sem_count)++;
    up(&sched_task->DFQ(sem));
  }

  // completion notification must have preceded

  // if the task was being held back, mark as let go
  // (already unblocked a few lines back)
  sched_task->DFQ(held_back) = 0;

  // force a new event to occur
  hrtimer_try_to_cancel(&sched_dev->DFQ(season_timer));
  neon_report("%s : canceled timer, set wake up event", __func__);
  if(atomic_read(&neon_global.ctx_live) > 0) {
    atomic_set(&sched_dev->DFQ(action), 1);
    wake_up_interruptible(&neon_kthread_event_wait_queue);
  }

  if(sched_dev->id == NEON_MAIN_GPU_DID)
    neon_report("DFQ : did %d : cid %d : pid %d : sem %d : "
                "vtime %d : %s held back : stop",
                sched_dev->id, sched_work->id, sched_task->pid,
                sched_task->DFQ(sem_count), sched_task->DFQ(vtime),
                sched_task->DFQ(held_back) == 0 ? "not" : "was");

 just_stop:

  sched_work->DFQ(engage) = 0;

  return;
}

/**************************************************************************/
// submit_event
/**************************************************************************/
// submit a request for scheduling consideration under EVENT policy
// CAREFUL sched-dev write lock held
static void
submit_event(sched_dev_t  * const sched_dev,
                sched_work_t * const sched_work,
                sched_task_t * const sched_task)
{
  unsigned int  block       = 0;

  if(sched_work->DFQ(heed) == 0)
    goto just_submit;

  // notice that new tasks starting off at freerun get unobstructed
  // access to the device; make sure to bring them back under control
  // when freerun ends
  if(sched_task->DFQ(held_back) == 0) {
    block = 0;
    sched_work->DFQ(engage) = 0;
  } else {
    block = 1;
    sched_work->DFQ(engage) = 1;
  }

  neon_info("DFQ : %s : did %d : cid %d : pid %d : "
            "(added %ld [i2c %d]) : "
            "nrqst %ld (+1 on issue) : submit %s @ %ld",
            season_name[sched_dev->DFQ(season)],
            sched_dev->id, sched_work->id,
            sched_task->pid, atomic64_read(&sched_task->DFQ(exe_dt_sampled)),
            (test_bit(sched_work->id, sched_task->bmp_issue2comp) != 0),
            sched_task->DFQ(ntimes_run),
            block == 1 ? "WILL__BLOCK" : "WONT_BLOCK",
            (unsigned long) (timespec_to_ns(&sched_work->submit_ts)));

  if(block == 1) {
    // because as I realized update_ts != 0, subsequent request
    // that will block will consider issue-bit set
    clear_bit(sched_work->id, sched_task->bmp_issue2comp);
    sched_task->DFQ(sem_count)--;
    write_unlock_irqrestore(&sched_dev->lock, sched_dev->flags);
    // wait here
    down_interruptible(&sched_task->DFQ(sem));
    write_lock_irqsave(&sched_dev->lock, sched_dev->flags);
  }

 just_submit:
  neon_policy_issue(sched_dev, sched_work, sched_task, block);

  return;
}

/**************************************************************************/
// issue_event
/**************************************************************************/
// issue request to GPU for processing, scheduled EVENT
// CAREFUL : sched_dev write lock held
static void
issue_event(sched_dev_t  * const sched_dev,
               sched_work_t * const sched_work,
               sched_task_t * const sched_task,
               unsigned int had_blocked)
{

  if(sched_work->DFQ(heed) == 0)
    goto just_issue;

  // notice that new tasks starting off at freerun get unobstructed
  // access to the device; make sure to bring them back under control
  // when freerun ends
  if(sched_task->DFQ(held_back) == 0)
    sched_work->DFQ(engage) = 0;
  else
    sched_work->DFQ(engage) = 1;

  neon_info("DFQ : %s : did %d : cid %d : pid %d : "
            "engage %d : sem %d : issue... ",
            season_name[sched_dev->DFQ(season)],
            sched_dev->id, sched_work->id, sched_task->pid,
            sched_work->DFQ(engage),
            sched_task->DFQ(sem_count));

  neon_info("DFQ : held_back %d : refc 0x%x/0x%x :"
            "exe_dt %ld : nrqst %ld [i2c %d]: %s : ...issue",
            sched_task->DFQ(held_back),
            *((unsigned int *) sched_work->neon_work->refc_kvaddr),
            sched_work->neon_work->refc_target,
            atomic64_read(&sched_task->DFQ(exe_dt_sampled)),
            atomic64_read(&sched_task->DFQ(ntimes_run)),
            (test_bit(sched_work->id, sched_task->bmp_issue2comp) != 0),
            had_blocked == 1 ? "UN__BLOCKED" : "NOT_BLOCKED");

 just_issue:
  return;
}

/**************************************************************************/
// complete_event
/**************************************************************************/
// mark completion of GPU request
// CAREFUL : sched_dev write lock held
static void
complete_event(sched_dev_t  * const sched_dev,
                  sched_work_t * const sched_work,
                  sched_task_t * const sched_task)
{
  // completion events can (legally) arrive at freerun period
  // (the work issued immediately after re-engaging)
  if(sched_task->DFQ(held_back) == 0)
    sched_work->DFQ(engage) = 0;
  else
    sched_work->DFQ(engage) = 1;

  neon_report("DFQ : %s : did %d : cid %d : pid %d : eng %d : "
	      "held_back %d : refc 0x%x/0x%x : exe_dt = %ld : "
	      "nrqst %ld [i2c %d] : complete",
	      season_name[sched_dev->DFQ(season)],
	      sched_dev->id, sched_work->id, sched_task->pid,
	      sched_work->DFQ(engage), sched_task->DFQ(held_back),
	      *((unsigned int *) sched_work->neon_work->refc_kvaddr),
	      sched_work->neon_work->refc_target,
	      atomic64_read(&sched_task->DFQ(exe_dt_sampled)),
	      sched_task->DFQ(ntimes_run),
	      (test_bit(sched_work->id, sched_task->bmp_issue2comp) != 0));

  return;
}

/**************************************************************************/
// event_event
/**************************************************************************/
// asynchronous event handler
static void
event_event(void)
{
  unsigned int i = 0;

  for(i = 0; i < neon_global.ndev; i++) {
    sched_dev_t     *sched_dev     = &sched_dev_array[i];
    event_season_t   last_season   = DFQ_TASK_NOFSEASONS_EVENT;
    ktime_t          interval      = { .tv64 = 0 };
    u32              season_dt     = 0;

    if(atomic_cmpxchg(&sched_dev->DFQ(action), 1, 0) == 0)
      continue;

#ifdef NEON_EVENT_COMP0_ONLY
    if(i > 0)
      break;
#endif // NEON_EVENT_COMP0_ONLY

    write_lock_irqsave(&sched_dev->lock, sched_dev->flags);

    last_season = sched_dev->DFQ(season);

    neon_report("DFQ event_event: season %s : did %d : chans %d",
		season_name[last_season], sched_dev->id, sched_dev->DFQ(occ_chans));
    
    season_dt = sched_dev->DFQ(occ_chans) * event_T;

#ifndef LOGA_TRACE
    switch(last_season) {
    case DFQ_EVENT_FREERUN :
#endif /* LOGA_TRACE */
      if (sched_dev->DFQ(occ_chans)) {
	sched_dev->DFQ(season) = DFQ_EVENT_SAMPLING;
	wake_up_interruptible_all(&sampling_thread_queue);
	interval = ktime_set(0, season_dt * NSEC_PER_MSEC);
      }
#ifndef LOGA_TRACE
      break;
      
    case DFQ_EVENT_SAMPLING :
      sched_dev->DFQ(season) = DFQ_EVENT_FREERUN;

      write_unlock_irqrestore(&sched_dev->lock, sched_dev->flags);
      wait_event_interruptible(sampling_thread_awake_queue, atomic_read(&sampling_threads_awake) == 0);
      write_lock_irqsave(&sched_dev->lock, sched_dev->flags);

      update_now(sched_dev);
      interval = ktime_set(0, event_X * season_dt * NSEC_PER_MSEC);
      break;
    default :
      neon_error("Unknown season");
    }
#endif /* LOGA_TRACE */

    if(interval.tv64 != 0) {
      ktime_t next_in;
      hrtimer_cancel(&sched_dev->DFQ(season_timer));
      hrtimer_start(&sched_dev->DFQ(season_timer), interval,
		    HRTIMER_MODE_REL);
      next_in = hrtimer_expires_remaining(&sched_dev->DFQ(season_timer));
      neon_report("%s : canceled timer, restart, next expires in %ld",
		  __func__, next_in.tv64/1000);
    }

    write_unlock_irqrestore(&sched_dev->lock, sched_dev->flags);
  }

  return;
}

/**************************************************************************/
// reengage_map_event
/**************************************************************************/
// let event policy control disegnaging after faults
static int
reengage_map_event(const neon_map_t * const neon_map)
{
  sched_dev_t  *sched_dev  = NULL;
  sched_work_t *sched_work = NULL;
  unsigned int  did        = 0;
  unsigned int  cid        = 0;
  int           isreg      = 0;
  int           reengage   = 0;

  isreg = neon_hash_map_offset(neon_map->offset, &did, &cid);
  if(isreg != 0) {
    neon_error("%s : map 0x%x : dis-engage unnecessary, not index reg",
               __func__, neon_map->key);
    return 1;
  }

  sched_dev = &sched_dev_array[did];
  read_lock(&sched_dev->lock);
  sched_work = &sched_dev->swork_array[cid];
  reengage = (sched_work->DFQ(heed) != 0) && (sched_work->DFQ(engage) != 0);
  read_unlock(&sched_dev->lock);

  if(sched_work->DFQ(heed) != 0)
    neon_debug("did %d : cid %d : %s-engaged", did, cid,
               reengage == 0 ? "dis" : "___");

  return reengage;
}
