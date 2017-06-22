/* "Low-overhead GPU Accounting" master's thesis by Stanislav Spassov in 2014
 * See loga.c for more information
 */
#ifndef __LOGA_PHASE_STATS_H__
#define __LOGA_PHASE_STATS_H__

typedef unsigned long long ull_t;

/* This struct collects timestamps and other statistics for profiling
 * and proper time accounting purposes. The member variables are listed in#
 * chronological order.
 * Unions are used only for the sake of code readability through aliases
 */
typedef struct phase_stats_t_ {
  ull_t begin_poll_ns;
  ull_t num_pollphase_iters; // Number of iterations in this polling phase
  ull_t end_poll_ns;

  ull_t sleep_duration_ns;

} phase_stats_t;

#endif /* !defined(__LOGA_PHASE_STATS_H__) */
