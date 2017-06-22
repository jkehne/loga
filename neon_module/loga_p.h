/* "Low-overhead GPU Accounting" master's thesis by Stanislav Spassov in 2014
 * See loga.c for more information
 */
#ifndef __LOGA_P_H__
#define __LOGA_P_H__

#define NUM_CHANS 96

#define PFIFO (0x2000U)
#define PFIFO_CHAN_TABLE         (0x800000U)
#define PFIFO_ENGINE_STATUS      (0x644U)
#define PFIFO_CHAN_TABLE__STRIDE (8U)
#define PFIFO_CHAN_TABLE_CHAN    (0U)
#define PFIFO_CHAN_TABLE_STATE   (4U)

#define PGRAPH (0x400000U)
#define PGRAPH_DISPATCH (0x4000U)
#define PGRAPH_DISPATCH_CMD_ADDR (0x4U)
#define PGRAPH_CTXCTL (0x9000)
#define PGRAPH_CTXCTL_MEM_CHAN (0xa0c)

/******************************************
 *  CHAN_TABLE[x].STATE parsing functions *
 ******************************************/
static inline bool is_pending(u32 state)
{
  return  state & 0x1;
}

// Determine the currently running channel by polling the card about it.
// Returns a negative number if the card is idle.
static inline int get_running_cid(void __iomem *bar0) {

  // first, check if anyting is running on the GPU at all
  const u32 state = readl((void __iomem *)((u8 *)bar0 + PFIFO + PFIFO_ENGINE_STATUS));
  if (!is_pending(state))
    return -4242;

  const unsigned pgraph_ctxctl_reg_base = PGRAPH + PGRAPH_CTXCTL;
  const u32 ctxctl_mem_chan = readl((void __iomem *)((u8 *)bar0 + pgraph_ctxctl_reg_base + PGRAPH_CTXCTL_MEM_CHAN));
  int cid;

  // TODO: Cache the channel addresses or something, instead of polling over all of them each time!
  for(cid = 1; cid < NUM_CHANS; ++cid) {
    const unsigned fifo_cte_base = PFIFO_CHAN_TABLE + cid*PFIFO_CHAN_TABLE__STRIDE;
    const u32 fifo_cte_chan = readl((void __iomem *)((u8 *)bar0 + fifo_cte_base + PFIFO_CHAN_TABLE_CHAN));

    // Mask out the two most-significant bits (VALID, POLL_ENABLE)
    if((fifo_cte_chan ^ ctxctl_mem_chan) & 0x3fffffff) {
      continue; // No match yet
    } else { // The XOR produced zero, we have a match

      return cid;
    }
  }

  // we should never reach this
  return -666;
}
#endif /* !defined(__LOGA_P_H__) */
