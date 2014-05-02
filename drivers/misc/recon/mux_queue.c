/******************** (C) COPYRIGHT 2013 Recon Instruments ********************
*
* File Name          : risensqueue.c
* Authors            : Zeljko Kozomara
* Version            : V 1.0
* Date               : February 2013
* Description        : Implementation of Recon Sensor Queue
*
********************************************************************************

*
******************************************************************************/

// Kernel headers
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include <linux/kernel.h>

// our internal header
#include "muxprivate.h"   


/************** Sensor Queue Work Function **********************/ 
static void queue_work_function (struct work_struct* pws)
{
    sensworkqueue* pw = container_of((struct delayed_work*)pws, struct _sensworkqueue, work);
    struct list_head* iter = 0;  
    RI_SENSOR_HANDLE mask = 0x00;  

    MUX_QUEUE_LOCK (pw)

    // check if we are supposed to quit -- happens when queue is stopped/destroyed
    if (pw->exit == 1) goto done;
    
    // reschedule ourselves
    queue_delayed_work (pw->pwq, &pw->work, msecs_to_jiffies(pw->delay_ms)); 
  
    // iterate list of sensor nodes and invoke read functions. We allow non-active sensors
    // on started queue, as long as at least one sensor is active (Otherwise it is a bug!)
    list_for_each (iter, &(pw->nodelist) )
    {
       struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
       if (atomic_read(&(pn->enabled) ) )
       {
          NODE_LOCK(pn)

             if (pn->filledsize < pn->buffersize)
             {
                 int dataread = pn->cbkRead (
                    pn->drvcontext,
                    pn->databuffer + pn->filledsize, 
                    pn->buffersize - pn->filledsize);
              
                 if (dataread < 0)
                    printk(KERN_ERR "+++ Proxmux: Sensor Driver [0x%x] Read Error +++\n", pn->handle);
                 else
                 {
                    pn->filledsize += dataread;
                    mask |= pn->handle;
                 }
             }
          
          NODE_UNLOCK(pn)
       }
       
    }
   
    // if anything was read, signal main thread
    if (mask)
    {
       //RISENS_INFO("+++ %s: Queue [%s%d] Data Ready. Data Mask: [0x%x] +++", __FUNCTION__, CUSTOM_QUEUE_PREFIX, pw->delay_ms, mask);
       mux_signaldata(mask);
    }

done:
    MUX_QUEUE_UNLOCK (pw)
    return;
}


/* **************************************************************
 *   Queue Constructor: Caller should guarantee delay uniqueness
 * **************************************************************/
struct _sensworkqueue* mux_queue_create (u32 delay)
{
   // allocate queue memory
   char name[QUEUE_NAME_SIZE];
   struct _sensworkqueue* pthis = kzalloc(sizeof(struct _sensworkqueue), GFP_KERNEL);
   if (pthis == NULL) 
   {
      printk (KERN_ERR "+++ %s: Memory Allocation Error +++\n", __FUNCTION__);
      return pthis;
   }

   // We don't worry about
   // synchronization at this level -- we are just slave API
   sprintf(name, "%s%d", CUSTOM_QUEUE_PREFIX, delay);
   pthis->pwq = create_singlethread_workqueue (name);
   if (!pthis->pwq) goto err_alloc_workqueue;

   // init locking
   MUX_QUEUE_LOCK_INIT(pthis)

   // remember poll rate for this custom queue
   pthis->delay_ms = delay;

   // currently disabled 
   pthis->exit = 1;

   // initialize work. We will start it when 1st sensor gets enabled for this queue
   // (Otherwise queue would keep firing with no work, which is waste of power)
   INIT_DELAYED_WORK ( (struct delayed_work*)&(pthis->work), queue_work_function);

   // init head of link list for list of nodes in this queue
   INIT_LIST_HEAD (&(pthis->nodelist) );

RISENS_INFO("+++ %s: Created Custom Queue [%s] +++\n", __FUNCTION__, name);
   return pthis;

err_alloc_workqueue:
   printk (KERN_ERR "+++ %s: Failed to create custom MUX queue [%s] +++\n", __FUNCTION__, name);
   kfree (pthis);

   return 0;
}

/* **************************************************************
 *   Queue Destructor. Passed pointer is invalidated after this
 *   We make no checks here; if there are sensors on it, they are "lost"
 *   Work needs to be stoped with prior call to mux_queue_stop
 *
 * **************************************************************/
void mux_queue_destroy (struct _sensworkqueue* pthis)
{
   list_del (&(pthis->listhead) );
   destroy_workqueue (pthis->pwq);

   RISENS_INFO("+++ %s: Queue [%s%d] destroyed +++", PROXMUX_DRIVER_NAME, CUSTOM_QUEUE_PREFIX, pthis->delay_ms);
   kfree (pthis);
}


/* ***************************************************************
 * Start Queue: Caller is responsible for locking!
 * ***************************************************************/
void mux_queue_start (struct _sensworkqueue* pthis)
{  
   struct list_head* iter = 0;
   // check if the queue has already started
   if (pthis->exit == 0)
   {
      RISENS_INFO("+++ %s: Queue [%s%d] has already started! +++", __FUNCTION__, CUSTOM_QUEUE_PREFIX, pthis->delay_ms);
      return;
   }

   // each driver that is already enabled must be informed of queue sampling rate. If the driver
   // node is not currently active it will be informed via PROXMUX_IOCTL_SET_ENABLE
   list_for_each (iter, &(pthis->nodelist) )
   {
       struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
       if (atomic_read(&(pn->enabled) ) )
       {
          // if driver fails, log but continue
          if (pn->cbkActivate( pn->drvcontext, 1, pn->currentmode, pthis->delay_ms) != 0)
          {
              printk(KERN_ERR "+++ %s: Queue [%s%d]. Sensor: [0x%x] Driver Activate Error. +++\n",
                  __FUNCTION__, CUSTOM_QUEUE_PREFIX, pthis->delay_ms, pn->handle); 
          }
       }
   }
   // now start the queue
   pthis->exit = 0;
   queue_delayed_work (pthis->pwq, &pthis->work, msecs_to_jiffies(pthis->delay_ms)); // internal named Mux queue
 
   RISENS_INFO("+++ %s: Custom Queue [%s%d] has been started +++", __FUNCTION__, CUSTOM_QUEUE_PREFIX, pthis->delay_ms);

}

/* ***************************************************************
 * Stop Queue. Must complete synchronously
 * ***************************************************************/
void mux_queue_stop (struct _sensworkqueue* pthis)
{
   struct list_head* iter = 0;
   pthis->exit = 1;
   cancel_delayed_work(&(pthis->work) );
   //flush_workqueue (pthis->pwq);

   // inform each active driver node of old sampling rate, saved inside node itself
   list_for_each (iter, &(pthis->nodelist) )
   {
       struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
       if (atomic_read(&(pn->enabled) ) )
       {
          // if driver fails, log but continue
          if (pn->cbkActivate( pn->drvcontext, 1, pn->currentmode, pn->delay_ms) != 0)
          {
              printk(KERN_ERR "+++ %s: Queue [%s%d]. Sensor: [0x%x] Driver Activate Error. +++\n",
                  __FUNCTION__, CUSTOM_QUEUE_PREFIX, pthis->delay_ms, pn->handle); 
          }
       }
   }
   RISENS_INFO("+++ %s: Queue [%s%d] successfully stopped +++", PROXMUX_DRIVER_NAME, CUSTOM_QUEUE_PREFIX, pthis->delay_ms);
}


/* ****************************************************************
 * FindQueue: Finds queue that has passed reporting rate. It it does
 *            not exist, it will create it
 * ****************************************************************/
struct _sensworkqueue* mux_queue_get
(
   struct list_head* pqhead,        // link list of mux device queues
   unsigned int      delay_ms,      // reporting rate in miliseconds
   unsigned char     create         // create new queue if it doesn't exist flag
)
{
     struct list_head* iter = 0; 
     struct _sensworkqueue* pq = 0;

     list_for_each (iter, pqhead )
     {
        pq = list_entry(iter, struct _sensworkqueue, listhead);
        if (pq->delay_ms == delay_ms) return pq;
     }

     // if here not found, so we'll try to create it if specified so
RISENS_INFO("+++ %s: Requested Queue [%d] ms not found +++\n", __FUNCTION__, delay_ms);

     if (create == 0) return 0;
     pq = mux_queue_create(delay_ms);
     if (pq) list_add_tail (&(pq->listhead), pqhead);

     return pq;
}


// utilities that return total number of nodes and total number of active nodes
// We don't lock here: Caller will do it, depending if of what he is trying to do
void          mux_queue_nodes_count(struct _sensworkqueue* pthis, unsigned int* ptotal, unsigned int* pactive)
{
    struct list_head* iter = 0; 
    *ptotal = *pactive = 0;

    list_for_each (iter, &(pthis->nodelist) )
    {
        struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
        *pactive += atomic_read(&(pn->enabled) );
        (*ptotal)++;
    }

}

/* Helper to determine whether queue is empty (and thus can be stoped and destroyed */
u8  mux_queue_is_empty  (struct _sensworkqueue* pthis)
{
   unsigned int total = 0; unsigned int active = 0;
   mux_queue_nodes_count (pthis, &total, &active);
   return (total > 0 ? 0 : 1);
}


/* Helper to determine whether queue with passed mask and delay can start */
u8 mux_queue_can_start (RI_SENSOR_HANDLE mask, unsigned int delay_ms)
{
   u16 i = 0; u16 to = sizeof(RI_SENSOR_HANDLE) * 8;

   // queue reporting rate must be positive
   if (delay_ms < MINIMUM_DELAY)
   {
       printk(KERN_ERR "+++ %s -- Invalid Queue Reporting Rate [%d] +++\n", __FUNCTION__, delay_ms);
       return 0;
   }

   if (mask == 0)
   {
       printk(KERN_ERR "+++ %s -- Invalid Sensor Mask [0x%x] +++\n", __FUNCTION__, mask);
       return 0;
   }

   // validate mask: Sensors must be:
   //    -- registered
   //    -- not on some other queue
   //    -- have minimum rate faster than queue rate
   //
   for (i = 0; i < to; i++)
   {
      if (CHECK_BIT(mask, i ) ) 
      {
         struct _sensnode* pn = 0; struct _sensworkqueue* pq = 0;
         find_sensor_node (1 << (i), &pn, &pq);
               
         if (pn == 0)
         {
             printk(KERN_ERR "+++ %s -- Custom Queue [%s%d] can not be started because sensor [0x%x] has not been registered +++\n", __FUNCTION__, CUSTOM_QUEUE_PREFIX, delay_ms, 1 << (i) );

             return 0;
         }

         if (pn->min_delay > delay_ms)
         {
             printk(KERN_ERR "+++ %s -- Custom Queue [%s%d] can not be started because sensor [0x%x] can report at fastest rate of [%d] ms +++\n", __FUNCTION__, CUSTOM_QUEUE_PREFIX, delay_ms, 1 << (i), pn->min_delay);

             return 0;
         }

         if ((pq) && (pq->delay_ms != delay_ms) )
         {
             printk(KERN_ERR "+++ %s -- Custom Queue [%s%d] can not be started because sensor [0x%x] is reporting from different Custom Queue [%s%d] +++\n", __FUNCTION__, CUSTOM_QUEUE_PREFIX, delay_ms, 1 << (i), CUSTOM_QUEUE_PREFIX, pq->delay_ms );

             return 0;
         }
      }
  
   }   // for (i = 0; i < 16; i++

    return 1;   // parameters are valid
}


/* Diagnostics */
#if defined RISENS_DEBUG
void                   mux_queue_dump_status (struct _sensworkqueue* pthis)
{
    struct list_head* iter = 0; 
    unsigned int total = 0; unsigned int active = 0;

    RISENS_INFO("\n\n");
    RISENS_INFO("+++ Custom Sensor Queue [%s%d] +++\n", CUSTOM_QUEUE_PREFIX, pthis->delay_ms);
    RISENS_INFO("+++ ================================== +++\n");
    RISENS_INFO("+++ Rate: %d [ms], Stopped: %d +++\n", pthis->delay_ms, pthis->exit);

    mux_queue_nodes_count (pthis, &total, &active);
    RISENS_INFO("+++ Total Nodes: %d, Active: %d +++\n", total, active);
   
    list_for_each (iter, &(pthis->nodelist) )
    {
       sensor_node_dump_status(list_entry(iter, struct _sensnode, listhead) );
    }

    RISENS_INFO("\n\n");
}
#endif    // #if defined RISENS_DEBUG


