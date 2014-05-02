/******************** (C) COPYRIGHT 2013 Recon Instruments ********************
*
* File Name          : proxmux.c
* Authors            : Zeljko Kozomara
* Version            : V 1.0
* Date               : February 2013
* Description        : Virtual Sensor Multiplexer Module 
*
********************************************************************************

*
******************************************************************************/
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <asm-generic/poll.h>  
#include <linux/miscdevice.h>
#include <linux/sched.h>

#include <linux/wait.h>              // wait events

#include <linux/workqueue.h>         // workqueue stuff
#include <asm/uaccess.h>             // copy_to_user

#include "muxprivate.h"
#include "risensors_def.h"
/* MUX Driver Private Data */
typedef struct _risensdata
{
   // default workqueue. We are not using Kernel because we wake up often & don't want to clog system
   struct workqueue_struct*  pdefq;

   // list of polling sensor nodes on default queue
   struct list_head  nodelist;

   // link list of polling sensor nodes on managed custom queues
   struct list_head  queuelist;

   // link list of irq sensor nodes
   struct list_head  irqlist;

   // main thread wait queue (writers will set)
   wait_queue_head_t signal;
   RI_SENSOR_HANDLE  wake;

}risensdata;


/* global instance of MUX device */
risensdata*  gMUX = 0;


/************** Sensor Node Work Function (For nodes on Default queue) **********************/ 
static void node_work_function (struct work_struct* pws)
{
    sensnode* pn = 0;
    int dataread = 0;

    pn = container_of((struct delayed_work*)pws, struct _sensnode, work);

    // check if we are supposed to quit -- happens when queue is stopped/destroyed
    if (atomic_read (&(pn->enabled) ) == 0) 
      return;
    
    // reschedule node on default MUX queue first
    queue_delayed_work(gMUX->pdefq, &pn->work, msecs_to_jiffies(pn->delay_ms));       
    
    // read data from driver. We expect driver to obey granularity (i.e. X, Y, Z)
    // this completes synchronously inside lock
    NODE_LOCK(pn)
       if (pn->filledsize < pn->buffersize)
       {
          dataread = pn->cbkRead (
              pn->drvcontext,
              pn->databuffer + pn->filledsize, 
              pn->buffersize - pn->filledsize);

          if (dataread < 0)
             printk(KERN_ERR "+++ Proxmux: Sensor Driver [0x%x] Read Error +++\n", pn->handle);
          else
             pn->filledsize += dataread;
       }

    // if there is data, wake main thread which will signal to user space
    if (pn->filledsize > 0)
       mux_signaldata(pn->handle);

    NODE_UNLOCK(pn)
}


// 1st Phase MUX constructor
static risensdata* MUX_INIT(void)
{
   risensdata* p = kzalloc(sizeof(struct _risensdata), GFP_KERNEL);
   if (p == NULL) 
   {
      printk (KERN_ERR "+++ MUX_INIT: Memory Allocation Error +++\n");
      return p;
   }

   // initialize wait queue and flat for user waits
   init_waitqueue_head (&(p->signal) );
   p->wake = 0;

   // custom workqueue list
   INIT_LIST_HEAD(&(p->queuelist) );

   // list of sensor nodes on default queue
   INIT_LIST_HEAD(&(p->nodelist) );

   // IRQ node list
   INIT_LIST_HEAD(&(p->irqlist) );

   return p;   // OK
}



/* PROXMUX destructor -- for completeness. Likely never called */
static void MUX_RELEASE(risensdata** p)
{ 
   struct list_head* iter = 0;  

   // traverse custom queue list and destroy each one
   while( !list_empty(&( (*p)->queuelist)) )
   {
      struct _sensworkqueue* pwq = list_entry((*p)->queuelist.next, struct _sensworkqueue, listhead);
   
      MUX_QUEUE_LOCK(pwq)
         mux_queue_stop (pwq);
         while( !list_empty(&(pwq->nodelist)) )
         {
            struct _sensnode* pn = list_entry(pwq->nodelist.next, struct _sensnode, listhead);
            kfree (pn->databuffer); 
            list_del ( &(pn->listhead) );
            kfree(pn);
         }
      MUX_QUEUE_UNLOCK(pwq) 

      // now destroy queue structure as well
      mux_queue_destroy(pwq );
   } 

   // stop any work on default queue
   list_for_each (iter, &( (*p)->nodelist) )
   {
      struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
      cancel_delayed_work (&(pn->work) );
   }

   // Destroy global workqueue
   //flush_workqueue ( (*p)->pdefq);
   destroy_workqueue( (*p)->pdefq);

   // deallocate node list that were on global workqueue
   while( !list_empty(&( (*p)->nodelist)) )
   {
      struct _sensnode* pn = list_entry((*p)->nodelist.next, struct _sensnode, listhead);
      kfree (pn->databuffer); 
      list_del ( &(pn->listhead) );
      kfree(pn);
   }

   // stop irq sensors
   list_for_each (iter, &( (*p)->irqlist) )
   {
      enable_sensor_irq(list_entry(iter, struct _sensirqnode, listhead), 0);  
   }

   // deallocate irq list
   while (!list_empty( &((*p)->irqlist) ) )
   {
      struct _sensirqnode* pnq = list_entry((*p)->irqlist.next, struct _sensirqnode, listhead);
      kfree(pnq->databuffer);
      list_del(&(pnq->listhead));      // remove this entry from the list
      kfree(pnq);                      // free the node itself
   }

   // finally deallocate memory
   kfree((*p)); *p = 0;
}



/* IOCTL into  /dev/proxmux 
   Passed parameter is always user space struct _risensorcmd pointer (see "risensors_def.h")

*/
static long proxmux_ioctl
(
   struct file*  file,
   unsigned int  cmd, 
   unsigned long arg
)
{
   void __user* argp = (void __user*)arg;
   risensorcmd sc;
   struct _sensnode* pn = 0; struct _sensworkqueue* pwq = 0;
   struct _sensirqnode* pnq = 0;
 
   if (copy_from_user(&sc, argp, sizeof(struct _risensorcmd) ))
   {
      printk(KERN_ERR "+++ PROXMUX: %s -- copy_from_user Error +++\n", __FUNCTION__);
      return -EFAULT;
   }	

   find_sensor_node (sc.handle, &pn, &pwq);
   if (pn == 0) find_sensor_node_irq (sc.handle, &pnq);

   // validate command
   if ( (cmd == PROXMUX_IOCTL_SET_ENABLE) || (cmd == PROXMUX_IOCTL_SET_DELAY) ||
        (cmd == PROXMUX_IOCTL_GET_STATUS) || (cmd == PROXMUX_IOCTL_SET_MODE)  || (cmd == PROXMUX_IOCTL_SET_MIN_DELAY) )
   {
      // must have node
      if ( (pn == 0) && (pnq == 0) )
      {
          printk(KERN_ERR "%s: Sensor [0x%x] Not Registered\n", __FUNCTION__, sc.handle);
          return -ENOENT;
      }

      // irq node can have only enable and status
      if ( (pnq) && ( (cmd != PROXMUX_IOCTL_SET_ENABLE) && (cmd != PROXMUX_IOCTL_GET_STATUS) ) ) 
      {
          RISENS_INFO("+++ %s: IRQ Sensor [0x%x] supports only enable/status query operations. (Requested: [0x%x]) +++\n", __FUNCTION__, sc.handle, cmd);
          return -ENXIO;
      }


   }

   
   // now act on command
   switch (cmd)
   {
      // enable / disable sensor. flag is in long1, mode is in short1
      case PROXMUX_IOCTL_SET_ENABLE:
      {
         return (pn) ? enable_sensor_poll (pn, pwq, sc.long1, sc.short1) : enable_sensor_irq (pnq, sc.long1);
      }
      break;

      // change polling rate for POLL based sensors
      // (see implementation function for behavior)
      case PROXMUX_IOCTL_SET_DELAY:
      {           
          return set_sensor_reporting_rate (pn, pwq, sc.long1);
      }
      break;

      // change minimum polling rate for POLL based sensors
      case PROXMUX_IOCTL_SET_MIN_DELAY:
      {
          return set_sensor_min_reporting_rate (pn, pwq, sc.long1);
      }
      break;

      // sensor status: Enabled/disabled, Reporting Rate, Min Reporting Rate
      // TODO: If we need, and how to propagate Queue Afinity (i.e. new IOCTL -- GET_QUEUE_STATUS)
      case PROXMUX_IOCTL_GET_STATUS:
      {
           if (pn)
           {
              sc.long1  = (pwq) ? pwq->delay_ms : pn->delay_ms;
	           sc.long2  = pn->min_delay;
              sc.short1 = atomic_read(&(pn->enabled) );
              sc.short2 = pn->buffersize;
              sc.short3 = pn->modemask;
              sc.short4 = pn->currentmode;
           }
           else
           {
              sc.long1  = sc.long2 = 0;    // no delay params for IRQ sensors
              sc.short1 = pnq->enabled;
              sc.short2 = pnq->buffersize;
              sc.short3 = sc.short4 = 0;
           }

           // write the info back to user space
           if (copy_to_user(argp, &sc, sizeof(struct _risensorcmd) ) )
           {
               printk(KERN_ERR "PROXMUX: %s -- copy_to_user Error +++\n", __FUNCTION__);
               return -EFAULT;
           }
           return 0;
      }
      break;

      // board configuration. Registered Sensors
      case PROXMUX_IOCTL_GET_CONFIG:
      {
          // use internal helper to retrieve full bitmask of registered sensors
          // param has mask, extra has total buffer size!
          get_board_config(&sc);

          // write the info back to user space
          if (copy_to_user(argp, &sc, sizeof(struct _risensorcmd) ) )
          {
             printk(KERN_ERR "+++ %s: %s -- copy_to_user Error +++\n", PROXMUX_DRIVER_NAME, __FUNCTION__);
             return -EFAULT;
          }
          return 0;
      }
   
      // Queue Start
      case PROXMUX_IOCTL_START_QUEUE:
      {
          return start_sensor_queue (sc.handle, sc.long1);
      }
      break;

      // Queue Stop.
      // Default Kernel Queue can never be stopped & is never seen by user
      // Currently sensors do not remember previous affinity; stoping the queue they are
      // currently reporting from always moves them to default queue. It is important to note
      // start/stop of queue has nothing to do with sensor enable/disable!
      case PROXMUX_IOCTL_STOP_QUEUE:
      {
          struct _sensworkqueue* pq = mux_queue_get(&(gMUX->queuelist), sc.long1, 0);
          if (pq == 0)
          {
             printk(KERN_ERR "%s: Requested Custom Queue [%s:%d] does not exist \n", __FUNCTION__, CUSTOM_QUEUE_PREFIX, sc.long1);
             return -ENXIO;
          }

          MUX_QUEUE_LOCK(pq)
             mux_queue_stop(pq);
             empty_custom_queue(pq);  
          MUX_QUEUE_UNLOCK(pq);

          mux_queue_destroy(pq);

          RISENS_INFO("+++ %s Custom Queue [%s:%d] has been stopped. All sensors are transferred to Default Queue +++",
              __FUNCTION__, CUSTOM_QUEUE_PREFIX, sc.long1);

          return 0;        
      }
      break;

      //
      // set sensor reporting mode: 
      // If sensor is not active, will remember it (if supported). If sensor is currently active, will
      // actually call into driver (regardless of queue affinity). Protocol:
      //  --- handle: sensor id  (already retrieved on top)
      //  --- short1: reporting mode
      //
      case PROXMUX_IOCTL_SET_MODE:
      {
         if (sc.short1 == pn->currentmode) return 0;   // already set to this mode
      
         if ( (sc.short1 & pn->modemask) != sc.short1 )
         {
            printk(KERN_ERR "+++ Sensor: [0x%x]. Invalid Reporting Mode [0x%x] requested. Supported: [0x%x] +++\n",
               pn->handle, sc.short1, pn->modemask);

            return -ENXIO;
         }

         if (atomic_read(&(pn->enabled) ) == 0)
         {
            pn->currentmode = sc.short1;   // remember for next activate
            return 0;
         }
         else   // have to flip into driver which is currently reporting
            return enable_sensor_poll (pn, pwq, 1, sc.short1);
      }
      break;

      default:

        printk(KERN_ERR "%s: %s Unrecognized IOCTL (0x%x) +++\n", PROXMUX_DRIVER_NAME, __FUNCTION__, cmd);
        return -EINVAL;
   }

   // never reached
   return 0;
}

/* Poll -- waits on FIFO event queue */
static unsigned int proxmux_poll
(
   struct file*               file, 
   struct poll_table_struct*  pt
)
{
   // wait till we are signalled from workqueue nodes
   wait_event_interruptible (gMUX->signal,  (gMUX->wake > 0) );

   RISENS_INFO_V("+++ %s: User Data signalled. Mask: [0x%x] +++\n", __FUNCTION__, gMUX->wake);

   return POLLIN;
}

/* Read: We have been awaken by id of queue that signaled data (Queue ID is sensor bitmask)
         Find the queue and copy sensor data to user space */
static ssize_t proxmux_read
(
   struct file* filp,
   char __user* buf,         // pointer to user space allocated buffer
   size_t       count,       // number of bytes from user space
   loff_t*      f_pos
)
{
    struct list_head* iter = 0;
    struct list_head* queueiter = 0;
    struct list_head* irqiter = 0;

    size_t copied = 0;

    RISENS_INFO_V("+++ %s BEGIN: Available mask: [0x%x] +++", __FUNCTION__, gMUX->wake);

    // check irq nodes first
    list_for_each (irqiter, &(gMUX->irqlist) )
    {
       struct _sensirqnode* pnq = list_entry(irqiter, struct _sensirqnode, listhead);
       int err = 0;

       NODE_LOCK (pnq)
       //if (pn->filledsize > 0)
       if ( (gMUX->wake & pnq->handle) == pnq->handle)
       {
          // check if there is enough room. We want whole buffer
          if ( (count - copied) < pnq->buffersize )
          {
                  RISENS_INFO("+++ %s: Unable to transfer all data to User Space. Handle: [0x%x], User Size: %d, Copied: %d, Pending: %d +++\n", __FUNCTION__, pnq->handle, count, copied, pnq->buffersize);

               err = ENOSPC;
          }
          else
          {
              err = sensor_node_read (pnq->databuffer, pnq->buffersize, buf + copied, &copied);
          }
       }

       if (err == 0)
          gMUX->wake &= ~pnq->handle;
   
       NODE_UNLOCK (pnq)
    }

    // check default queue
    list_for_each (iter, &(gMUX->nodelist) )
    {
       struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
       int err = 0;

       NODE_LOCK (pn)
       //if (pn->filledsize > 0)
       if ( (gMUX->wake & pn->handle) == pn->handle)
       {
          // check if there is enough room. We want whole buffer
          if ( (count - copied) < ( pn->filledsize) )
          {
                  RISENS_INFO("+++ %s: Unable to transfer all data to User Space. Handle: [0x%x], User Size: %d, Copied: %d, Pending: %d +++\n", __FUNCTION__, pn->handle, count, copied, pn->filledsize);

               err = ENOSPC;
          }
          else
             err = sensor_node_read (pn->databuffer, pn->filledsize, buf + copied, &copied);
       }

       if (err == 0)
       {
          pn->filledsize = 0;
          gMUX->wake &= ~pn->handle;
       }

       NODE_UNLOCK (pn)
    }

    // now check each queue
    list_for_each (queueiter, &(gMUX->queuelist) )
    {
       struct _sensworkqueue* pq = list_entry(queueiter, struct _sensworkqueue, listhead);
       list_for_each (iter, &(pq->nodelist) )
       {
          struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
          int err = 0;

          NODE_LOCK (pn)

          //if (pn->filledsize > 0)
          if ( (gMUX->wake & pn->handle) == pn->handle)
          {
              // check if there is enough room. We want whole buffer
              if ( (count - copied) < (pn->filledsize) )
              {
                  RISENS_INFO("+++ %s: Unable to transfer all data to User Space. Handle: [0x%x], User Size: %d, Copied: %d, Pending: %d +++\n", __FUNCTION__, pn->handle, count, copied, pn->filledsize);

                  err = ENOSPC;
              }
              else
                 err = sensor_node_read (pn->databuffer, pn->filledsize, buf + copied, &copied);
          }

          if (err == 0)
          {
             pn->filledsize = 0;
             gMUX->wake &= ~pn->handle;
          }

          NODE_UNLOCK (pn)
  

       }   // for each node in a queue
    }  // for each queue


    // return what we filled
    RISENS_INFO_V("+++ %s END: Transfered [%d] bytes of data to User Space. Remaining mask: [0x%x] +++",
      __FUNCTION__, copied, gMUX->wake);

    return copied;
}



/* File operations on proxmux device */
static const struct file_operations proxmux_fops =
{
        .owner                  = THIS_MODULE,
        .poll                   = proxmux_poll,
        .read                   = proxmux_read,
#if defined RISENS_DEBUG
        .write                  = proxmux_write,
#endif
        .unlocked_ioctl         = proxmux_ioctl,
};
static struct miscdevice proxmux_device =
{
        .minor = MISC_DYNAMIC_MINOR,
        .name  = PROXMUX_DRIVER_NAME,
        .fops  = &proxmux_fops,
};


static int __init proxmux_init(void)
{
   int err = 0;

   // allocate MUX data - 1st Phase constructor
   gMUX = MUX_INIT();
   if (!gMUX)
   {
      err = -ENOMEM;
      goto err_memory;
   }

   // allocate default queue. All sensors will be placed there during registration.
   // HAL layer will have a chance to do initial configuration based on sensors.conf
   gMUX->pdefq = create_singlethread_workqueue (DEFAULT_QUEUE_NAME);
   if (!gMUX->pdefq)
   {
      err = -ENXIO;
      printk (KERN_ERR "+++ Failed to create default MUX queue +++\n"); 

      goto err_queue;
   }

   // Register device
   err = misc_register(&proxmux_device);
   if (err < 0)
   {
      printk (KERN_ERR "+++ Failed to register %s Device (%d) +++\n", PROXMUX_DRIVER_NAME, err);
      goto err_register;
   }

   return 0;

   err_register:
       destroy_workqueue(gMUX->pdefq);
       gMUX->pdefq = 0;

   err_queue:
       kfree(gMUX);
       gMUX = 0;

   err_memory:
      return err;

}

static void __exit proxmux_exit(void)
{
    RISENS_INFO("+++ %s +++\n", __FUNCTION__);

    // deregister device first
    misc_deregister(&proxmux_device);

    // call MUX destructor. This will stop default and all custom queues, with associated work
    MUX_RELEASE(&gMUX);

    return;
}


/* Registration export: Irq mode */
RI_SENSOR_STATUS risensor_register_irq 
(
    RI_SENSOR_HANDLE handle,        // sensor id
    const char*      name,          // sensor name
    void*            context,       // context pointer, passed back during activate callback
    unsigned int     datasize,      // interrupt event data size
    PFNSENS_ACTIVATE cbkActivate    // activate callback -- optional (Zero is ok)
)
{
    sensirqnode* pnq = 0;
    sensnode* pn = 0;
    struct _sensworkqueue* pwq = 0;

    RISENS_INFO("+++ %s: Registration for Sensor 0x%x [%s] Called. Mode: IRQ +++\n", __FUNCTION__, handle, name);

    // validate passed data: We require non-zero handle, name and buffer size
    // (Activate is OK zero --- drivers which support concept of turning power off immediately after single read
    if ( (handle == 0) || (name == 0)  || (datasize == 0) )
    {
       printk(KERN_ERR "+++ %s: Registration for Sensor 0x%x (%s) Failed -- Invalid Arguments +++\n",
          __FUNCTION__, handle, name);

       return -EINVAL;
    }

    // check duplicate in POLL database
    find_sensor_node (handle, &pn, &pwq);
    if (pn)
    {
       printk ("+++ %s:: Duplicate registration for Sensor 0x%x [%s] -- second attempt rejected +++\n",
          __FUNCTION__, handle, name);

       return -EEXIST;
    }

    // check duplicate in IRQ Database
    find_sensor_node_irq (handle, &pnq);
    if (pnq)
    {
       printk ("+++ %s:: Duplicate registration for Sensor 0x%x [%s] -- second attempt rejected +++\n",
          __FUNCTION__, handle, name);

       return -EEXIST;
    }

    // Registration Valid. Allocate new irq node
    pnq =  kzalloc(sizeof(struct _sensirqnode), GFP_KERNEL);
    if (pnq)
    {
       // allocate data buffer. This includes event data + event header!
       pnq->databuffer = kzalloc(datasize, GFP_KERNEL);
       if (!(pnq->databuffer) )
       {
           kfree(pnq); pnq = 0;
       }

       pnq->buffersize = datasize;   // single interrupt data buffer

    }

    if (!pnq) goto err_alloc_node;
    
    // init head of link list
    INIT_LIST_HEAD (&(pnq->listhead) );

    // initialize data buffer lock
    NODE_LOCK_INIT(pnq)
    
    // fill in the rest of data fields
    pnq->handle      = handle;
    pnq->name        = name;
  
    pnq->cbkActivate = cbkActivate;
    pnq->drvcontext  = context;

    pnq->enabled = 0;                 // currently inactive

    // finally add it to the list
    list_add_tail ( &(pnq->listhead), &(gMUX->irqlist) );

    // and we are done!
    RISENS_INFO("+++ %s: Sensor 0x%x [%s] successfully registered in IRQ mode! +++\n", __FUNCTION__, handle, name);

    return 0;

err_alloc_node:
  printk(KERN_ERR "+++ %s: Registration for Sensor 0x%x (%s) Failed -- Memory Allocation Error +++\n",
          __FUNCTION__, handle, name);

  return -ENOMEM;
}



/* Registration export: Poll mode */
RI_SENSOR_STATUS risensor_register
(
    RI_SENSOR_HANDLE handle,        // sensor id
    const char*      name,          // sensor name
    void*            context,       // context pointer, passed back during callbacks (driver private data structure usually)
    RI_SENSOR_MODE   modemask,      // bitmask of modes this sensor supports (FIFO, CR or both)
    RI_SENSOR_MODE   currentmode,   // current mode this driver is on
    unsigned int     datasize,      // max data size (# of bytes) for this sensor
    PFNSENS_ACTIVATE cbkActivate,   // activate callback -- required.
    PFNSENS_READ     cbkRead        // read callback -- required
)
{
     struct _sensnode* pn = 0; struct _sensworkqueue* pwq = 0;

     RISENS_INFO("+++ %s: Registration for Sensor 0x%x [%s] Called. Mode: POLL +++\n", __FUNCTION__, handle, name);

    // parameter check
    // validate passed data: We require non-zero handle, name, read and activate callbacks non-zero
    // fifosize must be > 0, modemask must be valid
    if ( (handle == 0) || (name == 0) || (cbkRead == 0) || (cbkActivate == 0) || (datasize == 0) )
       goto err_invalid_registration;

    // must support either FIFO or CR reporting mode
    if ( ( (modemask & RI_SENSOR_MODE_FIFO) != RI_SENSOR_MODE_FIFO) && ( (modemask & RI_SENSOR_MODE_CR) != RI_SENSOR_MODE_CR) )
       goto err_invalid_registration;

    // sanity check: Search for double registration
    find_sensor_node (handle, &pn, &pwq);
    if (pn)
       goto err_duplicate_registration;
    else
    {
       struct _sensirqnode* pnq = 0;
       find_sensor_node_irq (handle, &pnq);
       if (pnq) goto err_duplicate_registration;
    }

    // allocate memory for this sensor node and his data buffer
    pn =  kzalloc(sizeof(struct _sensnode), GFP_KERNEL);
    if (pn)
    {
       pn->databuffer = kzalloc(datasize, GFP_KERNEL);
       if (!(pn->databuffer) )
       {
           kfree(pn); pn = 0;
       }
    }

    if (!pn) goto err_alloc_node;

    // initialize rest of data members
    pn->handle      = handle;
    pn->name        = name;
    pn->cbkActivate = cbkActivate;
    pn->cbkRead     = cbkRead;

    pn->buffersize  = datasize; 
    pn->filledsize  = 0;              // initially empty

    pn->drvcontext  = context;        // driver context

    atomic_set (&(pn->enabled), 0);   // sensor is initially disabled
    pn->delay_ms    = DEFAULT_DELAY;  // initial firing rate when in Kernel queue
    pn->min_delay   = MINIMUM_DELAY;  // default minimum delay; if configured, value will be passed from user at boot time

    pn->modemask = modemask;          // bitmask of supported reporting modes  
    pn->currentmode = currentmode;    // current mode

    // initialize data buffer lock
    NODE_LOCK_INIT(pn)

    // init head of link list as node is maintained as part of default or one of custom queues
    INIT_LIST_HEAD (&(pn->listhead) );

    // Initialize work function when node is in Default queue
    INIT_DELAYED_WORK ( (struct delayed_work*)&(pn->work), node_work_function);

    // add to default queue
    list_add_tail ( &(pn->listhead), &(gMUX->nodelist) );

    // Log what we did
    RISENS_INFO("+++ %s: Sensor 0x%x [%s] successfully registered! Enabled: [%d], Current mode: [0x%x], Current rate (ms): [%d]. Buffer Data Size: [%d] bytes +++\n",
       __FUNCTION__, handle, name, atomic_read(&(pn->enabled) ), pn->currentmode, pn->delay_ms, pn->buffersize );

    return 0;

err_invalid_registration:
     printk(KERN_ERR "+++ %s: Registration for Sensor 0x%x (%s) Failed -- Invalid Arguments +++\n",
          __FUNCTION__, handle, name);

     return -EINVAL;

err_duplicate_registration:
    printk(KERN_ERR "+++ %s: Duplicate registration for Sensor 0x%x (%s) -- 2nd registration rejected +++\n",
          __FUNCTION__, handle, name);

     return -EEXIST;

err_alloc_node:
  printk(KERN_ERR "+++ %s: Registration for Sensor 0x%x (%s) Failed -- Memory Allocation Error +++\n",
          __FUNCTION__, handle, name);

  return -ENOMEM;


}


/* Reverse callback for Interrupt sensors */
RI_SENSOR_STATUS risensor_irq_data (RI_SENSOR_HANDLE handle, unsigned char* databuffer, RI_DATA_SIZE buffersize)
{
   // sensor must be registered in irq mode
   sensirqnode* pnq = 0;

   find_sensor_node_irq (handle, &pnq);
   if (pnq == 0)
   {
      printk(KERN_WARNING "+++ %s: IRQ Data Signaled, but Sensor [0x%x] has not been registered +++\n", __FUNCTION__, handle);
      return -ENODEV;
   }

   // data must be what he registered with
   if (pnq->buffersize != buffersize)
   {
     printk(KERN_WARNING "+++ %s: IRQ Data Signaled for Sensor [0x%x], but data size (%d bytes) different from registered (%d bytes)  +++\n", __FUNCTION__, handle, buffersize, pnq->buffersize);
      return -EINVAL;
   }

   RISENS_INFO("+++ %s: Interrupt Event Signaled. Handle: [0x%x]. Sensor: %s +++\n",
     __FUNCTION__, handle, pnq->name);

   // save interrupt data in node buffer. We always overwrite
   // then signal so that POLL from user space wakes up
   NODE_LOCK(pnq)
      memcpy(pnq->databuffer, databuffer, buffersize);
      mux_signaldata(pnq->handle);
   NODE_UNLOCK(pnq)

   return 0;
}

/* Internal functionality to signal data ready from queue. */
void mux_signaldata(RI_SENSOR_HANDLE mask)
{
    gMUX->wake |= mask;
    wake_up_interruptible (&(gMUX->signal) ); 

   // RISENS_INFO("+++ %s: Data signalled. Data Mask: [0x%x] +++\n", __FUNCTION__, gMUX->wake);
}


/* Misc helpers */

void get_board_config (risensorcmd* pcmd)
{
    // we iterate Default and Custom queues & build bitmask, then tack-on IRQ nodes
    struct list_head* iter = 0;
    struct list_head* queueiter = 0;
    
    memset (pcmd, 0, sizeof(risensorcmd) );

    // default queue
    list_for_each (iter, &(gMUX->nodelist) )
    {
       struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
       pcmd->handle |= pn->handle;
       pcmd->long1 += pn->buffersize;
    }

    // now check each queue
    list_for_each (queueiter, &(gMUX->queuelist) )
    {
       struct _sensworkqueue* pq = list_entry(queueiter, struct _sensworkqueue, listhead);
       list_for_each (iter, &(pq->nodelist) )
       {
           struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
           pcmd->handle |= pn->handle;
           pcmd->long1 += pn->buffersize;
       }
    }

    // IRQ list
    list_for_each (iter, &( gMUX->irqlist) )
    {
        struct _sensirqnode* pnq = list_entry(iter, struct _sensirqnode, listhead);
        pcmd->handle |= pnq->handle;
        pcmd->long1 += pnq->buffersize;
    }

}

// looks for sensor node that matches passed handle
// Returns also workqueue pointer, if node is on one of custom queues
void find_sensor_node (RI_SENSOR_HANDLE handle, struct _sensnode** ppnode, struct _sensworkqueue** ppwq)
{
    struct list_head* iter = 0;
    struct list_head* queueiter = 0;

    // clear return data
    *ppnode = 0; *ppwq = 0;

    // check default queue first
    list_for_each (iter, &(gMUX->nodelist) )
    {
       struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
       if (pn->handle == handle)
       {
          *ppnode = pn;
          return;
       }
    }

    // now check each queue
    list_for_each (queueiter, &(gMUX->queuelist) )
    {
       struct _sensworkqueue* pq = list_entry(queueiter, struct _sensworkqueue, listhead);
       list_for_each (iter, &(pq->nodelist) )
       {
          struct _sensnode* pn = list_entry(iter, struct _sensnode, listhead);
          if (pn->handle == handle)
          {
             *ppnode = pn; *ppwq = pq;
             return;
          }
       }
    }

}


// finds the node for passed handle in irq list
void find_sensor_node_irq (RI_SENSOR_HANDLE handle, struct _sensirqnode** ppnq)
{
   struct list_head* iter = 0;
   *ppnq = 0;

   list_for_each (iter, &(gMUX->irqlist) )
   {
      struct _sensirqnode* pnq = list_entry(iter, struct _sensirqnode, listhead);
      if (pnq->handle == handle)
      {
          *ppnq = pnq;
          return;
      }
   }

}


/* Queue Start. "Primary Key" is reporting rate. Validation ensures that
     -- Queue is unique
     -- Sensors mask is not null (can not start empty custom queue)
     -- All sensors are poll, and NOT reporting from some other custom queue

     Important note; Starting of Custom queue is separate from sensor enabling;
        this merely groups sensors under same wake&report rate. During lifetime
        of the queue individual sensor rate can not be changed, but enable/disable
        (which goes standard Android route) can

*/
RI_SENSOR_STATUS start_sensor_queue (RI_SENSOR_HANDLE mask, unsigned int delay_ms)
{
   struct _sensworkqueue* pq = 0;
   u16 i = 0; u16 to = sizeof(RI_SENSOR_HANDLE) * 8;

   // validate if queue can start
   if (mux_queue_can_start(mask, delay_ms) == 0) 
      return -EINVAL;
   
   // now get the queue. If it doesn't exist, it will be created
   pq = mux_queue_get (&(gMUX->queuelist), delay_ms, 1);
   if (pq == 0) return -ENOSPC;

   // iterate mask & transfer nodes
   for (i = 0; i < to; i++)
   {
      if (CHECK_BIT(mask, i ) )
      {
          struct _sensnode* pn = 0; struct _sensworkqueue* pcq = 0;
          find_sensor_node (1 << (i), &pn, &pcq);

          // move node to new queue
          if (pcq == 0)  // default kernel queue
             transfer_sensor_node (pn, pq);
      }
   }

   // finally start the queue and we are done!
   MUX_QUEUE_LOCK(pq)
      mux_queue_start (pq);
   MUX_QUEUE_UNLOCK(pq)

   return 0;
}


// Transfer sensor node from default to custom queue
void transfer_sensor_node (struct _sensnode* pn, struct _sensworkqueue* pdestination)
{
   NODE_LOCK(pn)
      // must cancel his work function if sensor was enabled
      if ( (atomic_read(&(pn->enabled) ) == 1))
          cancel_delayed_work (&(pn->work) );
  
       list_del_init ( &(pn->listhead) );
       list_add_tail ( &(pn->listhead), &(pdestination->nodelist) );
      
   NODE_UNLOCK(pn)
}


/* Helper to empty custom queue. All sensors are transferred to Default Queue, 
   with internal properties (rate, mode, enabled) unmodified */
void empty_custom_queue (struct _sensworkqueue* pq)
{    
       while( !list_empty(&(pq->nodelist) ) )
       {
          struct _sensnode* pn = list_entry(pq->nodelist.next, struct _sensnode, listhead);

          NODE_LOCK(pn)
             list_del_init ( &(pn->listhead) );
             list_add_tail ( &(pn->listhead), &(gMUX->nodelist) ); 
             if (atomic_read(&(pn->enabled) ) )
             {
                queue_delayed_work(gMUX->pdefq, &pn->work, msecs_to_jiffies(pn->delay_ms));  
             }
          NODE_UNLOCK(pn)
       }

   RISENS_INFO("+++ %s: Custom Queue [%s%d] has been emptied +++\n", __FUNCTION__, CUSTOM_QUEUE_PREFIX, pq->delay_ms);
}

/*  
     Set reporting rate for Specific sensor. Behavior:

--- Sensor can belong to only one queue at a time ("category")
--- Rates for sensors on default queue can be changed at will (Android API behavior)
--- Sensors on custom queue are not mutable; thus if sensor is on custom queue rate can not be changed

*/
RI_SENSOR_STATUS set_sensor_reporting_rate (struct _sensnode* pn, struct _sensworkqueue* pwq, unsigned int delay_ms)
{
   RI_SENSOR_STATUS stat = 0;
   if (pwq != 0)
   {
      RISENS_INFO("+++ %s: Rate for Sensor [%s] can not be changed to [%d] ms, as it is on custom Kernel queue [%s%d] +++ \n",
        __FUNCTION__, pn->name, delay_ms, CUSTOM_QUEUE_PREFIX, pwq->delay_ms);
     
      return -EPERM;
   }

   // check for minimum rate
   if (delay_ms < pn->min_delay)
   {
      printk(KERN_INFO "+++ %s: Rate for Sensor [%s] can not be changed to [%d] ms, as it is less than fastest allowed rate. Setting to [%d] ms +++ \n",
        __FUNCTION__, pn->name, delay_ms, pn->min_delay);
     
      delay_ms=pn->min_delay;
   }

   NODE_LOCK(pn)
       // it is not enough just to flip poll rate internally; driver must be informed as well
        stat = pn->cbkActivate(
        pn->drvcontext,                    // driver context pointer
        atomic_read(&(pn->enabled) ),   // current status
        pn->currentmode,                   // current driver mode
        delay_ms);                         // new poll rate

        if (stat == 0)
            pn->delay_ms = delay_ms;   // flip internally as well
   NODE_UNLOCK(pn)

    return stat;
}

// Set minimum reporting rate for Specific POLL sensor; used by user HAL to pass value from SENSORS.CONF at boot
RI_SENSOR_STATUS set_sensor_min_reporting_rate (struct _sensnode* pn, struct _sensworkqueue* pwq, unsigned int min_delay)
{
   u32 current_rate = (pwq) ? pwq->delay_ms : pn->delay_ms;
   if ( (current_rate < min_delay) && (atomic_read(&(pn->enabled) ) ) > 0 )
   {
      printk(KERN_INFO "+++ %s: Minimum Rate for Sensor [%s] can not be changed to [%d] ms, as it is already reporting on faster rate [%d] ms +++ \n",
        __FUNCTION__, pn->name, min_delay, current_rate);

      return -EPERM;
   }

   NODE_LOCK(pn)
      pn->min_delay = min_delay;
      if (pn->delay_ms < pn->min_delay)
        pn->delay_ms = min_delay;
   NODE_UNLOCK(pn)

   return 0;
}


// helper to read data of sensor node. Could be a macro. Returns 0 on success, or error
int sensor_node_read (unsigned char* psrc, RI_DATA_SIZE srcsize, unsigned char* pdest, size_t* pcopied)
{
    int err = copy_to_user (pdest, psrc, srcsize );
    if (err)
    {
       printk(KERN_ERR "+++ %s:%s copy_to_user Error +++\n", PROXMUX_DRIVER_NAME, __FUNCTION__);
       goto done;
    }   

    (*pcopied) += srcsize;

done:
   return err;
}

// enable sensor node in POLL reporting mode. If pwq is NULL, node is on default queue
// We send other control info in the node (current rate + current mode)
RI_SENSOR_STATUS enable_sensor_poll (struct _sensnode* pn, struct _sensworkqueue* pwq, unsigned char flag, RI_SENSOR_MODE mode)
{
   RI_SENSOR_STATUS stat = 0;

   // what is the reporting rate -- queue rate, or individual?
   u32 rate = (pwq) ? pwq->delay_ms : pn->delay_ms;

   // if mode is 0, this is no change. Otherwise it must be supported!
   mode = (mode > 0) ? mode : pn->currentmode;
   if (mode != pn->currentmode)
   {
      if ( (mode & pn->modemask) != mode )
      {
         printk(KERN_ERR "+++ Sensor: [0x%x]. Invalid Reporting Mode [0x%x] requested. Supported: [0x%x] +++\n",
               pn->handle, mode, pn->modemask);

         return -ENXIO;
      }
   }

   // call into driver 
   stat = pn->cbkActivate(
        pn->drvcontext,    // driver context pointer
        flag,              // enable/disable
        mode,              // current driver mode; flipped inside IOCTL
        rate);             // current poll rate (queue or driver)

   if (stat == 0)
   {
      // flip the flag in node
      atomic_set (&(pn->enabled), flag);

      // remember the mode: Driver didn't object
      pn->currentmode = mode;

      // if default queue, must schedule work function
      if ((pwq == 0) && (flag) )
         queue_delayed_work(gMUX->pdefq, &(pn->work), msecs_to_jiffies(pn->delay_ms));    
   }

   return stat;
}


/* Activating of IRQ sensor node. 
   
*/
RI_SENSOR_STATUS enable_sensor_irq (sensirqnode* pnq, int enable)
{
    if (enable)
       RISENS_INFO("+++ %s: Enabling Sensor [0x%x] (%s). Mode: IRQ +++\n", __FUNCTION__, pnq->handle, pnq->name); 
    else
       RISENS_INFO("+++ %s: Disabling Sensor [0x%x] (%s). Mode: IRQ +++\n", __FUNCTION__, pnq->handle, pnq->name); 

     // currently last 2 parameters (rate and mode) have no meaning for irq sensors
     // *** keeping same function signature, as this can be potentially used later for different purposes ***
     if ( (pnq->cbkActivate) )
        pnq->cbkActivate (pnq->drvcontext, enable, 0, 0);   

     pnq->enabled = enable;
     return 0;
}

/* Diagnostics */
#if defined RISENS_DEBUG
void                sensor_node_dump_status (struct _sensnode* pnode)
{
    RISENS_INFO("+++ ---------------------------------- +++\n");

    RISENS_INFO("+++ Sensor Node: [%s]. Handle: 0x%x, Enabled: %d  Rate: %d [ms]. Buffer Size: %d, Filled: %d. Current mode: 0x%x, Supported: 0x%x +++\n",
       pnode->name, pnode->handle, atomic_read(&(pnode->enabled) ), pnode->delay_ms,  pnode->buffersize, pnode->filledsize, pnode->currentmode, pnode->modemask);

    RISENS_INFO("+++ ---------------------------------- +++\n");
}

ssize_t proxmux_write
(
   struct file* filp,
   const char __user*  buffer,
   size_t       len,
   loff_t*      off
)
{
   char buff [20];
   if (copy_from_user(buff, buffer, len) )
       return -EFAULT;

   if (len < 1) goto errexit;


   // show current MUX layout
   if (buff[0] == '3')
   {
       struct list_head* iter = 0;

       // dump default queue first
       RISENS_INFO("\n\n+++ Current MUX Device Configuration +++\n");
       RISENS_INFO("+++ Default Queue +++\n");
       RISENS_INFO("+++ ================================== +++\n");
       list_for_each (iter, &(gMUX->nodelist) )
       {
          sensor_node_dump_status( list_entry(iter, struct _sensnode, listhead) );
       }

       // dump custom queues now
       list_for_each (iter, &(gMUX->queuelist) )
       {
          mux_queue_dump_status ( list_entry(iter, struct _sensworkqueue, listhead) );
       } 

       goto errok;
   }


errok:
   RISENS_INFO("+++ %s: Exit +++\n", __FUNCTION__);
   return len;

errexit:
   RISENS_INFO("+++ %s: Invalid Arguments [%s] +++\n", __FUNCTION__, buff);
   return -EINVAL;
}
#endif    // #if defined RISENS_DEBUG


/* Exports to Drivers */
EXPORT_SYMBOL(risensor_register);         // registration: POLL mode
EXPORT_SYMBOL(risensor_register_irq);     // registration: IRQ mode

EXPORT_SYMBOL(risensor_irq_data);         // irq sensors: direct write

/* Module Entry Points */
subsys_initcall(proxmux_init);     // ensure mux is called BEFORE drivers
module_exit(proxmux_exit);         // cleanup -- never happens really

MODULE_DESCRIPTION("RECON Proxy Sensor Multiplexer");
MODULE_AUTHOR("Zeljko Kozomara");
MODULE_LICENSE("GPL");


