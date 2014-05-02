#if !defined __muxprivate_h__
#define      __muxprivate_h__

/* *************************************************************************
 * muxprivate.h: Various declarations private to MUX device
 *
 * *************************************************************************/

#include "risensors_def.h"           // framework definitions

#include <linux/list.h>              // linked list container of nodes
#include <linux/workqueue.h>         // workqueue stuff
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>

//#define RISENS_DEBUG   1  // comment out in production
//#define RISENS_VERBOSE 1  // super verbose debugging; comment in production

#if defined CONFIG_RISENS_DEBUG
   #define RISENS_INFO(fmt, args...) printk(KERN_DEBUG "PROXMUX: " fmt, ##args)
#else
   #define RISENS_INFO(fmt, args...)    // nothing
#endif

#if defined RISENS_VERBOSE
   #define RISENS_INFO_V(fmt, args...) printk(KERN_DEBUG "PROXMUX: " fmt, ##args)
#else
   #define RISENS_INFO_V(fmt, args...)    // nothing
#endif

#define QUEUE_NAME_SIZE  10            // queue name: create_singlethread_queue expects max 10
#define DEFAULT_QUEUE_NAME "Default"   // Default Queue name
#define CUSTOM_QUEUE_PREFIX  "sq"      // Custom Queue Prefix
#define DEFAULT_DELAY    1000          // 1 sec default polling rate   
#define MINIMUM_DELAY    1             // minimum reporting rate

#define NODE_LOCK_INIT(pn) mutex_init(&(pn->lock));

#define NODE_LOCK(pn)  mutex_lock(&(pn->lock));
            /*RISENS_INFO("+++ %s:Locking Node [0x%x] +++\n", __FUNCTION__, pn->handle); \
            mutex_lock(&(pn->lock)); \
            RISENS_INFO("+++ %s:Node [0x%x] Locked +++\n", __FUNCTION__, pn->handle); */

#define NODE_UNLOCK(pn) mutex_unlock(&(pn->lock));
            /*RISENS_INFO("+++ %s:Unlocking Node [0x%x] +++\n", __FUNCTION__, pn->handle); \
            mutex_unlock(&(pn->lock)); \
            RISENS_INFO("+++ %s:Node [0x%x] Unlocked +++\n", __FUNCTION__, pn->handle); */

/*
   Polling Sensor Node Definition

   Work is contained in one of MUX maintained queues. Queue can have >=1 Sensor Nodes
   associated with it. User can also request work to perform
   on global kernel queue (PROXMUX_IOCTL_SET_DELAY -- sensorHAL::setDelay)
*/
typedef struct _sensnode
{
   struct list_head   listhead;          // Queue maintains list of Sensor Nodes, so this is required

   struct delayed_work work;             // work structure for this node, when placed on default queue

   RI_SENSOR_HANDLE   handle;            // sensor id
   const char*        name;              // sensor string -- passed during registration. Memory is owned by the Driver

   unsigned char*     databuffer;        // event data buffer; allocated during registration based on FIFO size
   RI_DATA_SIZE       buffersize;        // total size of event data buffer (bytes)
   RI_DATA_SIZE       filledsize;        // currently filled size

   atomic_t           enabled;           // enabled/disabled flag
   u32                delay_ms;          // individual node firing rate, when in default Kernel Queue
   u32                min_delay;         // minimum delay for this sensor; will be passed from user at init

   PFNSENS_ACTIVATE   cbkActivate;       // activate callback
   PFNSENS_READ       cbkRead;           // read callback

   void*              drvcontext;        // driver context

   RI_SENSOR_MODE     modemask;          // bitmask of supported reporting modes, passed during configuration
   RI_SENSOR_MODE     currentmode;       // current reporting mode for this sensor

   struct mutex       lock;              // node lock
}sensnode;

/* 
   IRQ Sensor Node Definition:
      -- this is for support of FreeFall interrupt really; however
         architecture allows any number and type of sensors registering
         and propagating through MUX framework 
 */
typedef struct _sensirqnode
{
   struct list_head   listhead;          // weird Linux link list implementation

   RI_SENSOR_HANDLE   handle;            // sensor UID
   const char*        name;              // sensor string -- passed during registration. Memory is owned by the Driver
   
   unsigned char*     databuffer;        // event data buffer; allocated during registration for single interrupt event
   RI_DATA_SIZE       buffersize;        // event data size   (non-zero required).

   PFNSENS_ACTIVATE   cbkActivate;       // activate callback
   void*              drvcontext;        // driver context

   int                enabled;           // enabled flag
   struct mutex       lock;              // node lock

}sensirqnode;


/* Sensor Workqueue Definition */
typedef struct _sensworkqueue
{
    struct delayed_work work;              // kernel work structure for this queue

    struct list_head    nodelist;          // sensor nodes managed by this workqueue
    struct list_head    listhead;          // MUX device maintains list of queues, so this is required

    // queue properties: poll rate, type
    u32 delay_ms;                          // queue firing rate
    u32 exit;                              // stop signal

    struct mutex          lock;
    struct workqueue_struct*  pwq;         // Kernel workqueue structure
}sensworkqueue;

/* internal functions between modules */



// Sensor Node Query
void find_sensor_node (RI_SENSOR_HANDLE handle, struct _sensnode** ppnode, struct _sensworkqueue** ppwq);  
void find_sensor_node_irq (RI_SENSOR_HANDLE handle, struct _sensirqnode** ppnq);

// Sensor Node Enabling
RI_SENSOR_STATUS enable_sensor_poll (struct _sensnode* pn, struct _sensworkqueue* pwq, unsigned char flag, RI_SENSOR_MODE mode);
RI_SENSOR_STATUS enable_sensor_irq (sensirqnode* pnq, int enable);

// Set reporting rate for Specific POLL sensor
RI_SENSOR_STATUS set_sensor_reporting_rate (struct _sensnode* pn, struct _sensworkqueue* pwq, unsigned int delay_ms);

// Set minimum reporting rate for Specific POLL sensor
RI_SENSOR_STATUS set_sensor_min_reporting_rate (struct _sensnode* pn, struct _sensworkqueue* pwq, unsigned int delay_ms);

// Start new queue
RI_SENSOR_STATUS start_sensor_queue (RI_SENSOR_HANDLE mask, unsigned int delay_ms);

// Transfer sensor node from one queue to another
void transfer_sensor_node (struct _sensnode* pn, struct _sensworkqueue* pdestination);

// Get Full Bitmask of registered sensors
void get_board_config (risensorcmd* pcmd); 

// Data Ready signal. Parameter is sensors bitmask
void   mux_signaldata (RI_SENSOR_HANDLE mask); 

// Empty custom queue. Transfer all sensor nodes to default queue
void empty_custom_queue (struct _sensworkqueue* pq);

// Sensor Node Data Read
int sensor_node_read (unsigned char* psrc, RI_DATA_SIZE srcsize, unsigned char* pdest, size_t* pcopied);

// Queue API
struct _sensworkqueue* mux_queue_create  (u32 delay);
void                   mux_queue_destroy (struct _sensworkqueue* pthis);

void                   mux_queue_start (struct _sensworkqueue* pthis);
void                   mux_queue_stop  (struct _sensworkqueue* pthis);

struct _sensworkqueue* mux_queue_get (struct list_head* pqlist, unsigned int delay_ms, unsigned char create);
u8                     mux_queue_can_start (RI_SENSOR_HANDLE mask, unsigned int delay_ms);
u8                     mux_queue_is_empty  (struct _sensworkqueue* pthis);

void                   mux_queue_nodes_count(struct _sensworkqueue* pthis, unsigned int* ptotal, unsigned int* pactive);

#if defined RISENS_DEBUG
   void                mux_queue_dump_status   (struct _sensworkqueue* pthis);
   void                sensor_node_dump_status (struct _sensnode* pnode);
   ssize_t             proxmux_write(struct file* filp, const char __user*  buffer, size_t len, loff_t* off);
#endif

// queue locking. It might not be necessary to do this, as any operation 
// always executes in context of single thread (invoked by user ioctl). 
// In that case simply set defines bellow to nothing
#define MUX_QUEUE_LOCK_INIT(pq) mutex_init(&(pq->lock));   //spin_lock_init(&(pq->lock));
#define MUX_QUEUE_LOCK(pq)      mutex_lock(&(pq->lock));   //spin_lock(&(pq->lock));
#define MUX_QUEUE_UNLOCK(pq)    mutex_unlock(&(pq->lock)); //spin_unlock(&(pq->lock));

#endif   // if !defined __muxprivate_h__

