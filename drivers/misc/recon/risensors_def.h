#if !defined __risensor_def_h__
#define      __risensor_def_h__

/* *************************************************************************
 * risensors_def.h: Constants and types shared between different pieces of
 *                  Sensor Framework (both User and Kernel)
 *
 * *************************************************************************/

// Generic status code
typedef int RI_SENSOR_STATUS;

// Virtual sensor node name
#define PROXMUX_DRIVER_NAME "proxmux"


/* Sensor Handle Definition. We spec this by setting bits of low order byte in 32-bit value
  (Google claims only 256 Handles are supported currently & it must be part of low order byte,
  This could be an issue with our board configuration bitmask approach if we support more 
  than 8 sensors, but for now it is sufficient. 

  We choose the values so that bit which is set directly maps to Android SENSOR_TYPE_XXX enumeration
  (see <hardware/sensor.h> 
 */

typedef int            ANDROID_SENSOR_TYPE;
typedef int            RI_SENSOR_HANDLE;
typedef unsigned short RI_DATA_SIZE;

/* Primary Sensor Events */
#define RI_SENSOR_HANDLE_ACCELEROMETER        0x01      // 1 << (SENSOR_TYPE_ACCELEROMETER - 1)
#define RI_SENSOR_HANDLE_MAGNETOMETER         0x02      // 1 << (SENSOR_TYPE_MAGNETIC_FIELD - 1)
#define RI_SENSOR_HANDLE_GYROSCOPE            0x08      // 1 << (SENSOR_TYPE_GYROSCOPE - 1)
#define RI_SENSOR_HANDLE_PRESSURE             0x20      // 1 << (SENSOR_TYPE_PRESSURE  - 1)
#define RI_SENSOR_HANDLE_TEMPERATURE          0x40      // 1 << (SENSOR_TYPE_TEMPERATURE - 1)

//SENSOR_TYPE_LIGHT 5
//SENSOR_TYPE_PROXIMITY 8
#define RI_SENSOR_HANDLE_LIGHT                0x10      // 1 << (SENSOR_TYPE_LIGHT - 1)
#define RI_SENSOR_HANDLE_PROXIMITY            0x80      // 1 << (SENSOR_TYPE_PROXIMITY - 1)


/* Secondary -- Derived Sensor Events (Fusion) */
#define RI_SENSOR_HANDLE_ROTATION_VECTOR      0x400     // 1 << (SENSOR_TYPE_ROTATION_VECTOR - 1)


/* RI Specific Events */
#define RI_SENSOR_EVENT_BASE                  0x10

// FreeFall Event
#define RI_SENSOR_TYPE_FREEFALL               RI_SENSOR_EVENT_BASE + 1
#define RI_SENSOR_HANDLE_FREEFALL             0x10000   // 1 << (RI_SENSOR_TYPE_FREEFALL - 1) 

// Various macros for Handle <--> Type mapping
#define CHECK_BIT(var, pos) ((var) & (1<<(pos)))


/* Sensor registration */

// Sensor mode: bitmask
typedef unsigned short            RI_SENSOR_MODE;
#define RI_SENSOR_MODE_CR   0x01      // Sensor supports FIFO mode
#define RI_SENSOR_MODE_FIFO 0x02      // Sensor supports CR   mode

// Driver Sensor Activation. Flag 1 on, 0 off. Mode -- FIFO or CR. Rate: Queue reporting rate [ms].
// Driver should adjust Frequency depending on rate/mode. Returns 0 on success, or error code
typedef RI_SENSOR_STATUS (*PFNSENS_ACTIVATE)(void* context, unsigned char flag, RI_SENSOR_MODE mode, unsigned int rate);

// Sensor Data I/O in caller managed buffer. Returns: >=0 # of bytes filled, <0 error code
typedef RI_SENSOR_STATUS (*PFNSENS_READ)(void* context, unsigned char* databuffer, unsigned int buffersize);


// Registration in Poll mode
RI_SENSOR_STATUS risensor_register
(
    RI_SENSOR_HANDLE handle,        // sensor id
    const char*      name,          // sensor name
    void*            context,       // context pointer, passed back during callbacks (driver private data structure usually)
    RI_SENSOR_MODE   modemask,      // bitmask of modes this sensor supports (FIFO, CR or both)
    RI_SENSOR_MODE   currentmode,   // current mode this driver is on
    unsigned int     datasize,      // max data size (# of bytes) for this sensor
    PFNSENS_ACTIVATE cbkActivate,   // activate callback -- required. If flag is 1, extra is mode (FIFO or CR)
    PFNSENS_READ     cbkRead);      // read callback -- required


// Registration in Irq mode
RI_SENSOR_STATUS risensor_register_irq
(
    RI_SENSOR_HANDLE handle,        // sensor id
    const char*      name,          // sensor name
    void*            context,       // context pointer, passed back during callbacks (driver private data structure usually)
    unsigned int     datasize,      // max data size (# of bytes) for single interrupt event (usually timestamp)
    PFNSENS_ACTIVATE cbkActivate);  // activate callback -- optional (Zero is ok)


// IRQ direct driver access to MUX fifo.
RI_SENSOR_STATUS risensor_irq_data (RI_SENSOR_HANDLE handle, unsigned char* databuffer, RI_DATA_SIZE buffersize);


/** User Land Interface to PROXMUX Node **/

// encapsulation of command that can be issued to PROXMUX node via IOCTL
// parameters are context dependent
typedef struct _risensorcmd
{
   RI_SENSOR_HANDLE handle;         // sensor id

   unsigned int     long1;
   unsigned int     long2;
   unsigned short   short1;
   unsigned short   short2;
   unsigned short   short3;
   unsigned short   short4;
}risensorcmd;



/*** MUX Sensor Control ***/
#define PROXMUX_IOCTL_BASE 'x'  // 0x78 -- see Documentation/ioctl-number.txt

// Individual Sensor Enable/Disable
#define PROXMUX_IOCTL_SET_ENABLE  _IOR  (PROXMUX_IOCTL_BASE, 1, int) 

// Sensor Poll Rate: Compatibility with Android API - puts sensor to default (Kernel) queue   
#define PROXMUX_IOCTL_SET_DELAY   _IOR  (PROXMUX_IOCTL_BASE, 2, int)  

// Board Configuration: Mask of all registered sensors   
#define PROXMUX_IOCTL_GET_CONFIG  _IOW  (PROXMUX_IOCTL_BASE, 3, int)  

// Sensor Status (Enabled/Disabled + RI_SENSOR_MODE + poll rate)   
#define PROXMUX_IOCTL_GET_STATUS  _IOWR (PROXMUX_IOCTL_BASE, 4, int) 

// Start Sensor Reporting Queue: Handle (bitmask), param (rate [ms]). Returns Queue ID (Handle)   
#define PROXMUX_IOCTL_START_QUEUE _IOWR (PROXMUX_IOCTL_BASE, 5, int)    

// Stop Sensor Reporting Queue
#define PROXMUX_IOCTL_STOP_QUEUE  _IOR  (PROXMUX_IOCTL_BASE, 6, int)

// Set  Sensor Reporting Mode
#define PROXMUX_IOCTL_SET_MODE    _IOR  (PROXMUX_IOCTL_BASE, 7, int)

// Set  Sensor Fastest allowed reporting rate
#define PROXMUX_IOCTL_SET_MIN_DELAY _IOR (PROXMUX_IOCTL_BASE, 8, int)

//TODO: Expose Queue configuration query?

#endif

