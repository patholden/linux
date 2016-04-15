#ifndef _LASERDEV_H
#define _LASERDEV_H
/******************************************************************************/
/*                                                                            */
/*     Specific changes for the LaserVision system                            */
/* NOTE1:  lg_out_data is an array of XY and control-flag data.  X & Y MUST   */
/*         use OUTB and even-based byte (high byte) MUST be last because data */
/*         is clocked into DAC on that byte write.                            */
/* NOTE2:  LTC1597 spec calls for bipolar 16 bit values to produce correct    */
/*         +/- VREF output voltages. Refer to Linear Technologies LTC1597     */
/*         Data sheet, "Bipolar Offset Binary Code Table                      */
/*          (table starting with Vref(32,767/32,768)" for details.            */
/* NOTE3:  Intel x86s are little endian, so, a position is four 16-bit        */
/*         values:                                                            */
/*                   X high byte->LG_IO_XH                                    */
/*                   X low  byte->LG_IO_XL                                    */
/*                   Y high byte->LG_IO_YH                                    */
/*                   Y low  byte->LG_IO_YL                                    */
/******************************************************************************/

// Max # devices allowed for laser (only 1 is initialized right now)
#define LG_NUM_DEVICES   256
#define LTC1597_BIPOLAR_OFFSET_MAX   0xFFFF
#define LTC1597_BIPOLAR_OFFSET_PLUS  0x8000
#define LTC1597_BIPOLAR_OFFSET_NEG   0x7FFF
#define LTC1597_BIPOLAR_MAX          0x7FFF
#define LTC1597_BIPOLAR_MAX_INP_VAL1 0x7FFF
#define LTC1597_BIPOLAR_MAX_INP_VAL2 0x8000
#define SHORT_MAX_OVERFLOW           0x10000
#define SENSOR_DEF_FREQ              40
#define SENSOR_COARSE_READ_FREQ      400
#define SENSOR_FINE_READ_FREQ        100
#define SENSOR_MAX_POLL_NUM          100

// state machine defines
enum lg_states {
    LGSTATE_IDLE=0,
    LGSTATE_DISPLAY,
    LGSTATE_SENSE,
    LGSTATE_SENSEREAD,
    LGSTATE_SENSEREADY,
    LGSTATE_DARKMOVE,
    LGSTATE_LITEMOVE,
    LGSTATE_GOPULSE,
};

struct lg_dev {
  // dev stuff first
  struct miscdevice       miscdev;
  struct miscdevice 	  lgttyS1;
  struct miscdevice       lgttyS2;
  struct mutex            lg_mutex;
  spinlock_t              lock;
  struct kref             ref;
  wait_queue_head_t       wait;
  struct list_head        free;
  struct list_head        used;
  struct completion       completion;
  struct device           *dev;
  void __iomem	          *iobase;
  struct dentry           *dbg_entry;
  // Start of actual lg private data
  struct hrtimer          lg_timer;
  struct timeval          last_ts;
  struct event_times      last_events;     // Used to track timing of lg_evt_hdlr
  struct lg_disp_data     lg_display;
  struct lg_move_data     lg_darkmove;
  struct lg_move_data     lg_litemove;
  struct lg_move_data     lg_sensor;
  struct lg_pulse_data    lg_gopulse;
  struct lg_xydata        lg_lastxy;
  struct lg_xydata        lg_goangle;
  struct lg_xydelta       lg_delta;
  struct lg_xydata        *lg_display_data;
  uint32_t                lg_disp_count;
  uint32_t                lg_state;
  uint32_t                poll_frequency;    // default clock setting used by idle
  uint8_t                 lg_ctrl2_store;    // Control byte 2 settings
  uint8_t                 pad[3];
};
struct file_priv {
	struct lg_dev *lg_devp;
};
struct lg_dev_ops {
  void (*lg_getoptic)(struct device *dev, int status);
  void (*lg_set_opticcmd)(struct device *dev, int optics_cmd);
  void (*lg_set_opticstat)(struct device *dev, int optics_status);
};

#endif  /*  _LASERGUIDE_H  */
