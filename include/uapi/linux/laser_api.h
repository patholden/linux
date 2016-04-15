#ifndef _LASERAPI_H
#define _LASERAPI_H

// Kernel Event Timer values used
#define KETIMER_10M   10000         // Sets timer to approx 10msec
#define KETIMER_10U    10           // Sets timer to approx 10usec
#define KETIMER_30U    30           // 30usec, SENSE mode default
#define KETIMER_40U    40           // Sets timer to approx 40usec
#define KETIMER_50U    50           // Sets timer to approx 50usec
#define KETIMER_75U    75           // Sets timer to approx 75usec(default)
#define KETIMER_100U  100           // Sets timer to approx 100usec
#define KETIMER_150U  100           // Sets timer to approx 150usec

// In sensor mode, use this as indicator that target may be nearby sensed
// xy pair
#define SENSE_MAX_THRESHOLD          0x3F0

/*  IO addresses */
//NOTE: 0x380 & 0x390 are reserved for X86 PIC
#define   LG_BASE	     0x378
#define   LG_IO_CNTRL1       0x378
#define   LG_IO_CNTRL2       0x379          //
#define   LG_IO_XL           0x37A          // X low byte (writes to DAC input)
#define   LG_IO_XH           0x37B          // X high byte
#define   LG_IO_YL           0x37C          // Y low byte (writes to DAC input)
#define   LG_IO_YH           0x37D          // Y high byte
#define   TFPORTRL           0x37E          // Read lower byte of TF IO
#define   TFPORTRH           0x37F          // Read upper byte of TF IO
#define   LASEREND	     0x380
#define   LASER_REGION (LASEREND-LG_BASE)

//This area is the same as the ISA serial port region for the 1st 2 serial ports
//Depends on the 8250 device support being turned off in the kernel
#define   LG_TTYS2_BASE      0x3E8
#define   LG_TTYS2_END       0x3F0
#define   LG_TTYS2_REGION (LG_TTYS2_END-LG_TTYS2_BASE)

#define   LG_TTYS1_BASE	     0x2F8
#define   LG_TTYS1_END	     0x300
#define   LG_TTYS1_REGION (LG_TTYS1_END-LG_TTYS1_BASE)    

// Control flags for IO-writes
#define  BEAMONISSET    0x1
#define  BEAMONNOTSET   0xFE
#define  LASERENBISSET  0x2
#define  LASERENBNOTSET  0xFD
#define  BRIGHTBEAMISSET 0x4
#define  BRIGHTBEAMNOTSET 0xFB

// BITMASKS for LG_IO_CNTRL1
#define  LASER_BEAM_ON  0x40
#define  DACSTRBBITMASK  0x80
#define   STROBE_ON_LASER_ON  DACSTRBBITMASK | LASER_BEAM_ON
#define   STROBE_OFF_LASER_ON  LASER_BEAM_ON
#define   STROBE_ON_LASER_OFF  DACSTRBBITMASK
#define   STROBE_OFF_LASER_OFF  0x00

// BITMASKS FOR LG_IO_CNTRL2
#define  LASERENABLE  0x80
#define  LASERDISABLE 0x7F
#define  BRIGHTBEAM   0x40
#define  DIMBEAM      0xBF
#define  RDYLEDON     0x10
#define  RDYLEDOFF    0xEF
#define  LNKLEDON     0x04
#define  LNKLEDOFF    0xFB
#define  SHUTENB_ON_LASER_ON LASERENABLE | BRIGHTBEAM
#define  SHUTENB_ON_LASER_OFF LASERENABLE
#define  SHUTENB_OFF_LASER_ON BRIGHTBEAM
#define  SHUTENB_OFF_LASER_OFF DIMBEAM

// STATUS BITMASKS LG_IO_CNTRL2 READ
#define SHTOPNCLSDTCT 0x80
#define PHTEDGEDTCT   0x40
#define PHTLVLDTCT    0x20
#define CDRHBITMASK   0x08

// WRITE Command defines
typedef enum{
  CMDW_BUFFER=1,
  CMDW_STOP,
  CMDW_DISPLAY,
  CMDW_DODARKMOVE,
  CMDW_QUICKCHECK,
  CMDW_SETDELTA,
  CMDW_GOANGLE,
  CMDW_LOADWRTCNT,
  CMDW_SETQCCOUNTER,
  CMDW_SETXOFFSET,
  CMDW_SETYOFFSET,
  CMDW_ROIOFF,
  CMDW_SETROI,
  CMDW_STARTPULSE,
  CMDW_STOPPULSE,
  CMDW_SETPULSEONVAL,
  CMDW_SETPULSEOFFVAL,
  CMDW_SETQCFLAG,
  CMDW_SETCLOCK,
  CMDW_UNUSED2,
  CMDW_DOSENSOR,
  CMDW_READYLEDON,
  CMDW_READYLEDOFF,
  CMDW_SEARCHBEAMON,
  CMDW_SEARCHBEAMOFF,
  CMDW_LINKLEDON,
  CMDW_LINKLEDOFF,
  CMDW_SETSHUTENB,
  CMDW_CLEARSHUTENB,
  CMDR_BUFFER,
  CMDR_GETQCCOUNTER,
  CMDR_GETANGLE,
  CMDR_GETQCFLAG,
  CMDW_DOLITEMOVE,
  CMDW_STOPCMD,
  CMDW_LAST                    // DO NOT ADD BELOW HERE
}lg_cmd_enums;

// CMD-WRITE DEFINES
#define CMD_LAST CMDW_LAST    // NOTE:  This is used by lg_write to validate
                              // incoming commands.
#define MAX_TARGETS       24
#define MAX_XYPOINTS    2048 * MAX_TARGETS * 2 
#define MAX_LG_BUFFER   MAX_XYPOINTS * sizeof(struct lg_xydata)  /* maximum size of lg_data[] */
#define   MAX_TGFIND_BUFFER 32768  // maximum number of target-find readings
#define   DO_TEST_DISPLAY  0x1     // USED BY DIAGS.  Will simulate DISPLAY mode
struct hobbs_ctrs {
  time_t    hobbs_time;
  time_t    xscanner_time;
  time_t    yscanner_time;
  time_t    laser_time;
};
struct event_times {
  time_t    last_gap_usec;    // Used to track time between lg_timer events from user side
  time_t    last_exec_usec;   // Used to track execution time from user side
};
struct lg_xydata {
  uint8_t   ctrl_flags;
  char      unused1;
  int16_t   xdata;     // Needs to be signed to deal with LTC1597 bipolar data
  char      unused2;
  char      unused3;
  int16_t   ydata;     // Needs to be signed to deal with LTC1597 bipolar data
};
struct lg_xydelta {
  int16_t   xdata;
  int16_t   ydata;
};
struct lg_val16 {
  uint16_t   val16;
  uint16_t   pad2;
  uint32_t   pad3;
};
struct lg_val32 {
  uint32_t   val32;
  uint32_t   pad1;
};
struct lg_val64 {
  double     val64;
};
struct lg_move_data
{
  struct lg_xydata   xy_curpt;
  struct lg_xydelta  xy_delta;
  uint32_t           poll_freq;
  uint32_t           start_index;
  uint32_t           cur_index;
  uint32_t           nPoints;
  uint32_t           do_coarse;   // used for sensor-mode only
};
struct lg_disp_data
{
  uint32_t           poll_freq;
  uint32_t           start_index;
  uint32_t           cur_index;
  uint32_t           nPoints;
  uint32_t           is_restart;
};
struct lg_pulse_data
{
  struct lg_xydata   xy_curpt;
  uint32_t           poll_freq;
  uint32_t           counter;
  uint32_t           on_val;
  uint32_t           off_val;
};
struct cmd_hdr {
  uint16_t   cmd;
  uint16_t   test_mode;
  uint32_t   length;
};
struct cmd_rw_base {
  struct cmd_hdr hdr;
  union {
    struct lg_val16 dat16;
    struct lg_val32 dat32;
    struct lg_val64 dat64;
    struct lg_xydata xydata;
    struct lg_xydelta xydelta;
  };
};
struct cmd_rw_movedata {
  struct cmd_hdr hdr;
  struct lg_move_data movedata;
} __attribute__ ((packed));
struct cmd_rw_dispdata {
  struct cmd_hdr hdr;
  struct lg_disp_data dispdata;
} __attribute__ ((packed));
struct cmd_rw_pulsedata {
  struct cmd_hdr hdr;
  struct lg_pulse_data pulsedata;
} __attribute__ ((packed));

struct cmd_rw {
  struct cmd_rw_base base;
  struct lg_xydata  xydata[MAX_XYPOINTS];
} __attribute__ ((packed));

/* the following are ioctl commands following the linux convention */
#define LG_IOCNUM  0xE1
#define   LGGETANGLE      _IOR(LG_IOCNUM, 0xA2, struct lg_xydata)
#define   LGGETQCFLAG     _IOR(LG_IOCNUM, 0xB5, unsigned int)
#define   LGGETQCCOUNTER  _IOR(LG_IOCNUM, 0xB7, unsigned int)
#define   LGGETCTL2STAT   _IOR(LG_IOCNUM, 0xB8, unsigned int)
#define   LGGETEVENTTIMES _IOR(LG_IOCNUM, 0xB9, struct event_times)
#define   LGGETHOBBSTIMES _IOR(LG_IOCNUM, 0xB9, struct hobbs_ctrs)

#endif  /*  _LASERIOCTL_H  */
