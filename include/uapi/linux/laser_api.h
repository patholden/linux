#ifndef _LASERAPI_H
#define _LASERAPI_H

// Kernel Event Timer values used
#define KETIMER_10M   10000         // Sets timer to approx 10msec
#define KETIMER_10U    10           // Sets timer to approx 10usec
#define KETIMER_40U    40           // Sets timer to approx 40usec
#define KETIMER_50U    50           // Sets timer to approx 50usec
#define KETIMER_75U    75           // Sets timer to approx 75usec(default)
#define KETIMER_100U  100           // Sets timer to approx 100usec
#define KETIMER_150U  100           // Sets timer to approx 150usec
#define XYMAX         0x7FFF        // XY vals can't exceed 0x7FFF for DAC.

/*  IO addresses */
//NOTE: 0x380 & 0x390 are reserved for X86 PIC
#define   LG_BASE            0x378
#define   LG_IO_CNTRL1       0x378
#define   LG_IO_CNTRL2       0x379          //
#define   LG_IO_XL           0x37A          // X low byte (writes to DAC input)
#define   LG_IO_XH           0x37B          // X high byte
#define   LG_IO_YL           0x37C          // Y low byte (writes to DAC input)
#define   LG_IO_YH           0x37D          // Y high byte
#define   TFPORTRL           0x37E          // Read lower byte of TF IO
#define   TFPORTRH           0x37F          // Read upper byte of TF IO
#define   LASEREND           0x380          // RESERVED BY X86 FOR APIC
#define   LASER_REGION (LASEREND-LG_BASE)

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
  CMDW_DODARKSENS,
  CMDW_QUICKCHECK,
  CMDW_SETDELTA,
  CMDW_GOANGLE,
  CMDW_LOADWRTCNT,
  CMDW_LOADRDCNT,
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
}lg_cmd_enums;

// CMD-WRITE DEFINES
#define CMD_LAST CMDR_GETQCFLAG   // NOTE:  Change this if appending new commands
#define   MAX_LG_BUFFER    0x80000  /* maximum size of lg_data[] */
#define   MAX_TGFIND_BUFFER 0x10000  /* maximum number of target-find readings */
#define   DO_TEST_DISPLAY  0x1      // USED BY DIAGS.  Will simulate DISPLAY mode
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
struct cmd_rw_base {
  uint16_t   cmd;
  uint16_t   test_mode;
  uint32_t   length;
  union {
    struct lg_val16 dat16;
    struct lg_val32 dat32;
    struct lg_val64 dat64;
    struct lg_xydata xydata;
  };
};
#define   MAX_XYPOINTS    MAX_LG_BUFFER/sizeof(struct lg_xydata)
struct cmd_rw {
  struct cmd_rw_base base;
  struct lg_xydata  xydata[MAX_XYPOINTS];
};


/* the following are ioctl commands following the linux convention */
#define LG_IOCNUM  0xE1
#define   LGGETANGLE      _IOR(LG_IOCNUM, 0xA2, struct lg_xydata)
#define   LGGETQCFLAG     _IOR(LG_IOCNUM, 0xB5, unsigned int)
#define   LGGETQCCOUNTER  _IOR(LG_IOCNUM, 0xB7, unsigned int)
#define   LGGETCTL2STAT   _IOR(LG_IOCNUM, 0xB8, unsigned int)
#define   LGGETEVENTTIMES _IOR(LG_IOCNUM, 0xB9, struct event_times)
#define   LGGETHOBBSTIMES _IOR(LG_IOCNUM, 0xB9, struct hobbs_ctrs)

#endif  /*  _LASERIOCTL_H  */
