#ifndef _LASERAPI_H
#define _LASERAPI_H

// Kernel Event Timer values used
#define KETIMER_10M   10000         // Sets timer to approx 10msec
#define KETIMER_40U    40           // Sets timer to approx 40usec
#define KETIMER_50U    50           // Sets timer to approx 50usec
#define KETIMER_75U    75           // Sets timer to approx 75usec(default)
#define KETIMER_100U  100           // Sets timer to approx 100usec
#define KETIMER_150U  100           // Sets timer to approx 150usec

/*  IO addresses */
#define   LG_BASE            0x380
#define   LG_IO_CNTRL1       0x380
#define   LG_IO_X2           0x382          // X low byte (writes to DAC input)
#define   LG_IO_X3           0x383          // X high byte
#define   LG_IO_Y2           0x386          // Y low byte (writes to DAC input)
#define   LG_IO_Y3           0x387          // Y high byte
#define   LG_IO_CNTRL2       0x388          // 
#define   TFPORTRL           0x390          // Read TF IO port address
#define   TFPORTRH           0x392          // Read upper byte of TF IO ;
#define   TFPORTW            0x394          // Write TF IO port address 
#define   LASER_REGION (TFPORTW-LG_BASE)

// Control flags for IO-writes
#define  UNBLANKISSET    0x1
#define  SHUTENBISSET  0x2
#define  BRIGHTISSET   0x4

// BITMASKS for LG_IO_CNTRL1
#define  UNBLANKBITMASK  0x40
#define  DACSTRBBITMASK  0x80
#define   STROBE_ON_LASER_ON  DACSTRBBITMASK | UNBLANKBITMASK
#define   STROBE_OFF_LASER_ON  UNBLANKBITMASK
#define   STROBE_ON_LASER_OFF  DACSTRBBITMASK
#define   STROBE_OFF_LASER_OFF  0x00

// BITMASKS FOR LG_IO_CNTRL2
#define  SHUTENBBITMASK  0x80
#define  BRIGHTBITMASK   0x40
#define  RDYLEDBITMASK      0x10
#define  LNKLEDBITMASK   0x02
#define  SHUTENB_ON_LASER_ON SHUTENBBITMASK | BRIGHTBITMASK
#define  SHUTENB_ON_LASER_OFF SHUTENBBITMASK
#define  SHUTENB_OFF_LASER_ON BRIGHTBITMASK
#define  SHUTENB_OFF_LASER_OFF 0x0

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
  CMDW_RSTRTTMR,
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
#define   MAX_LG_BUFFER    0x40000  /* maximum size of lg_data[] */
#define   MAX_DIODE_BUFFER 0x10000  /* maximum number of diode readings */

struct lg_xydata {
  char      ctrl_flags;
  char      unused1;
  uint16_t  xdata;
  char      unused2;
  char      unused3;
  uint16_t  ydata;
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
  uint32_t   cmd;
  uint32_t   length;
  union {
    struct lg_val16 dat16;
    struct lg_val32 dat32;
    struct lg_val64 dat64;
    struct lg_xydata xydata;
  };
};
struct cmd_rw {
  struct cmd_rw_base base;
  char   data[MAX_LG_BUFFER * sizeof(struct lg_xydata)];
};

/* the following are ioctl commands following the linux convention */
#define LG_IOCNUM  0xE1
#define   LGGETANGLE    _IOR(LG_IOCNUM, 0xA2, struct lg_xydata)
#define   LGGETQCFLAG    _IOR(LG_IOCNUM, 0xB5, unsigned int)
#define   LGGETQCCOUNTER  _IOR(LG_IOCNUM, 0xB7, unsigned int)
#define   LGGETCTL2STAT  _IOR(LG_IOCNUM, 0xB8, unsigned int)

#endif  /*  _LASERIOCTL_H  */
