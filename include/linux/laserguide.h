/*     specific changes for the laserguide system  */

#define   MAX_LG_BUFFER    262144  /* maximum size of lg_data[] */
#define   MAX_DIODE_BUFFER  65536  /* maximum number of diode readings */

#ifdef __KERNEL__


   /* NOTE:  lg_out_data is 16-bit short so that outw can move data out ASAP */
   /*   Intel x86s are little endian, so, a position is four shorts:     */
   /*      X position, low word  */
   /* for low byte (1st), also bit 7   strobe bit   */
   /*                          bit 6   unblank      */
   /*      X position, high word */
   /*      Y position, low word  */
   /*      Y position, high word */
   /*  please also note that the "write" routine must be given  */
   /*  a char (byte) array (each position being 8 bytes)        */
extern unsigned short lg_out_data[]; 
   /*  arrary of diode data   */
extern unsigned char   diode_data[];
extern unsigned short  diode_word[];

extern unsigned char lg_xlo_debug[];
extern unsigned char lg_xhi_debug[];
extern unsigned char lg_ylo_debug[];
extern unsigned char lg_yhi_debug[];
extern long lg_index_debug[];
extern unsigned char lg_info_debug[];
extern long  lg_ring_debug;
extern long  lg_count_debug;


extern long  lg_out_data_index;   /* current index in lg_data */
extern long  lg_out_data_start;   /* start of displayed data in lg_data */
extern long  lg_out_data_end;     /* end of displayed data in lg_data */
extern long  lg_out_data_max;     /* end of valid data in lg_data */
extern long  lg_in_data_index;   /* current index in lg_data */
extern long  lg_in_data_end;     /* end of valid data in lg_data */
extern long  lg_move;         /* non-zero to move laser */
extern long  lg_trigger;      /* variable of looking at trigger */
extern unsigned char  lg_threshold;  /* light threshold ? */

extern unsigned long  lg_num_points;   /* number of points in all */
extern unsigned long  lg_x_save;     /* last X position written out */
extern unsigned long  lg_y_save;     /* last Y position written out */
extern unsigned long  lg_x_delta;     /*  step in X position */
extern unsigned long  lg_y_delta;     /*  step in Y position */

extern long  lg_qc_flag;    /* quick check flag  (0 is false) */
extern long  lg_qc_counter; /* quick check dec. counter (-1 for no qc) */

extern long  lg_hw_flag;       /* hardware debug flag  (0 is false) */
extern long  lg_hw_trigger;    /* trigger flag for line search */

extern unsigned short lg_xhi_offset;   /* offset for xhi word */
extern unsigned short lg_yhi_offset;   /* offset for yhi word */

 /* region-of-interest stuff */
extern long  lg_roi_on;   
extern long  lg_roi_del;
extern long  lg_roi_test;
extern long  lg_roi_dwell;
extern long  lg_roi_first;
extern long  lg_roi_search;
extern long  lg_debounce;
extern long  lg_hit;
extern long  lg_trig_flag;
extern long  lg_step_neg;
extern long  lg_index_avg;

extern unsigned char lg_optic_store;

 /* pulse laser mode stuff */
extern long  lg_pulsemodeflag;
extern long  lg_pulsecounter;
extern long  lg_pulseonvalue;
extern long  lg_pulseoffvalue;

extern struct wait_queue * lg_wait;  /* wait queue */

#endif  /* __KERNEL__ */

      /* the following are ioctl commands following the linux convention */
#define   LGSTOP        _IO( 0x99, 0x02)

#define   LGGOANGLE     _IOW(0x99, 0x03, union lg_info)

#define   LGDISPLAY     _IO( 0x99, 0x05)
#define   LGQUICKCHECK  _IO( 0x99, 0x06)
#define   LGDOSENSOR    _IO( 0x99, 0x09)

#define   LGDODARKSENS  _IO( 0x99, 0x29)

#define   LGSETSEARCHB  _IO( 0x99, 0x10)
#define   LGCLRSEARCHB  _IO( 0x99, 0x11)
#define   LGSETSWITCHB  _IO( 0x99, 0x12)
#define   LGCLRSWITCHB  _IO( 0x99, 0x13)
#define   LGSETLEDBIT   _IO( 0x99, 0x14)
#define   LGCLRLEDBIT   _IO( 0x99, 0x15)
#define   LGGETSTORE    _IOR(0x99, 0x16, union lg_info)
#define   LGFASTSCAN    _IO( 0x99, 0x18)
#define   LGSLOWSCAN    _IO( 0x99, 0x19)
#define   LGSETLINKLED  _IO( 0x99, 0x1A)
#define   LGCLRLINKLED  _IO( 0x99, 0x1B)

#define   LGLOADTHRESH  _IOW(0x99, 0xA1, union lg_info)

#define   LGGETANGLE    _IOR(0x99, 0xA2, union lg_info)
#define   LGGETBRIGHT   _IOR(0x99, 0xA3, union lg_info)
#define   LGGETSERVO    _IOR(0x99, 0xA4, union lg_info)

#define   LGLOADOPTIC   _IOW(0x99, 0xA5, union lg_info)

#define   LGGETOPTIC    _IOR(0x99, 0xA6, union lg_info)
#define   LGGETBOARD    _IOR(0x99, 0xA7, union lg_info)
#define   LGGETBOOST    _IOR(0x99, 0xA8, union lg_info)
#define   LGGETTERROR   _IOR(0x99, 0xA9, union lg_info)

#define   LGLOADREADNUMBER   _IOW(0x99, 0xAA, union lg_info)
#define   LGLOADWRITENUMBER  _IOW(0x99, 0xAB, union lg_info)

#define   LGFASTCLOCK   _IO( 0x99, 0xB0)
#define   LGSLOWCLOCK   _IO( 0x99, 0xB1)

#define   LGSETCLOCK    _IOW(0x99, 0xB2, union lg_info)
#define   LGSETDELTA    _IOW(0x99, 0xB3, union lg_info)

#define   LGGETQCFLAG    _IOR(0x99, 0xB5, union lg_info)
#define   LGSETQCFLAG    _IOW(0x99, 0xB6, union lg_info)

#define   LGGETQCCOUNTER    _IOR(0x99, 0xB7, union lg_info)
#define   LGSETQCCOUNTER    _IOW(0x99, 0xB8, union lg_info)

#define   LGHWFLAGON    _IO(0x99, 0xB9)
#define   LGHWFLAGOFF   _IO(0x99, 0xBA)

#define   LGHWTRIGGER   _IO(0x99, 0xBB)

#define   LGROIOFF      _IO(0x99, 0xC0)
#define   LGROION       _IO(0x99, 0xC1)

#define   LGSETROI       _IOW(0x99, 0xC2, union lg_info)

#define   LGSETROISEARCH _IO(0x99, 0xC3)
#define   LGCLRROISEARCH _IO(0x99, 0xC4)

#define   LGGETIP        _IOR(0x99, 0xD0, union lg_info)

#define   LGSETXOFF      _IOW(0x99, 0xD1, union lg_info)
#define   LGSETYOFF      _IOW(0x99, 0xD2, union lg_info)

#define   LGSTARTPULSE   _IO(0x99, 0xDA)
#define   LGSTOPPULSE    _IO(0x99, 0xDB)
#define   LGSETPULSEON   _IOW(0x99, 0xDC, union lg_info)
#define   LGSETPULSEOFF  _IOW(0x99, 0xDD, union lg_info)


/*  IO addresses */
#define   LG_BASE            0x380
#define   LG_IO_X0           0x380
#define   LG_IO_X1           0x381
#define   LG_IO_X2           0x382
#define   LG_IO_X3           0x383
#define   LG_IO_Y0           0x384
#define   LG_IO_Y1           0x385
#define   LG_IO_Y2           0x386
#define   LG_IO_Y3           0x387
#define   LG_IO_SERVO        0x380
#define   LG_IO_OPTIC        0x388
#define   LG_IO_BOARD        0x38A
#define   LG_IO_TCPIP        0x38C
#define   TFPORTRL           0x390          // Read TF IO port address
#define   TFPORTRH           0x392          // Read upper byte of TF IO ;
#define   TFPORTW            0x394          // Write TF IO port address 
#define   LASER_REGION (TFPORTW-LG_BASE)

struct lg_points {
   unsigned long   xpos;   
   unsigned long   ypos;
};

struct lg_delta {
   long   xdel;   
   long   ydel;
};

   /* the following is a union passed to ioctl() */
   /*   (changed to a union)  */
union lg_info {
       /*   positions are 16/18 bits left-justified in 32-bit longs         */
       /*  X and Y positions, plus a strobe bit 7 of first (least sig) byte */
   struct lg_points ptns;
       /*   change in position for searching sensor                         */
   struct lg_delta  delta;
   long   num_points;   /* number of points (as position pairs!) */
   long   roi_points;   /* number region-of-interest points  */
   unsigned short  threshold;  /* diode light threshold -- needed? */
   unsigned short  clock_tick;  /* length of clock tick -- 1.192 MHz */
   unsigned short  pos_offset;  /* position offset (high word) */
      /*  servo status -- input from projector  */
      /*    bit 7   X ok                        */
      /*    bit 6   Y ok                        */
      /*    bit 5   Projector Power Supplies up */
      /*    bit 4   position err comparator  (future imp)       */
      /*    bit 3   (unassigned)                                */
      /*    bit 2   scan. board ID 2                            */
      /*    bit 1   scan. board ID 1                            */
      /*         ID2    ID1                                     */
      /*          0      0      6350 scanners installed         */
      /*          0      1      6810 scanners installed         */
      /*          1      0      undef                           */
      /*          1      1      undef                           */
      /*    bit 0   0=18-bit DAC     1=16-bit DAC               */
   unsigned char   servo_status;
      /*  optics command  --  output to projector   */
      /*   bit 7   fast/slow scanner speed toggle                   */
      /*   bit 6   search power   set enables higher power          */
      /*   bit 5   video switch and flag  (video board?)            */
      /*   bit 4   comm error LED   visible on outside              */
      /*   bit 3   n/a                                              */
      /*   bit 2   ethernet link LED                                */
      /*   bit 1   n/a                                              */
      /*   bit 0   n/a                                              */
   unsigned char   optics_command;
      /*  optics status  --  input from projector   */
      /*   bit 7   shutter relay check ???                          */
      /*   bit 6   trigger 1 (edge)  goes to 1 briefly (how brief?) */
      /*   bit 5   trigger 2 (level)                                */
      /*   bit 4   diode temperature check                          */
      /*   bit 3   Center of Devices and Radiological Health        */
      /*   bit 2   diode current check                              */
      /*   bit 1   interlock check                                  */
      /*   bit 0   n/a                                              */
   unsigned char   optics_status;
      /*  board configuration  --  input from projector   */
      /*   bit 7   optics board ID 3                                */
      /*   bit 6   optics board ID 2                                */
      /*   bit 5   optics board ID 1                                */
      /*   bit 4   optics board ID 0                                */
      /*   bit 3   diode driver ID 3  0000 SDL 15mW 635nm laser diode        */
      /*   bit 2   diode driver ID 2  0001 Sanyo 10mW 635nm laser diode      */
      /*   bit 1   diode driver ID 1  0010 Casix 50mW 532nm Double Nd:YVO4   */
      /*   bit 0   diode driver ID 0    all others undefined                 */
   unsigned char   board_config;
       /* diode_light, diode_boost, and temp_error are inputs for projector */
   unsigned char   diode_light;  /* diode light monitor  0x00 to 0xFF */
   unsigned char   diode_boost;  /* diode boost current  0x00 to 0xFF */
       /*  temperature error      */
       /*  0x00  =  -10 C below set-point  */
       /*  0x80  =  perfect   (nominal setpoint should be 17 C) */
       /*  0xFF  =  +10 C above set-point  */
   unsigned char   temp_error;
   unsigned char   low_ip_octet;
   unsigned char   unassigned1;
   unsigned long   unassigned2;
   long            qcFlag;      /* quick check flag (0 is false) */
   long            qcCounter;   /* quick check counter */
   long            pulseonvalue;
   long            pulseoffvalue;
};
