#ifndef _LASERDEV_H
#define _LASERDEV_H
/*     specific changes for the laserguide system  */

   /* NOTE:  lg_out_data is 16-bit values so that outw can move data out ASAP */
   /*   Intel x86s are little endian, so, a position is four 16-bit values:     */
   /*      X position, low word  */
   /* for low byte (1st), also bit 7   strobe bit   */
   /*                          bit 6   unblank      */
   /*      X position, high word */
   /*      Y position, low word  */
   /*      Y position, high word */
   /*  please also note that the "write" routine must be given  */
   /*  single-byte array (each position being 8 bytes)        */

// state machine defines
#define LGSTATE_IDLE     0
#define LGSTATE_DISPLAY  1
#define LGSTATE_DOSENS   2
#define LGSTATE_TGFIND   3

// Max # devices allowed for laser (only 1 is initialized right now)
#define LG_NUM_DEVICES   256

struct lg_dev_ops {
  void (*lg_getoptic)(struct device *dev, int status);
  void (*lg_set_opticcmd)(struct device *dev, int optics_cmd);
  void (*lg_set_opticstat)(struct device *dev, int optics_status);
};

#endif  /*  _LASERGUIDE_H  */
