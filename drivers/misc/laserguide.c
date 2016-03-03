/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2015 Assembly Guidance Systems, Inc.  All rights reserved.
 *
 * Description:
 *	AGS Laser Guidance driver.  Refer to laser_guide.h for ioctl commands
 *      available to control the laser guidance system.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/laser_dev.h>
#include <linux/laser_api.h>

#define LG_VERSION "0.2"
#define DEV_NAME   "laser"

struct lg_dev {
  // dev stuff first
  struct miscdevice       miscdev;
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
  struct timer_list       lg_timer;
  uint32_t                time_expires;
};
struct file_priv {
	struct lg_dev *lg_devp;
};
static struct lg_xydata lg_out_data[MAX_XYPOINTS];
static uint8_t  diode_data[MAX_DIODE_BUFFER];
static uint16_t diode_word[MAX_DIODE_BUFFER];
static uint32_t lg_out_data_index = 0;
uint32_t lg_out_data_end   = 0;
uint32_t lg_in_data_index = 0;
uint32_t lg_in_data_end   = 0;
uint32_t lg_state = LGSTATE_IDLE;
uint32_t lg_qc_flag = 0;
uint32_t lg_qc_counter = 0;
int32_t  lg_hw_trigger = 0;
int32_t  lg_roi_test   = 0;
int32_t  lg_roi_dwell  = 0;
int32_t  lg_roi_on     = 0;
int32_t  lg_roi_del    = 0;
int32_t  lg_index_avg = 0;
int32_t  lg_pulsemodeflag   = 0;
int32_t  lg_pulsecounter    = 0;
int32_t  lg_pulseonvalue    = 0;
int32_t  lg_pulseoffvalue   = 0;
int32_t  lg_dark_search;
struct lg_xydata   lg_save;
struct lg_xydata   lg_delta;
static int lg_shutter_open;
uint8_t lg_threshold = 0;
uint8_t lg_ctrl2_store = 0;

// DEFINES used by event timer
#define UARTPORT 0x3F8
#define UCOUNT   500

// STATIC FUNCTION PROTOTYPES
static        void lg_timeout(unsigned long data);
static inline void lg_write_io_to_dac(struct lg_xydata *pDevXYData);
static int lg_pdev_remove(struct platform_device *pdev);
static int lg_dev_probe(struct platform_device *dev);

// START OF LOCAL FUNCTIONS
static void lg_get_xydata_ltcval(int16_t *output_val, int16_t input_val)
{
  if (!input_val)
    *output_val = LTC1597_BIPOLAR_OFFSET_ZERO;
  else if ((input_val == LTC1597_BIPOLAR_MAX_INP_VAL1)
	   || (input_val == LTC1597_BIPOLAR_MAX_INP_VAL2))
    *output_val = LTC1597_BIPOLAR_OFFSET_MAX;
  else if (input_val < 0)
    *output_val = (input_val & LTC1597_BIPOLAR_OFFSET_NEG);
  else
    *output_val = input_val | LTC1597_BIPOLAR_OFFSET_PLUS;
  return;
}
static inline void lg_write_io_to_dac(struct lg_xydata *pDevXYData)
{
  int16_t       dac_xval;
  int16_t       dac_yval;
  unsigned char ctrl1_on = 0;
  unsigned char ctrl1_off = 0;
  int8_t        xhi,xlo,ylo,yhi;
 
  // Strobe-bit high to load in CNTRL1 register
  if (pDevXYData->ctrl_flags & BEAMONISSET)
    {
      ctrl1_on = STROBE_ON_LASER_ON;
      ctrl1_off = STROBE_OFF_LASER_ON;
    }
  else
    {
      ctrl1_on = STROBE_ON_LASER_OFF;
      ctrl1_off = STROBE_OFF_LASER_OFF;
    }

  // Figure out & set control byte 2
  if (pDevXYData->ctrl_flags & LASERENBISSET)
    lg_ctrl2_store |= LASERENABLE;
  else
    lg_ctrl2_store &= LASERDISABLE;
  if (pDevXYData->ctrl_flags & BRIGHTBEAMISSET)
    lg_ctrl2_store |= BRIGHTBEAM;
  else
    lg_ctrl2_store &= DIMBEAM;
  outb(lg_ctrl2_store, LG_IO_CNTRL2);  // Apply CNTRL2 settings

  // Adjust XY data for producing correct LTC1597 output voltage to DAC
  lg_get_xydata_ltcval((int16_t *)&dac_xval, pDevXYData->xdata);
  lg_get_xydata_ltcval((int16_t *)&dac_yval, pDevXYData->ydata);
 // Write XY data, data is applied to DAC input after lo byte is written
  // so sequence is important.  hi byte then lo byte.
  // udelay is here for adhering to timing spec of DAC chips
  xhi = (int8_t)(dac_xval >> 8) & 0xFF;
  xlo = (int8_t)(dac_xval & 0xFF);
  yhi = (int8_t)(dac_yval >> 8);
  ylo = (int8_t)(dac_yval & 0xFF);
  outb(xhi, LG_IO_XH);
  outb(xlo, LG_IO_XL);
  outb(yhi, LG_IO_YH);
  outb(ylo, LG_IO_YL);
  // Let data WRITE to DAC input register operation take place
  outb(ctrl1_on, LG_IO_CNTRL1);
  // Strobe bit 0->1 latches data,
  // Strobe bit 1->0 writes data to DAC
  outb(ctrl1_off, LG_IO_CNTRL1);
  return;
}
static int lg_open(struct inode *inode, struct file *file)
{
  return 0;
}
int lg_release(struct inode *_inode, struct file *f)
{
  return 0;
}

static int lg_proc_cmd(struct cmd_rw *p_cmd_data, struct lg_dev *priv)
{
  struct lg_xydata  xy_data;
  uint32_t          timer_expiration;

  if (!priv)
    return(-ENODEV);

  // Stop timer for new command
  del_timer_sync(&priv->lg_timer);

  // May be running timer here, so make sure we have the latest saved value
  timer_expiration = usecs_to_jiffies(priv->time_expires);

  switch(p_cmd_data->base.cmd) {
  case CMDW_RSTRTTMR:
    mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
    break;
  case CMDW_SETQCCOUNTER:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_qc_counter = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_STOP:
    lg_state = LGSTATE_IDLE;
    memset((char *)&xy_data, 0, sizeof(struct lg_xydata));
    lg_write_io_to_dac((struct lg_xydata *)&xy_data);
    break;
  case CMDW_STARTPULSE:
    lg_pulsecounter  = 0;
    lg_pulsemodeflag = 1;
    break;
  case CMDW_STOPPULSE:
    lg_pulsemodeflag = 0;
    break;
  case CMDW_DISPLAY:
    lg_qc_flag = 0;   /* set quick check flag to "false" */
    lg_state = LGSTATE_DISPLAY;
    lg_out_data_index = 0;  /* reset from beginning */
    lg_ctrl2_store |= LASERENABLE;
    mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
    break;
  case CMDW_ROIOFF:
    lg_roi_test  = 0;
    lg_roi_on    = 0;
    lg_roi_dwell = 6;
    break;
  case CMDW_DOSENSOR:  /* everything should be set up before this */
    lg_dark_search = 0;
    lg_state = LGSTATE_DOSENS;       /* not moved to read routine */
    lg_in_data_index = 0;
    mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
    break;
  case CMDW_DODARKSENS:  /* everything should be set up before this */
    lg_dark_search = 1;
    lg_state = LGSTATE_DOSENS;       /* not moved to read routine */
    lg_in_data_index = 0;
    mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
    break;
  case CMDW_QUICKCHECK:
    lg_state    = LGSTATE_DISPLAY;
    mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
    break;
  case CMDW_SETDELTA:
    if (p_cmd_data->base.length != sizeof(struct lg_xydata))
      return(-EINVAL);
    memcpy((char *)&lg_delta, (char *)&p_cmd_data->base.xydata, sizeof(struct lg_xydata)); 
    break;
  case CMDW_GOANGLE:
    if (p_cmd_data->base.length != sizeof(struct lg_xydata))
      return(-EINVAL);

    /* Disable LGDISPLAY mode, only writing 1 set of coords here */
    lg_state = LGSTATE_IDLE;
    lg_save.xdata = p_cmd_data->base.xydata.xdata;
    lg_save.ydata = p_cmd_data->base.xydata.ydata;
    lg_save.ctrl_flags = p_cmd_data->base.xydata.ctrl_flags;
    lg_write_io_to_dac((struct lg_xydata *)&p_cmd_data->base.xydata);
    break;
  case CMDW_SETROI:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_roi_del = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_LOADWRTCNT:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_out_data_end = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_LOADRDCNT:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_in_data_end   = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_SETPULSEONVAL:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_pulsemodeflag = 0;
    lg_pulseonvalue = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_SETPULSEOFFVAL:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_pulsemodeflag = 0;
    lg_pulseoffvalue = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_SETQCFLAG:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    // When a quick check is needed, this flag is set to non-zero
    // LGDISPLAY  automatically sets this to zero
    lg_qc_flag = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_SETCLOCK:
    if (p_cmd_data->base.length != sizeof(uint32_t))
      return(-EINVAL);
    // timer expires ~x usec as set by init file
    priv->time_expires = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_READYLEDON:
    lg_ctrl2_store |= RDYLEDON;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_READYLEDOFF:
    lg_ctrl2_store &= RDYLEDOFF;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_SEARCHBEAMON:
    lg_ctrl2_store |= BRIGHTBEAM;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_SEARCHBEAMOFF:
    //    lg_ctrl2_store &= DIMBEAM;
        lg_ctrl2_store &= LASERDISABLE;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_LINKLEDOFF:
    lg_ctrl2_store &= LNKLEDOFF;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_LINKLEDON:
    lg_ctrl2_store |= LNKLEDON;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_SETSHUTENB:
    lg_ctrl2_store |= LASERENABLE;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_CLEARSHUTENB:
    lg_ctrl2_store &= LASERDISABLE;
    outb(lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  default:
    printk(KERN_ERR "\nAGS-LG: CMDW %d option not found", p_cmd_data->base.cmd);
    break;
  }
  return(0);
};
ssize_t lg_write(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos)
{
  struct lg_dev *priv;
  struct cmd_rw *cmd_data;
  struct lg_xydata *pXYData;
  int           i;

  priv = (struct lg_dev *)file->private_data;
  if (!priv)
    return(-EBADF);
  
  // Get command data
  cmd_data = kzalloc(sizeof(struct cmd_rw), GFP_KERNEL);
  if (!cmd_data)
    {
      pr_err("kzalloc() failed for laser\n");
      return(-ENOMEM);
    }

  if (count <= 0) return -EINVAL;
  if (count > sizeof(struct cmd_rw)) return -ENOMEM;

  if(copy_from_user((char *)cmd_data, buffer, count))
    return -EFAULT;

  // Validate command type
  if (!cmd_data->base.cmd || (cmd_data->base.cmd > CMD_LAST))
    return(-EINVAL);
  
  if (cmd_data->base.cmd != CMDW_BUFFER)
    return(lg_proc_cmd(cmd_data, priv));

  if (!cmd_data->base.length || (cmd_data->base.length > (sizeof(struct cmd_rw)-offsetof(struct cmd_rw, xydata))))
    return(-EINVAL);

  // Copy data from cmd buffer to local static buff
  if (cmd_data->base.length > MAX_LG_BUFFER)
    {
      printk(KERN_ERR "\nAGS-LG: LG_WRITE Buffer too big %d",cmd_data->base.length);
      return(-EINVAL);
    }
  memset((char *)&lg_out_data, 0, sizeof(lg_out_data));
  memcpy((char *)&lg_out_data[0], (char *)&cmd_data->xydata[0], cmd_data->base.length);
  if (cmd_data->base.test_mode == DO_TEST_DISPLAY)
    {
      lg_qc_flag = 0;   /* set quick check flag to "false" */
      for(i = 0; i < (cmd_data->base.length/sizeof(struct lg_xydata)); i++)
	{
	  pXYData = (struct lg_xydata *)&lg_out_data[i*sizeof(struct lg_xydata)];
	  lg_write_io_to_dac(pXYData);
	}
    }

  lg_out_data_end = cmd_data->base.length;
  lg_out_data_index = 0;
  return count;
}

/**************************************************
*                                                 *
* lg_read()                                       *
* Description:   This function is used to process *
*                read commands.  Either a buffer  *
*                of data (diode_word) is sent     *
*                back to user.                    *
*                                                 *
**************************************************/
ssize_t lg_read(struct file *file, char __user * buffer
               , size_t count
               , loff_t *f_pos
               )
{
  struct lg_dev *priv = file->private_data;

  if (!priv)
    return(-EBADF);
    
  if (count<=0) return -EINVAL;
  if (count > MAX_DIODE_BUFFER)
    return(-ENOMEM);
  if (lg_state == LGSTATE_DOSENS)
    return(-EAGAIN);
  if (lg_state <= LGSTATE_DISPLAY)
    return(-EBUSY);
  if (lg_state != LGSTATE_TGFIND)
    return(-EIO); 

  /* sensor scan has been done */
  if (copy_to_user((void *)buffer, (void *)diode_word, (uint32_t)count))
    return -EFAULT;    
  lg_state = LGSTATE_IDLE;    /* reset to IDLE state */
  return count;
}

  /* the messy catch-all driver routine to do all sorts of things */
long lg_ioctl(struct file *file, unsigned int cmd, unsigned long arg )
{
  struct lg_xydata  xydata;
  struct lg_dev     *priv;
  void __user       *argp = (void __user *)arg;
  uint32_t          timer_expiration;
  uint32_t          ctl2_val;
  
  priv = (struct lg_dev *)file->private_data;
  if (!priv)
    return(-EBADF);

  timer_expiration = usecs_to_jiffies(priv->time_expires);
  switch (cmd) {
  case LGGETANGLE:
    xydata.xdata = lg_save.xdata;
    xydata.ydata = lg_save.ydata;
    xydata.ctrl_flags = lg_save.ctrl_flags;
    if (copy_to_user(argp, &xydata, sizeof(struct lg_xydata) ))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
    // Restart event timer
    mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1msec
    break;
    /* this is a count-down and must be reset each time */
    /*  -1 is reserved for NO quick checks              */
  case LGGETQCCOUNTER:
    if (copy_to_user(argp, &lg_qc_counter, sizeof(int)))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
    break;
  case LGGETQCFLAG:
    if (copy_to_user(argp, &lg_qc_flag, sizeof(int)))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
    break;
  case LGGETCTL2STAT:
    ctl2_val = (uint32_t)(inb(LG_IO_CNTRL2) & 0xFF);
    if (copy_to_user(argp, &ctl2_val, sizeof(uint32_t)))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
    break;
  default:
    return(-EINVAL);
  }
  return(0);
}

static long compat_lg_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
  return(lg_ioctl(f, cmd, arg));
}

static void lg_timeout(unsigned long data)
{
  char             time_lg_inbuff[8];
  struct lg_xydata *xydata;
  struct lg_dev    *priv;
  uint32_t         timer_expiration;
  uint16_t         lg_sum;
  uint16_t         lg_00x;
  uint16_t         lg_x00;
  uint8_t          tg_find_val;
  uint8_t          b_optic;

  priv = (struct lg_dev *)data;
  if (!priv)
    return;
  timer_expiration = usecs_to_jiffies(priv->time_expires);

  //  Check CHDR and, if need be, change shutter state
  b_optic = inb(LG_IO_CNTRL2);
  if (b_optic & CDRHBITMASK)
    {
      if (lg_shutter_open == 0)
	{
	  lg_ctrl2_store |= LASERENABLE;
	  outb(lg_ctrl2_store, LG_IO_CNTRL2);
	}
      lg_shutter_open = 1;
    }
  else
    {
      if (lg_shutter_open == 1)
	{
	  lg_ctrl2_store &= LASERDISABLE;
	  outb(lg_ctrl2_store, LG_IO_CNTRL2);
	}
      lg_shutter_open = 0;
    }
  /*
   *  laser pulse mode
   *   if flag is on, do this and only this section
   */
  if (lg_pulsemodeflag)
    {
      // first, the sanity checks (turn off pulse mode)
      if ( lg_pulsecounter  > 16384 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulsecounter  <     0 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseonvalue  > 16384 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseonvalue  <     0 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseoffvalue > 16384 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseoffvalue <     0 )  { lg_pulsemodeflag = 0; }

      // Now get to the business of turning the beam on and off
      lg_pulsecounter++;
      if ((lg_pulsecounter <= lg_pulseoffvalue) || (lg_pulsecounter <= lg_pulseonvalue))
	{
	  printk(KERN_INFO "\nAGS-LG PULSE: xdata %x,ydata %x,ctrl_flags %x",
		 lg_save.xdata,lg_save.ydata,lg_save.ctrl_flags);
	  lg_write_io_to_dac((struct lg_xydata *)&lg_save);
	}
    else
      {
	lg_pulsecounter = 0;
      }
      // Restart timer to continue working on data until user-app suspends work
      mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
      return;
  }

  if (lg_state == LGSTATE_IDLE)
    return;
  else if (lg_state == LGSTATE_DISPLAY)
    {
      // Check for end of buffer, start over when end is reached
      if ((lg_out_data_index * sizeof(struct lg_xydata)) >= lg_out_data_end)
	lg_out_data_index = 0;
      xydata = (struct lg_xydata *)&lg_out_data[lg_out_data_index++];
      lg_save.ctrl_flags = xydata->ctrl_flags;
      lg_save.xdata = xydata->xdata;
      lg_save.ydata = xydata->ydata;
      lg_write_io_to_dac((struct lg_xydata*)&lg_save);
      
      /* a negative lg_qc_counter will never do a quick check */
      if (lg_qc_counter > 0)
	lg_qc_counter--;   /* decrement counter */
      else if (lg_qc_counter == 0)
	{
	  lg_qc_flag = 1;   /* set the quick check flag */
	  lg_state = LGSTATE_IDLE;      /*  go into the idle state  */
	}
    }
#if 0
  // FIXME---PAH---THIS STATE IS NEVER USED!
  else if( lg_state == 4 )
    {
      if ( lg_in_data_index > 3 )
	{
	  /* stop everything and leave mark */
	  diode_data[0] = 0xA5;
	  diode_data[1] = 0x5A;
	  diode_data[2] = 0xA5;
	  diode_data[3] = 0x5A;
	  lg_state = LGSTATE_TGFIND;
	  lg_dark_search = 0;
	}
      else
	{
	  // prepare x & Y vals
	  xydata = (struct lg_xydata *)&lg_out_data[lg_out_data_index];
	  if (lg_dark_search)
	    xydata->ctrl_flags &= BRIGHTBEAMNOTSET;
	  printk(KERN_INFO "\nAGS-LG:  STATE4 X=%d, Y=%d, flags %d",
		 xydata->xdata,xydata->ydata, xydata->ctrl_flags);
	  lg_write_io_to_dac(xydata);
	  printk(KERN_INFO "\nAGS-LG BAD4: DK xdata %x,ydata %x,ctrl_flags %x",
		 xydata->xdata,xydata->ydata,xydata->ctrl_flags);
	  diode_data[lg_in_data_index] = inb(LG_IO_CNTRL2);
	  lg_in_data_index++;   /* increment positions and index */
	  lg_save.xdata += lg_delta.xdata;
	  lg_save.ydata += lg_delta.ydata;
	  lg_save.ctrl_flags = xydata->ctrl_flags;
	  if (lg_in_data_index >= lg_in_data_end)
	    {
	      lg_state = LGSTATE_TGFIND;
	      lg_dark_search = 0;
	    }
	}
    }
#endif
  else if (lg_state == LGSTATE_DOSENS)
    {
      // prepare x & Y vals
      xydata = (struct lg_xydata *)&lg_out_data[lg_out_data_index];
      if (lg_dark_search)
	xydata->ctrl_flags &= BEAMONNOTSET;
      lg_write_io_to_dac(xydata);
      
      // Read in data from target-find board
      tg_find_val = inb(TFPORTRL);
      time_lg_inbuff[1] = tg_find_val & 0xff;
      tg_find_val = inb(TFPORTRH);
      time_lg_inbuff[0] = tg_find_val & 0x03;
      
      lg_00x = 0xff & time_lg_inbuff[1];
      lg_x00 = 0xff & time_lg_inbuff[0];
      lg_sum = lg_00x + lg_x00 * 0x100 ;
      
      diode_word[lg_in_data_index] = lg_sum;
      if ( lg_sum < 0xFF )
	diode_data[lg_in_data_index] = lg_sum & 0xFF;
      else
	diode_data[lg_in_data_index] = 0xFF;

      lg_in_data_index++;   /* increment positions and index */
      lg_save.xdata  += lg_delta.xdata;
      lg_save.ydata  += lg_delta.ydata;
      lg_save.ctrl_flags = xydata->ctrl_flags;
      if (lg_in_data_index >= lg_in_data_end)
	{
	  lg_state = LGSTATE_TGFIND;
	  lg_dark_search = 0;
	}
    }
  // Restart timer to continue working on data until user-app suspends work
  mod_timer(&priv->lg_timer, jiffies + timer_expiration);  // Restart timer
  return;
}
static const struct file_operations lg_fops = {
  .owner	    = THIS_MODULE,
  .llseek           = no_llseek,
  .read             =  lg_read,        /* lg_read */
  .write            = lg_write,       /* lg_write */
  .unlocked_ioctl   = lg_ioctl,       /* lg_ioctl */
#ifdef CONFIG_COMPAT
  .compat_ioctl    = compat_lg_ioctl,
#endif
  .open             = lg_open,        /* lg_open */
  .release          = lg_release,     /* lg_release */
};
struct miscdevice lg_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = DEV_NAME,
  .fops = &lg_fops,
};
static int lg_dev_probe(struct platform_device *plat_dev)
{
  struct lg_xydata xydata;
  struct lg_dev *lg_devp;
  struct device *this_device;
  int           rc;

  // allocate mem for struct device will work with
  lg_devp = kzalloc(sizeof(struct lg_dev), GFP_KERNEL);
  if (!lg_devp)
    {
      pr_err("kzalloc() failed for laser\n");
      return(-ENOMEM);
    }

  memset((char *)lg_devp,0,sizeof(struct lg_dev));
  platform_set_drvdata(plat_dev, lg_devp);
  lg_devp->dev = &plat_dev->dev;

  // We don't use kref or mutex locks yet.
  kref_init(&lg_devp->ref);
  mutex_init(&lg_devp->lg_mutex);

  dev_set_drvdata(lg_devp->dev, lg_devp);
  spin_lock_init(&lg_devp->lock);
  INIT_LIST_HEAD(&lg_devp->free);
  INIT_LIST_HEAD(&lg_devp->used);
  init_waitqueue_head(&lg_devp->wait);

  // Setup misc device
  lg_devp->miscdev.minor = lg_device.minor;
  lg_devp->miscdev.name = DEV_NAME;
  lg_devp->miscdev.fops = lg_device.fops;
  rc = misc_register(&lg_devp->miscdev);
  if (rc)
    {
      printk(KERN_ERR "AGS-LG:  Failed to register Laser misc_device, err %d \n", rc);
      kfree(lg_devp);
      return(rc);
    }

  this_device = lg_devp->miscdev.this_device;
  lg_devp->miscdev.parent = lg_devp->dev;
  dev_set_drvdata(this_device, lg_devp);
  platform_set_drvdata(plat_dev, lg_devp);
  printk(KERN_INFO "\nAGS-LG:laser misc-device created\n");

  // Obtain IO space for device
  if (!request_region(LG_BASE, LASER_REGION, DEV_NAME))
    {
      kfree(lg_devp);
      misc_deregister(&lg_devp->miscdev);
      printk(KERN_CRIT "\nUnable to get IO regs");
      return(-EBUSY);
    }
  
  // Set up the event timer but don't start it yet(del_timer_sync()) stops it.
  lg_devp->time_expires = KETIMER_75U;  // DEFAULT event timer is 75 usec
  lg_devp->lg_timer.function = lg_timeout;
  lg_devp->lg_timer.expires = 0;
  lg_devp->lg_timer.data = (unsigned long)lg_devp;
  init_timer(&lg_devp->lg_timer);
  
  /* move to 0,0 position */
  memset((char *)&xydata, 0, sizeof(struct lg_xydata));
  lg_write_io_to_dac(&xydata);

  // Initialize save, delta, & all buffers to 0
  memset((char *)&lg_save, 0, sizeof(struct lg_xydata));
  memset((char *)&lg_delta, 0, sizeof(struct lg_xydata));
  memset((char *)&lg_out_data, 0, sizeof(lg_out_data));
  memset((char *)&diode_word, 0, sizeof(diode_word));
  memset((char *)&diode_data, 0, sizeof(diode_data));
  
  // All initialization done, so enable timer
  add_timer(&lg_devp->lg_timer);
  printk(KERN_INFO "\nAGS-LG:Laser Guide miscdevice installed, timer created.\n");
  return(0);
}
static struct platform_device lg_dev = {
  .name   = "laser",
  .id     = 0,
  .num_resources = 0,
};
static const struct of_device_id lg_of_match[] = {
	{ .compatible = "ags-lg,laser", },
	{},
};
static struct platform_driver lg_platform_driver = {
  .probe   = lg_dev_probe,
  .remove  = lg_pdev_remove,
  .driver  = {
    .name  = DEV_NAME,
  },
};
static int lg_pdev_remove(struct platform_device *pdev)
{
  struct lg_dev *lg_devp = platform_get_drvdata(pdev);
  struct device *this_device;
  
  if (!lg_devp)
    return(-EBADF);

  this_device = lg_devp->miscdev.this_device;
  release_region(LG_BASE, LASER_REGION);
  misc_deregister(&lg_devp->miscdev);
  kfree(lg_devp);
  return(0);
}

static int __init laser_init(void)
{
  int rc;

  rc = platform_driver_register(&lg_platform_driver);
  if (rc)
    {
      printk(KERN_ERR "Unable to register platform driver laser, ret %d\n", rc);
      return(rc);
    }
  rc = platform_device_register(&lg_dev);
  if (rc)
    {
      printk(KERN_ERR "Unable to register platform device laser, ret %d\n", rc);
      return(rc);
    }
  printk(KERN_INFO "\nAGS-LG:Laser Guide platform device/driver installed.\n");
  return(rc);
}

static void __exit laser_exit(void)
{
  platform_device_unregister(&lg_dev);
  platform_driver_unregister(&lg_platform_driver);
  return;
}
module_init(laser_init);
module_exit(laser_exit);

MODULE_AUTHOR("Patricia A. Holden for Assembly Guidance Systems");
MODULE_DESCRIPTION("Driver for AGS Laser Guidance System 2");
MODULE_LICENSE("GPL");

