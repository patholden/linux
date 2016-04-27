/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2015 Assembly Guidance Systems, Inc.  All rights reserved.
 *
 * Description:
 *	AGS Laser Guidance driver.  Refer to laser_api.h for commands
 *      available to control the laser guidance system.
 */



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/laser_api.h>
#include <linux/laser_dev.h>
#include <linux/serial_reg.h>

#define LG_VERSION 	 "0.3"
#define DEV_NAME   	 "laser"
#define DEV_NAME_LGTTYS1 "lgttyS1"
#define DEV_NAME_LGTTYS2 "lgttyS2"

static struct lg_xydata lg_display_data[MAX_XYPOINTS];
static uint16_t tgfind_word[MAX_TGFIND_BUFFER];
uint32_t lg_qc_flag = 0;
uint32_t lg_qc_counter = 0;
int32_t  lg_hw_trigger = 0;
int32_t  lg_roi_test   = 0;
int32_t  lg_roi_dwell  = 0;
int32_t  lg_roi_on     = 0;
int32_t  lg_roi_del    = 0;
static int lg_shutter_open;
uint8_t lg_threshold = 0;
uint32_t status = 0;

// DEFINES used by event timer
#define UARTPORT LG_TTYS1_BASE //0x2F8 
#define UCOUNT   500

//DEFINES used by LG serial port functions 
#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

// STATIC FUNCTION PROTOTYPES
static enum hrtimer_restart lg_evt_hdlr(struct hrtimer *timer);
static inline void lg_write_io_to_dac(struct lg_dev *priv, struct lg_xydata *pDevXYData);
static int lg_pdev_remove(struct platform_device *pdev);
static int lg_dev_probe(struct platform_device *dev);

//Simple Serial IO functions
ssize_t LG_SerialRead1(struct file *file, char __user *buffer, size_t count, loff_t *f_pos)
{
    int  value;
    struct lg_dev *priv = file->private_data;

    if (!priv)
      return(-EBADF);

    if (count != 1)
      return(-EINVAL);

    status = inb(LG_TTYS1_BASE + UART_LSR);
    if ((status & UART_LSR_DR) != UART_LSR_DR) //Rx Ready??
      return(-EAGAIN);

    value = inb(LG_TTYS1_BASE);
    if (copy_to_user(buffer, (char *)&value, count))
      return(-EFAULT);
    return(count);
}


ssize_t LG_SerialWrite1(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos)
{
  int value;

  struct lg_dev *priv = file->private_data;

  if (!priv)
    return(-EBADF);

  if (count != 1)
    return -EINVAL;

  status = inb(LG_TTYS1_BASE + UART_LSR);
  if ((status & BOTH_EMPTY) != BOTH_EMPTY) //Tx Empty??
    return(EAGAIN);

  if(copy_from_user((char *)&value, buffer, count))
    {
      return -EFAULT;
    }

  outb(value, LG_TTYS1_BASE);
  return(count);
}

ssize_t LG_SerialRead2(struct file *file, char __user *buffer, size_t count, loff_t *f_pos)
{
    int value;
    struct lg_dev *priv = file->private_data;

    if (!priv)
      return(-EBADF);

    if (count != 1)
      return(-EINVAL);

    status = inb(LG_TTYS2_BASE + UART_LSR);
    if ((status & UART_LSR_DR) != UART_LSR_DR) //Rx Ready??
 	return(-EAGAIN);

    value = inb(LG_TTYS2_BASE);
    if (copy_to_user(buffer, (char *)&value, count))
      return(-EFAULT);
    return(count);
}


ssize_t LG_SerialWrite2(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos)
{
  int value;
  struct lg_dev *priv = file->private_data;

  if (!priv)
    return(-EBADF);

  if (count != 1)
    return -EINVAL;

  status = inb(LG_TTYS2_BASE + UART_LSR);
  if ((status & BOTH_EMPTY) != BOTH_EMPTY) //Tx Empty??
    return(-EAGAIN);

  if(copy_from_user((char *)&value, buffer, count))
    {
      return -EFAULT;
    }

  outb(value, LG_TTYS2_BASE);
  return(count);
}


//Start of Local Functions
//The following 2 are used to init the serial port registers in the FPGA
static uint8_t LG_SerialRead(int port, int offset)
{
	uint8_t value;
	value = inb(port + offset);
        return(value);
}


static void LG_SerialWrite(int port, int offset, int value)
{
        outb(value, port + offset);
	return;
}

static void lg_get_xydata_ltcval(int16_t *output_val, int16_t input_val)
{
  if (!input_val)                                       // Value 0 is represented as 0x8000 by LTC1597
    *output_val = LTC1597_BIPOLAR_OFFSET_PLUS;
  else if (input_val < 0)                               // Negative values are 0x1->0x7FFF 
    *output_val = (input_val & LTC1597_BIPOLAR_OFFSET_NEG);
  else
    {
      if (input_val >= LTC1597_BIPOLAR_OFFSET_PLUS)    // Positive values are 0x8000->0xFFFF
	*output_val = input_val;
      else
	*output_val = input_val | LTC1597_BIPOLAR_OFFSET_PLUS;
    }
  return;
}
static void lg_adjust_xypoints(struct lg_move_data *lg_data)
{
    int32_t temp_val1;
    int16_t current_val;

    if (lg_data->xy_delta.xdata)
      {
	current_val = lg_data->xy_curpt.xdata;
	temp_val1 = lg_data->xy_curpt.xdata + lg_data->xy_delta.xdata;
	lg_data->xy_curpt.xdata = temp_val1 & LTC1597_BIPOLAR_OFFSET_MAX;
	if (lg_data->xy_curpt.xdata == 0)
	  {
	    // If the current value of X was negative -> 0 then need
	    // to adjust correctly to go positive.  Otherwise, go negative
	    // so that DAC doesn't fault.
	    if (current_val < 0)
	      lg_data->xy_curpt.xdata = LTC1597_BIPOLAR_OFFSET_PLUS;
	    else
	      lg_data->xy_curpt.xdata = 1;
	  }
      }
    if (lg_data->xy_delta.ydata)
      {
	current_val = lg_data->xy_curpt.xdata;
	temp_val1 = lg_data->xy_curpt.ydata + lg_data->xy_delta.ydata;
	lg_data->xy_curpt.ydata = temp_val1 & LTC1597_BIPOLAR_OFFSET_MAX;
	if (lg_data->xy_curpt.ydata == 0)
	  {
	    // If the current value of Y was negative -> 0 then need
	    // to adjust correctly to go positive.  Otherwise, go negative
	    // so that DAC doesn't fault.
	    if (current_val < 0)
	      lg_data->xy_curpt.ydata = LTC1597_BIPOLAR_OFFSET_PLUS;
	    else
	      lg_data->xy_curpt.ydata = 1;
	  }
      }
    return;
}
static inline void lg_write_io_to_dac(struct lg_dev *priv, struct lg_xydata *pDevXYData)
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

  // LASER check. 3 states must have laser enable in specific
  // state (LGSTATE_DARKMOVE, LGSTATE_LITEMOVE, LGSTATE_SENSE).
  // Otherwise, pair control flags are checked.  if set, disable
  // laser to turn off beam, otherwise laser is enabled and either
  // set to high-intensity(BRIGHTBEAM flag set) or low-intensity
  // (DIMBEAM mask to clear BRIGHTBEAM flag) beam.
  if (priv->lg_state == LGSTATE_DARKMOVE)
    priv->lg_ctrl2_store &= LASERDISABLE;
  else if (priv->lg_state == LGSTATE_SENSE)
    priv->lg_ctrl2_store |= LASERENABLE;
  else if (priv->lg_state == LGSTATE_LITEMOVE)
    priv->lg_ctrl2_store |= LASERENABLE;
  else
    {
      if (pDevXYData->ctrl_flags & LASERENBISSET)
	priv->lg_ctrl2_store |= LASERENABLE;
      else
	priv->lg_ctrl2_store &= LASERDISABLE;
    }	
  if (pDevXYData->ctrl_flags & BRIGHTBEAMISSET)
    priv->lg_ctrl2_store |= BRIGHTBEAM;
  else
    priv->lg_ctrl2_store &= DIMBEAM;
  outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);  // Apply CNTRL2 settings

  // Adjust XY data for producing correct LTC1597 output voltage to DAC
  // This function also avoids fault condition.
  lg_get_xydata_ltcval((int16_t *)&dac_xval, pDevXYData->xdata);
  lg_get_xydata_ltcval((int16_t *)&dac_yval, pDevXYData->ydata);

  // Write XY data, data is applied to DAC input after lo byte is written
  // so sequence is important.  hi byte then lo byte.
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
static int lg_proc_pulse_cmd(struct cmd_rw_pulsedata *p_cmd_pulse, struct lg_dev *priv)
{
      priv->lg_state    = LGSTATE_IDLE;
      if (p_cmd_pulse->hdr.length != sizeof(struct lg_pulse_data))
	return(-EINVAL);
      priv->lg_state = LGSTATE_GOPULSE;
      priv->lg_ctrl2_store |= LASERENABLE;
      outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
      priv->lg_gopulse.xy_curpt.xdata = p_cmd_pulse->pulsedata.xy_curpt.xdata;
      priv->lg_gopulse.xy_curpt.ydata = p_cmd_pulse->pulsedata.xy_curpt.ydata;
      priv->lg_gopulse.xy_curpt.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
      priv->lg_gopulse.counter = p_cmd_pulse->pulsedata.counter;
      priv->lg_gopulse.on_val = p_cmd_pulse->pulsedata.on_val;
      priv->lg_gopulse.off_val = p_cmd_pulse->pulsedata.off_val;
      priv->lg_gopulse.poll_freq = p_cmd_pulse->pulsedata.poll_freq;
      if (!priv->lg_gopulse.poll_freq)
	priv->lg_gopulse.poll_freq = KETIMER_150U;
      priv->lg_gopulse.poll_freq *= 1000;
      printk(KERN_INFO "\nAGS-LG LGSTATE_GOPULSE: x=%x,y=%x,freq=%d",priv->lg_gopulse.xy_curpt.xdata,
             priv->lg_gopulse.xy_curpt.ydata, priv->lg_gopulse.poll_freq);
      printk(KERN_INFO ", cntr=%d,onval=%d,offval=%d",priv->lg_gopulse.counter,
      	     priv->lg_gopulse.on_val,priv->lg_gopulse.off_val); 
      return(0);
}
static int lg_proc_disp_cmd(struct cmd_rw_dispdata *p_cmd_disp, struct lg_dev *priv)
{
    struct lg_xydata   xydata;
    
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    priv->lg_state    = LGSTATE_IDLE;
    if (p_cmd_disp->hdr.length != sizeof(struct lg_disp_data))
      return(-EINVAL);
    lg_qc_flag = 0;   /* set quick check flag to "false" */
    if (!p_cmd_disp->dispdata.is_restart)
      {
	priv->lg_display.start_index = p_cmd_disp->dispdata.start_index;
	priv->lg_display.cur_index = p_cmd_disp->dispdata.start_index;
	priv->lg_display.nPoints = p_cmd_disp->dispdata.nPoints;
	if (priv->lg_display.nPoints > MAX_XYPOINTS)
	  return(-EINVAL);
      }
    priv->lg_display.poll_freq = p_cmd_disp->dispdata.poll_freq * 2000;
    priv->lg_state = LGSTATE_DISPLAY;
    priv->lg_ctrl2_store |= LASERENABLE;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    return(0);
}
static int lg_proc_move_cmd(struct cmd_rw_movedata *p_cmd_move, struct lg_dev *priv)
{
    struct lg_xydata   xydata;
    
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    switch(p_cmd_move->hdr.cmd)
      {
      case CMDW_DOSENSOR:
	priv->lg_state    = LGSTATE_IDLE;
	if (p_cmd_move->hdr.length != sizeof(struct lg_move_data))
	  return(-EINVAL);
	memset((char *)&priv->lg_sensor, 0, sizeof(struct lg_move_data));
	priv->lg_ctrl2_store |= LASERENABLE;
	outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
	priv->lg_sensor.xy_curpt.xdata = p_cmd_move->movedata.xy_curpt.xdata;
	priv->lg_sensor.xy_curpt.ydata = p_cmd_move->movedata.xy_curpt.ydata;
	priv->lg_sensor.xy_curpt.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
	priv->lg_sensor.xy_delta.xdata = p_cmd_move->movedata.xy_delta.xdata;
	priv->lg_sensor.xy_delta.ydata = p_cmd_move->movedata.xy_delta.ydata;
	// priv->lg_sensor.start_index = p_cmd_move->movedata.start_index; 
	priv->lg_sensor.start_index = 0;  // why is start_index other than zero for search?
	priv->lg_sensor.cur_index = priv->lg_sensor.start_index;
	priv->lg_sensor.nPoints = p_cmd_move->movedata.nPoints;
	priv->lg_sensor.poll_freq = p_cmd_move->movedata.poll_freq;
	priv->lg_sensor.do_coarse = p_cmd_move->movedata.do_coarse;
// debug
/*
printk(KERN_ERR "\nAGS-LG: sensor xy %x %x f %x del %x %x idx %x %x np %x"
     , priv->lg_sensor.xy_curpt.xdata
     , priv->lg_sensor.xy_curpt.ydata
     , priv->lg_sensor.xy_curpt.ctrl_flags
     , priv->lg_sensor.xy_delta.xdata
     , priv->lg_sensor.xy_delta.ydata
     , priv->lg_sensor.start_index
     , priv->lg_sensor.cur_index
     , priv->lg_sensor.nPoints
     );
*/
// debug
	if (!priv->lg_sensor.nPoints || (priv->lg_sensor.nPoints > MAX_TGFIND_BUFFER))
	  return(-EINVAL);
	// default sensor period is 30usec, but it takes a while to get data from
	// sense buffer.  old code used to access buffer via serial port (115200 baud, ie 86.8 usec)
	// Current hardware works better with returning sense data using 400 usec delay.
	if (!priv->lg_sensor.poll_freq)
	  priv->lg_sensor.poll_freq = KETIMER_30U;
	else
	  priv->lg_sensor.poll_freq = priv->lg_sensor.poll_freq * 2;
	// Write to current location in dark to fix up ghost beam
	priv->lg_state    = LGSTATE_SENSE;
	break;
      case CMDW_DODARKMOVE:
	priv->lg_state    = LGSTATE_IDLE;
	if (p_cmd_move->hdr.length != sizeof(struct lg_move_data))
	  return(-EINVAL);
	memset((char *)&priv->lg_darkmove, 0, sizeof(struct lg_move_data));
	priv->lg_ctrl2_store &= LASERDISABLE;
	outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
	priv->lg_darkmove.xy_curpt.xdata = p_cmd_move->movedata.xy_curpt.xdata;
	priv->lg_darkmove.xy_curpt.ydata = p_cmd_move->movedata.xy_curpt.ydata;
	priv->lg_darkmove.xy_curpt.ctrl_flags = 0;
	priv->lg_darkmove.xy_delta.xdata = p_cmd_move->movedata.xy_delta.xdata;
	priv->lg_darkmove.xy_delta.ydata = p_cmd_move->movedata.xy_delta.ydata;
	priv->lg_darkmove.start_index = p_cmd_move->movedata.start_index;
	priv->lg_darkmove.cur_index = priv->lg_darkmove.start_index;
	priv->lg_darkmove.nPoints = p_cmd_move->movedata.nPoints;
	priv->lg_darkmove.poll_freq = p_cmd_move->movedata.poll_freq;
	if (!priv->lg_darkmove.poll_freq)
	  priv->lg_darkmove.poll_freq = KETIMER_150U;
	// Write first set of points now, let event handler do rest
	priv->lg_darkmove.xy_curpt.ctrl_flags = 0;  // Force ctrl-flags to 0 for dark-move, just in case caller forgot to
	lg_write_io_to_dac(priv, &priv->lg_darkmove.xy_curpt);
	// Update last, prep for next set of points
	priv->lg_lastxy.xdata = priv->lg_darkmove.xy_curpt.xdata;
	priv->lg_lastxy.ydata = priv->lg_darkmove.xy_curpt.ydata;
	priv->lg_lastxy.ctrl_flags = priv->lg_darkmove.xy_curpt.ctrl_flags;
	lg_adjust_xypoints(&priv->lg_darkmove);
	priv->lg_state    = LGSTATE_DARKMOVE;
	break;
      case CMDW_DOLITEMOVE:
	priv->lg_state    = LGSTATE_IDLE;
	if (p_cmd_move->hdr.length != sizeof(struct lg_move_data))
	  return(-EINVAL);
	memset((char *)&priv->lg_litemove, 0, sizeof(struct lg_move_data));
	priv->lg_ctrl2_store |= LASERENABLE;
	outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
	priv->lg_litemove.xy_curpt.xdata = p_cmd_move->movedata.xy_curpt.xdata;
	priv->lg_litemove.xy_curpt.ydata = p_cmd_move->movedata.xy_curpt.ydata;
	priv->lg_litemove.xy_curpt.ctrl_flags = BRIGHTBEAMISSET | BEAMONISSET | LASERENBISSET;
	priv->lg_litemove.xy_delta.xdata = p_cmd_move->movedata.xy_delta.xdata;
	priv->lg_litemove.xy_delta.ydata = p_cmd_move->movedata.xy_delta.ydata;
	priv->lg_litemove.start_index = p_cmd_move->movedata.start_index;
	priv->lg_litemove.cur_index = priv->lg_litemove.start_index;
	priv->lg_litemove.nPoints = p_cmd_move->movedata.nPoints;
	priv->lg_litemove.poll_freq = p_cmd_move->movedata.poll_freq;
	if (!priv->lg_litemove.poll_freq)
	  priv->lg_litemove.poll_freq = KETIMER_150U;
	priv->lg_litemove.poll_freq *= 1000;
	// Write to first location get rid of trails
	xydata.xdata = priv->lg_litemove.xy_curpt.xdata;
	xydata.ydata = priv->lg_litemove.xy_curpt.ydata;
	lg_write_io_to_dac(priv, &xydata);
	udelay(2);
	// Write to first location, let event handler do rest
	lg_write_io_to_dac(priv, &priv->lg_litemove.xy_curpt);
	// Update last, prep for next set of points
	priv->lg_lastxy.xdata = priv->lg_litemove.xy_curpt.xdata;
	priv->lg_lastxy.ydata = priv->lg_litemove.xy_curpt.ydata;
	priv->lg_lastxy.ctrl_flags = priv->lg_litemove.xy_curpt.ctrl_flags;
	lg_adjust_xypoints(&priv->lg_litemove);
	priv->lg_state    = LGSTATE_LITEMOVE;
	break;      
      default:
	return(-EINVAL);
      }
    return(0);
}
static int lg_proc_cmd(struct cmd_rw *p_cmd_data, struct lg_dev *priv)
{
  if (!priv)
    return(-ENODEV);

  switch(p_cmd_data->base.hdr.cmd) {
  case CMDW_SETQCCOUNTER:
    if (p_cmd_data->base.hdr.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_qc_counter = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_STOP:
    priv->lg_state = LGSTATE_IDLE;
    //    priv->lg_ctrl2_store &= LASERDISABLE;
    //    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_ROIOFF:
    lg_roi_test  = 0;
    lg_roi_on    = 0;
    lg_roi_dwell = 6;
    break;
  case CMDW_QUICKCHECK:
    priv->lg_state    = LGSTATE_IDLE;
    priv->lg_display.start_index = 0;  /* reset from beginning */
    priv->lg_ctrl2_store |= LASERENABLE;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    memset((char *)&priv->lg_delta, 0, sizeof(struct lg_xydelta));
    priv->lg_state    = LGSTATE_DISPLAY;
    break;
  case CMDW_SETDELTA:
    if (p_cmd_data->base.hdr.length != sizeof(struct lg_xydata))
      return(-EINVAL);
    priv->lg_delta.xdata = p_cmd_data->base.xydata.xdata;
    priv->lg_delta.ydata = p_cmd_data->base.xydata.ydata;
    break;
  case CMDW_GOANGLE:
    priv->lg_state    = LGSTATE_IDLE;
    if (p_cmd_data->base.hdr.length != sizeof(struct lg_xydata))
      return(-EINVAL);
    /* Disable state machine, PREPPING FOR NEXT COMMAND SEQUENCE */
    priv->lg_lastxy.xdata = priv->lg_goangle.xdata = p_cmd_data->base.xydata.xdata;
    priv->lg_lastxy.ydata = priv->lg_goangle.ydata = p_cmd_data->base.xydata.ydata;
    priv->lg_lastxy.ctrl_flags = priv->lg_goangle.ctrl_flags = p_cmd_data->base.xydata.ctrl_flags;
    lg_write_io_to_dac(priv, (struct lg_xydata *)&priv->lg_lastxy);
    break;
  case CMDW_SETROI:
    if (p_cmd_data->base.hdr.length != sizeof(uint32_t))
      return(-EINVAL);
    lg_roi_del = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_SETQCFLAG:
    if (p_cmd_data->base.hdr.length != sizeof(uint32_t))
      return(-EINVAL);
    // When a quick check is needed, this flag is set to non-zero
    // LGDISPLAY  automatically sets this to zero
    lg_qc_flag = p_cmd_data->base.dat32.val32;
    break;
  case CMDW_READYLEDON:
    priv->lg_ctrl2_store |= RDYLEDON;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_READYLEDOFF:
    priv->lg_ctrl2_store &= RDYLEDOFF;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_SEARCHBEAMON:
    priv->lg_ctrl2_store |= LASERENABLE;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_SEARCHBEAMOFF:
    priv->lg_ctrl2_store &= DIMBEAM;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_LINKLEDOFF:
    priv->lg_ctrl2_store &= LNKLEDOFF;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_LINKLEDON:
    priv->lg_ctrl2_store |= LNKLEDON;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_SETSHUTENB:
    priv->lg_ctrl2_store |= LASERENABLE;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_CLEARSHUTENB:
    priv->lg_ctrl2_store &= LASERDISABLE;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  case CMDW_STOPCMD:
    priv->lg_state = LGSTATE_IDLE;
    lg_roi_test  = 0;
    lg_roi_on    = 0;
    lg_roi_dwell = 6;
    lg_qc_counter = -1;
    // Turn off READY LED & LASER
    priv->lg_ctrl2_store &= RDYLEDOFF;
    priv->lg_ctrl2_store &= LASERDISABLE;
    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
    break;
  default:
    printk(KERN_ERR "\nAGS-LG: CMDW %d option not found", p_cmd_data->base.hdr.cmd);
    break;
  }
  return(0);
};
ssize_t lg_write(struct file *file, const char __user *buffer, size_t count, loff_t *f_pos)
{
  struct lg_dev           *priv;
  struct cmd_rw           *cmd_data;
  struct lg_xydata        *pXYData;
  struct cmd_hdr          *pHdr;
  struct cmd_rw_movedata  *pMoveCmd;
  struct cmd_rw_dispdata  *pDispCmd;
  struct cmd_rw_pulsedata *pPulseCmd;
  int                     i, rc;

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

  if ((count <= 0)|| (count > sizeof(struct cmd_rw)))
    {
      kfree(cmd_data);
      return -EINVAL;
    }
  if(copy_from_user((char *)cmd_data, buffer, count))
    {
      kfree(cmd_data);
      return -EFAULT;
    }
  pHdr = (struct cmd_hdr *)cmd_data;
  // Validate command type
  if (!pHdr->cmd || (pHdr->cmd > CMD_LAST))
    {
      printk(KERN_ERR "\nAGS-LG: lg_write unknown command %d", pHdr->cmd);
      kfree(cmd_data);
      return(-EINVAL);
    }
  
  if (pHdr->cmd != CMDW_BUFFER)
    {
      switch(pHdr->cmd)
	{
	case CMDW_DOSENSOR:
	case CMDW_DODARKMOVE:
	case CMDW_DOLITEMOVE:
	  pMoveCmd = (struct cmd_rw_movedata *)cmd_data;
	  rc = lg_proc_move_cmd(pMoveCmd, priv);
	  break;
	case CMDW_STARTPULSE:
	  pPulseCmd = (struct cmd_rw_pulsedata *)cmd_data;
	  rc = lg_proc_pulse_cmd(pPulseCmd, priv);
	  break;
	case CMDW_DISPLAY:
	  pDispCmd = (struct cmd_rw_dispdata *)cmd_data;
	  rc = lg_proc_disp_cmd(pDispCmd, priv);
	  break;
	default:
	  rc = lg_proc_cmd(cmd_data, priv);
	  break;
	}
      kfree(cmd_data);
      return(rc);
    }

  if (!cmd_data->base.hdr.length || (cmd_data->base.hdr.length > (sizeof(struct cmd_rw)-offsetof(struct cmd_rw, xydata))))
    {
      kfree(cmd_data);
      return(-EINVAL);
    }

  // First lock driver so no more writes can occur and event handler will not
  // hiccup.
  // Set state to IDLE mode so timer event will stop display
  // Should not be trying to continue writing from lg_display_data
  // when reloading it!
  spin_lock(&priv->lock);
  // Clear out old display data
  memset((char *)&lg_display_data, 0, MAX_LG_BUFFER);
  if (cmd_data->base.hdr.length > MAX_LG_BUFFER)
    {
      printk(KERN_ERR "\nAGS-LG: LG_WRITE Buffer too big %d, must be less than %d",
	     cmd_data->base.hdr.length, priv->lg_disp_count);
      kfree(cmd_data);
      return(-EINVAL);
    }  
  // Write new display data
  memcpy((char *)&lg_display_data, (char *)&cmd_data->xydata[0], cmd_data->base.hdr.length);
  // Release device for incoming commands and read/write operations.
  spin_unlock(&priv->lock);
  if (cmd_data->base.hdr.test_mode == DO_TEST_DISPLAY)
    {
      lg_qc_flag = 0;   /* set quick check flag to "false" */
      for(i = 0; i < (cmd_data->base.hdr.length/sizeof(struct lg_xydata)); i++)
	{
	  pXYData = (struct lg_xydata *)&lg_display_data[i*sizeof(struct lg_xydata)];
	  lg_write_io_to_dac(priv, pXYData);
	}
    }

  kfree(cmd_data);
  return(count);
}

/******************************************************************
*                                                                 *
* lg_read()                                                       *
* Description:   This function is used to process read commands.  *
*                A buffer target-find data is sent back to user   *
*                or BUSY if in wrong mode or unable to obtain     *
*                device lock. Target find has 3 states.  The      *
*                only time user is allowed to get the target find *
*                data is in LGSTATE_SENSEREADY.                   *
*                LGSTATE_SENSE is for sending XY point to         *
*                DAC and then getting feedback data by reading    *
*                from the target-find ports for up to 250 usec    *
*                delay, depending on how refined the search is.   *
*                Once all sense data is collected, state goes     *
*                to LGSTATE_SENSEREADY.  Once user reads the      *
*                collected data, state goes back to LGSTATE_IDLE. *
*                                                                 *
******************************************************************/
ssize_t lg_read(struct file *file, char __user *buffer, size_t count, loff_t *f_pos)
{
    struct lg_dev *priv = file->private_data;
    
    if (!priv)
      return(-EBADF);
    
    if ((count<=0) || (count > MAX_TGFIND_BUFFER))
      return(-EINVAL);
    if (priv->lg_state != LGSTATE_SENSEREADY)
	return(-EBUSY);

    /* sensor scan has been done */
    if (copy_to_user(buffer, &tgfind_word[0], count))
      return(-EFAULT);
    priv->lg_state = LGSTATE_IDLE;
    return(count);
}

  /* the messy catch-all driver routine to do all sorts of things */
long lg_ioctl(struct file *file, unsigned int cmd, unsigned long arg )
{
  struct lg_dev     *priv;
  void __user       *argp = (void __user *)arg;
  uint32_t          inp_val;
  
  priv = (struct lg_dev *)file->private_data;
  if (!priv)
    return(-EBADF);

  switch (cmd) {
  case LGGETANGLE:
    if (copy_to_user(argp, &priv->lg_lastxy, sizeof(struct lg_xydata) ))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
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
    inp_val = inb(LG_IO_CNTRL2);
    if (copy_to_user(argp, &inp_val, sizeof(uint32_t)))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
    break;
  case LGGETEVENTTIMES:
    if (copy_to_user(argp, &priv->last_events, sizeof(struct event_times)))
      {
	printk(KERN_ERR "\nAGS-LG:Error occurred for message %x from user",cmd);
        return(-EFAULT);
      }
    break;
  case LGGETFPGAVERSION:
    inp_val = (inb(LG_FPGA_REV1_IO) << 8) + inb(LG_FPGA_REV2_IO);
    if (copy_to_user(argp, &inp_val, sizeof(uint32_t)))
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

static enum hrtimer_restart lg_evt_hdlr(struct hrtimer *timer)
{
#if 1
    struct timeval   start_ts;
    struct timeval   end_ts;
#endif
    struct lg_xydata *xydata;
    struct lg_dev    *priv;
    uint16_t         tfword;
    uint8_t          tg_find_val0;
    uint8_t          tg_find_val1;
    uint8_t          b_optic;
    uint8_t          cur_flags;

    priv = container_of(timer, struct lg_dev, lg_timer);
    if (!priv)
      return HRTIMER_RESTART;

    // ACTUAL TIME CYCLE
#if 1
    do_gettimeofday(&start_ts);
    priv->last_events.last_gap_usec = start_ts.tv_usec - priv->last_ts.tv_usec;
    memcpy((void *)&priv->last_ts, (void *)&start_ts, sizeof(struct timeval));
#endif

    // Start of actual event handler
    //  Check CDRH and, if need be, change shutter state
    b_optic = inb(LG_IO_CNTRL2);
    if (b_optic & CDRHBITMASK)
      {
	if (lg_shutter_open == 0)
	  {
	    priv->lg_ctrl2_store |= LASERENABLE;
	    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
	  }
	lg_shutter_open = 1;
      }
    else
      {
	if (lg_shutter_open == 1)
	  {
	    priv->lg_ctrl2_store &= LASERDISABLE;
	    outb(priv->lg_ctrl2_store, LG_IO_CNTRL2);
	  }
	lg_shutter_open = 0;
      }
    switch(priv->lg_state)
      {
      case LGSTATE_SENSE:
	// Write current xy pair to DAC, update last-xy for getangle queries,
	// If not end of query:
	//      1. set calculate new xy pair for next round.
	//      2. wait for valid data.
	lg_write_io_to_dac(priv, &priv->lg_sensor.xy_curpt);
	// Update last just in case user sends get-angle command
	priv->lg_lastxy.xdata = priv->lg_sensor.xy_curpt.xdata;
	priv->lg_lastxy.ydata = priv->lg_sensor.xy_curpt.ydata;
	priv->lg_lastxy.ctrl_flags = priv->lg_sensor.xy_curpt.ctrl_flags;
	if ((priv->lg_sensor.cur_index >= MAX_TGFIND_BUFFER) ||
	    ((priv->lg_sensor.cur_index - priv->lg_sensor.start_index) >= priv->lg_sensor.nPoints))
	  priv->lg_state = LGSTATE_SENSEREADY;
	else
	  {
	    // Calculate new data point for next round
	    lg_adjust_xypoints(&priv->lg_sensor);
	    // If in coarse search, wait up to 250 usec for good data
	    // else we're in fine mode & wait only up to 5 usec.
	    // When kind of close to target it takes up to 250 usec to start
	    // to see a change in sensor data.  When on target it only takes
	    // 3-4 usec to see a change in sensor data.
	    priv->lg_sensor.poll_freq = SENSOR_READ_FREQ;
	    priv->lg_state = LGSTATE_SENSEREAD;
	  }
	// Restart timer to continue working on data until end of sensor pairs
	// hrtimer_forward_now(&priv->lg_timer, ktime_set(0, 1000U*(priv->lg_sensor.poll_freq)));
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, 2000U*(priv->lg_sensor.poll_freq)));
	return(HRTIMER_RESTART);
	break;
      case LGSTATE_SENSEREAD:
	if ((priv->lg_sensor.cur_index < MAX_TGFIND_BUFFER) &&
	    ((priv->lg_sensor.cur_index - priv->lg_sensor.start_index) <= priv->lg_sensor.nPoints))
	  {
	    // Read in light-sensor info.
	    // NOTE:  Data from sensor logic is inverted by the FPGA.
	    tg_find_val1 = inb(TFPORTRL);
	    tg_find_val0 = inb(TFPORTRH) & 0x03;
	    tfword =  (tg_find_val0 << 8) | tg_find_val1;
// debug
/*
if ( priv->lg_sensor.cur_index == 3 ) {
printk(KERN_ERR "\nAGS-LG: tgfind xy %x %x  val %x", priv->lg_lastxy.xdata, priv->lg_lastxy.ydata, tfword);
}
*/
// debug
	    tgfind_word[priv->lg_sensor.cur_index++] = tfword;
	    priv->lg_sensor.poll_freq = SENSOR_WRITE_FREQ;
	    priv->lg_state = LGSTATE_SENSE;
	  }
	// Restart timer to continue working on data until end of sensor pairs
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, 1000U*(priv->lg_sensor.poll_freq)));
	return(HRTIMER_RESTART);
	break;
      case LGSTATE_DARKMOVE:
	// Write current point to DAC.
	lg_write_io_to_dac(priv, &priv->lg_darkmove.xy_curpt);
	// Update last just in case user sends get-angle command
	priv->lg_lastxy.xdata = priv->lg_darkmove.xy_curpt.xdata;
	priv->lg_lastxy.ydata = priv->lg_darkmove.xy_curpt.ydata;
	// Calculate new data point for next round
	if ((priv->lg_darkmove.cur_index - priv->lg_darkmove.start_index) < priv->lg_darkmove.nPoints)
	  {
	    // Calculate new data point for next round
	    lg_adjust_xypoints(&priv->lg_darkmove);
	    priv->lg_darkmove.cur_index++;
	  }
	else
	  priv->lg_state = LGSTATE_IDLE;
	// Restart timer to continue working on data until user-app suspends work
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->lg_darkmove.poll_freq));
	return(HRTIMER_RESTART);
	break;
      case LGSTATE_LITEMOVE:
	// Write current point to DAC.
	lg_write_io_to_dac(priv, (struct lg_xydata *)&priv->lg_litemove.xy_curpt);
	// Update last just in case user sends get-angle command
	// Update last just in case user sends get-angle command
	priv->lg_lastxy.xdata = priv->lg_litemove.xy_curpt.xdata;
	priv->lg_lastxy.ydata = priv->lg_litemove.xy_curpt.ydata;
	priv->lg_lastxy.ydata = priv->lg_litemove.xy_curpt.ctrl_flags;
	// Calculate new data point for next round
	if ((priv->lg_litemove.cur_index - priv->lg_litemove.start_index) < priv->lg_litemove.nPoints)
	  {
	    // Calculate new data point for next round
	    lg_adjust_xypoints(&priv->lg_litemove);
	    priv->lg_litemove.cur_index++;
	  }
	else
	  priv->lg_state = LGSTATE_IDLE;
	// Restart timer to continue working on data until user-app suspends work
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->lg_litemove.poll_freq));
	return(HRTIMER_RESTART);
	break;
      case LGSTATE_DISPLAY:
	// Writes xy-pair to DAC
	// Check for end of buffer, start over when end is reached
	if ((priv->lg_display.cur_index >= priv->lg_display.nPoints) ||
	    (priv->lg_display.cur_index >= MAX_XYPOINTS))
	  priv->lg_display.cur_index = 0;
	xydata = (struct lg_xydata *)&lg_display_data[priv->lg_display.cur_index];
	// Write XY pair to DAC
	if (priv->lg_display.cur_index == 0)
	  {
	    // Starting over, move XY to new location in dark
	    cur_flags = xydata->ctrl_flags;
	    xydata->ctrl_flags = 0;
	    lg_write_io_to_dac(priv, xydata);
	    xydata->ctrl_flags = cur_flags;
	    udelay(2);
	  }
	lg_write_io_to_dac(priv, xydata);
	// Save current XY point
	priv->lg_lastxy.xdata = xydata->xdata;
	priv->lg_lastxy.ydata = xydata->ydata;
	priv->lg_lastxy.ctrl_flags = xydata->ctrl_flags;
	priv->lg_display.cur_index++;
	/* a negative lg_qc_counter will never do a quick check */
	if (lg_qc_counter > 0)
	  lg_qc_counter--;   /* decrement counter */
	else if (lg_qc_counter == 0)
	  {
	    lg_qc_flag = 1;   /* set the quick check flag */
	    priv->lg_state = LGSTATE_IDLE;      /*  go into the idle state  */
	  }
	// Restart timer to continue working on data until user-app suspends work
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->lg_display.poll_freq));
	return(HRTIMER_RESTART);
	break;
      case LGSTATE_GOPULSE:
	// first, the sanity checks (turn off pulse mode)
	if ((priv->lg_gopulse.counter > 16384) || (priv->lg_gopulse.counter < 0)
	    || (priv->lg_gopulse.on_val > 16384) || (priv->lg_gopulse.on_val < 0)
	    || (priv->lg_gopulse.off_val > 16384) || (priv->lg_gopulse.off_val < 0))
	  {
	    priv->lg_state = LGSTATE_IDLE;
	    // Restart timer for idle-mode no-op
	    hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->poll_frequency));
	    return(HRTIMER_RESTART);
	  }
	// Now get to the business of turning the beam on and off
	priv->lg_gopulse.counter++;
	if ((priv->lg_gopulse.counter <= priv->lg_gopulse.off_val)
	    || (priv->lg_gopulse.counter <= priv->lg_gopulse.on_val))
	  {
	    printk(KERN_CRIT "\nAGS-LG GOPULSE: xdata %x,ydata %x,ctrl_flags %x",
		   priv->lg_gopulse.xy_curpt.xdata,priv->lg_gopulse.xy_curpt.ydata,
		   priv->lg_gopulse.xy_curpt.ctrl_flags);
	    lg_write_io_to_dac(priv, (struct lg_xydata *)&priv->lg_gopulse.xy_curpt);
	    // Update last just in case user sends get-angle command
	    priv->lg_lastxy.xdata = priv->lg_gopulse.xy_curpt.xdata;
	    priv->lg_lastxy.ydata = priv->lg_gopulse.xy_curpt.ydata;
	    priv->lg_lastxy.ctrl_flags = priv->lg_gopulse.xy_curpt.ctrl_flags;
	  }
	else
	  priv->lg_gopulse.counter = 0;

	// Restart timer to continue working on data until user-app suspends work
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->lg_gopulse.poll_freq));
	return(HRTIMER_RESTART);
	break;
      case LGSTATE_IDLE:
      case LGSTATE_SENSEREADY:
      default:
	// Restart timer to continue working on data until user-app suspends work
	hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->poll_frequency));
	return(HRTIMER_RESTART);
	break;
      }
#if 1
    do_gettimeofday(&end_ts);
    priv->last_events.last_exec_usec = end_ts.tv_usec - start_ts.tv_usec;
#endif
    // Default is to use the poll-frequency set by setclock or default value.
    //Restart timer to continue working on data until user-app suspends work
    hrtimer_forward_now(&priv->lg_timer, ktime_set(0, priv->poll_frequency));
    return(HRTIMER_RESTART);
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

static const struct file_operations lg_ttyS1fops = {
  .owner            = THIS_MODULE,
  .read             = LG_SerialRead1,  /* read */
  .write            = LG_SerialWrite1, /* write */
  .open             = lg_open,        /* open */
  .release          = lg_release,     /* release */
};

static const struct file_operations lg_ttyS2fops = {
  .owner            = THIS_MODULE,
  .read             = LG_SerialRead2,  /* read */
  .write            = LG_SerialWrite2, /* write */
  .open             = lg_open,        /* open */
  .release          = lg_release,     /* release */
};

struct miscdevice lg_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = DEV_NAME,
  .fops = &lg_fops,
};

struct miscdevice lg_ttyS1device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = DEV_NAME_LGTTYS1,
  .fops = &lg_ttyS1fops,
};

struct miscdevice lg_ttyS2device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = DEV_NAME_LGTTYS2,
  .fops = &lg_ttyS2fops,
};

static void serial_port_init(void)
{    //LGTTYS1 Front end serial port: 115200, N, 8, 1, no 'rupts, force DTR and RTS
  LG_SerialWrite(LG_TTYS1_BASE, UART_IER, 0);    // 'rupts off
  LG_SerialWrite(LG_TTYS1_BASE, UART_LCR, UART_LCR_DLAB); //DLAB = 1
  LG_SerialWrite(LG_TTYS1_BASE, UART_DLL, 0x1 ); // DLL = 0x1 
  LG_SerialWrite(LG_TTYS1_BASE, UART_DLM, 0x0);  // DLM = 0x0
  LG_SerialWrite(LG_TTYS1_BASE, UART_LCR, 0x3);  // DLAB = 0, 8bits, no parity, 1 stop bit
  LG_SerialWrite(LG_TTYS1_BASE, UART_FCR, 0xc7); // FCR = 14 bytes 
  LG_SerialWrite(LG_TTYS1_BASE, UART_MCR, 0x0); // MCR = 

  //LGTTYS2 Laser Control Board serial port: 115200, N, 8, 1, no 'rupts, force DTR and RTS
  LG_SerialWrite(LG_TTYS2_BASE, UART_IER, 0);    // 'rupts off
  LG_SerialWrite(LG_TTYS2_BASE, UART_LCR, UART_LCR_DLAB); //DLAB = 1
  LG_SerialWrite(LG_TTYS2_BASE, UART_DLL, 0x1 ); // DLL = 0x1 
  LG_SerialWrite(LG_TTYS2_BASE, UART_DLM, 0x0);  // DLM = 0x0
  LG_SerialWrite(LG_TTYS2_BASE, UART_LCR, 0x3);  // DLAB = 0, 8bits, no parity, 1 stop bit
  LG_SerialWrite(LG_TTYS2_BASE, UART_FCR, 0xc7); // FCR = 14 bytes 
  LG_SerialWrite(LG_TTYS2_BASE, UART_MCR, 0x0); // MCR =
  return;
} 

static int laser_dev_init(struct lg_dev *lg_devp)
{
    struct lg_xydata xydata;

    if (!lg_devp)
      return(-EINVAL);
  
    // DEFAULT event timer poll frequency is 75 usec, but need to
    // increase by 3000 instead of 1000 to get timing right for new
    // laservision product.
    lg_devp->poll_frequency = KETIMER_75U * 3000;
    hrtimer_init(&lg_devp->lg_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    lg_devp->lg_timer.function = lg_evt_hdlr;
    hrtimer_start(&lg_devp->lg_timer, ktime_set(0, lg_devp->poll_frequency), HRTIMER_MODE_REL);
    do_gettimeofday(&lg_devp->last_ts);

    /* move to 0,0 position, laser disabled, beam off */
    memset((char *)&xydata, 0, sizeof(struct lg_xydata));
    lg_write_io_to_dac(lg_devp, &xydata);

    // Start in idle state so timer handler does nothing
    lg_devp->lg_state = LGSTATE_IDLE;
    lg_devp->lg_disp_count = MAX_XYPOINTS * sizeof(struct lg_xydata);
  
    // Initialize buffers to 0
    memset((char *)&tgfind_word, 0, sizeof(tgfind_word));
    memset((char *)&lg_display_data,0, lg_devp->lg_disp_count);

    // Start with READY-LED ON TO INDICATE NOT READY TO RUN.
    lg_devp->lg_ctrl2_store = RDYLEDON;
    outb(lg_devp->lg_ctrl2_store, LG_IO_CNTRL2);
    return(0);
}

static int lg_dev_probe(struct platform_device *plat_dev)
{
    struct lg_dev *lg_devp;
    struct device *this_device;
    int           rc;
    uint8_t       fpga_version1;
    uint8_t       fpga_version2;

    // Check FPGA REV
    fpga_version1 = inb(LG_FPGA_REV1_IO);
    fpga_version2 = inb(LG_FPGA_REV2_IO);
    printk(KERN_INFO "AGS-LG:  FPGA version %02d.%02d",fpga_version1,fpga_version2);
    if ((fpga_version1 != FPGA_VERSION1) || (fpga_version2 != FPGA_VERSION2))
      return(-EIO);

    // allocate mem for struct device will work with
    lg_devp = kzalloc(sizeof(struct lg_dev), GFP_KERNEL);
    if (!lg_devp)
      {
	pr_err("kzalloc() failed for laser device struct\n");
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

    // Setup lgttyS1 device
    lg_devp->lgttyS1.minor = lg_ttyS1device.minor;
    lg_devp->lgttyS1.name = DEV_NAME_LGTTYS1;
    lg_devp->lgttyS1.fops = lg_ttyS1device.fops;
    rc = misc_register(&lg_devp->lgttyS1);
    if (rc)
      {
	printk(KERN_ERR "AGS-LG:  Failed to register Laser lgttyS1 device, err %d \n", rc);
	kfree(lg_devp);
	return(rc);
      }

    this_device = lg_devp->lgttyS1.this_device;
    lg_devp->lgttyS1.parent = lg_devp->dev;
    dev_set_drvdata(this_device, lg_devp);
    platform_set_drvdata(plat_dev, lg_devp);
    printk(KERN_INFO "\nAGS-LG: Device lgttyS1 created\n");

    // Obtain IO space for lgttyS1 device
    if (!request_region(LG_TTYS1_BASE, LG_TTYS1_REGION, DEV_NAME_LGTTYS1))
      {
	kfree(lg_devp);
	misc_deregister(&lg_devp->lgttyS1);
	printk(KERN_CRIT "\nUnable to get IO regs for device lgttyS1");
	return(-EBUSY);
      }

    // Setup lgttyS2 device
    lg_devp->lgttyS2.minor = lg_ttyS2device.minor;
    lg_devp->lgttyS2.name = DEV_NAME_LGTTYS2;
    lg_devp->lgttyS2.fops = lg_ttyS2device.fops;
    rc = misc_register(&lg_devp->lgttyS2);
    if (rc)
      {
	printk(KERN_ERR "AGS-LG:  Failed to register Laser lgttyS2 device, err %d \n", rc);
	kfree(lg_devp);
	return(rc);
      }

    this_device = lg_devp->lgttyS2.this_device;
    lg_devp->miscdev.parent = lg_devp->dev;
    dev_set_drvdata(this_device, lg_devp);
    platform_set_drvdata(plat_dev, lg_devp);
    printk(KERN_INFO "\nAGS-LG: Device lgttyS2 created\n");

    // Obtain IO space for lgttyS2 device
    if (!request_region(LG_TTYS2_BASE, LG_TTYS2_REGION, DEV_NAME_LGTTYS2))
      {
	kfree(lg_devp);
	misc_deregister(&lg_devp->lgttyS2);
	printk(KERN_CRIT "\nUnable to get IO regs for device lgttyS2");
	return(-EBUSY);
      }

    //Initialize serial ports
    serial_port_init();
    rc = laser_dev_init(lg_devp);
    if (rc)
      {
	printk(KERN_ERR "AGS-LG:  Failed to initialize Laser device, err %d \n", rc);
	kfree(lg_devp);
	return(rc);
      }
      
    // All initialization done, so enable timer
    printk(KERN_INFO "\nAGS-LG: Laser boardcomm Devices installed, initialized and timer created.\n");
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
  hrtimer_cancel(&lg_devp->lg_timer);
  release_region(LG_BASE, LASER_REGION);
  release_region(LG_TTYS1_BASE, LG_TTYS1_REGION);
  release_region(LG_TTYS2_BASE, LG_TTYS2_REGION);
  misc_deregister(&lg_devp->miscdev);
  misc_deregister(&lg_devp->lgttyS1);
  misc_deregister(&lg_devp->lgttyS2);
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
