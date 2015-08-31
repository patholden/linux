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
#include <linux/major.h>
#include <linux/laserguide.h>
#include <linux/timer.h>
#include <linux/interrupt.h>

unsigned short lg_out_data[MAX_LG_BUFFER];
unsigned char  diode_data[MAX_DIODE_BUFFER];
unsigned short diode_word[MAX_DIODE_BUFFER];
long  lg_out_data_index = 0;
long  lg_out_data_start = 0;
long  lg_out_data_end   = 0;
long  lg_out_data_max   = 0;
long  lg_in_data_index = 0;
long  lg_in_data_end   = 0;
long  lg_move = 0;
long  lg_trigger = 0;
long  lg_qc_flag = 0;
long  lg_qc_counter = 0;
long  lg_hw_flag = 0;
long  lg_hw_trigger = 0;
long  lg_roi_test   = 0;
long  lg_roi_dwell  = 0;
long  lg_roi_on     = 0;
long  lg_roi_del    = 0;
long  lg_roi_first  = 0;
long  lg_roi_search = 0;
long  lg_debounce  = 0;
long  lg_hit       = 0;
long  lg_trig_flag = 0;
long  lg_step_neg  = 0;
long  lg_index_avg = 0;
long  lg_pulsemodeflag   = 0;
long  lg_pulsecounter    = 0;
long  lg_pulseonvalue    = 0;
long  lg_pulseoffvalue   = 0;

unsigned long  lg_num_points = 0;

unsigned long  lg_x_save = 0;
unsigned long  lg_y_save = 0;
unsigned long  lg_x_delta = 0;
unsigned long  lg_y_delta = 0;

unsigned short  lg_xhi_offset = 0;
unsigned short  lg_yhi_offset = 0;

unsigned char lg_threshold = 0;
static int lg_out_data_size = 0;
static int lg_first_flag;
static int lg_shutter_open;

unsigned char lg_optic_store = 0;

struct wait_queue * lg_wait = NULL;
static void lg_timeout(unsigned long data);
static struct timer_list lg_timer =
	TIMER_INITIALIZER(lg_timeout , 0, 0);

int   lg_dark_search;

// Defines used by event timer
#define UARTPORT 0x3F8
#define UCOUNT   500

  /* note that everything must be written as "char"                */
  /* in one BIG write  (unless some sort of "seek" is implemented) */
/*
static int lg_write(struct inode * inode, struct file * file,
  const char * buffer, int count)
 */
ssize_t lg_write( struct file * file
                , const char __user * buffer
                , size_t count
                , loff_t * f_pos
                )
{

  if (count<=0) return -EINVAL;
  if (count > MAX_LG_BUFFER) return -ENOMEM;
  if(copy_from_user((void *)lg_out_data, (void *)buffer, (unsigned long)count))
    return -EFAULT;

  lg_out_data_size = count;
  lg_out_data_start = 0;
  lg_out_data_end = lg_out_data_size / 2;
  lg_out_data_max = lg_out_data_size / 2;
  lg_out_data_index = 0;
  return count;
}


  /* the routine might be used to get data */
  /* from a diode monitor buffer */
ssize_t lg_read( struct file * file
               , char __user * buffer
               , size_t count
               , loff_t *f_pos
               )
{
/*
  int minor = MINOR(inode->i_rdev);
 */

  if (count<=0) return -EINVAL;
  if (count > MAX_DIODE_BUFFER) return -ENOMEM;
  if( lg_move == 2 ) {
     return -EAGAIN;
  }

  if( lg_move <= 1 ) return -EBUSY;
  if( lg_move != 3 ) return -EIO; 

  /* sensor scan has been done */
  if (copy_to_user((void *)buffer, (void *)diode_word, (unsigned long)count))
    return -EFAULT;    
  lg_move = 0;    /* reset to IDLE state */
  return count;
}

  /* the messy catch-all driver routine to do all sorts of things */
static long lg_ioctl(struct file *file, unsigned int cmd,
 unsigned long param )
{
  union lg_info local_info;
  /* int retval; */
  unsigned short xlo,xhi,ylo,yhi;
  unsigned char lo;
  int timer_expiration=msecs_to_jiffies(1);

  if( _IOC_DIR(cmd) & _IOC_WRITE ) {
    copy_from_user(&local_info, (void *)param, sizeof(union lg_info) );
  }
  del_timer(&lg_timer);
  
  switch (cmd) {
     case LGSTOP:
       lg_move = 0;
       lg_hw_flag = 0;
       outb( 0x00, LG_IO_X0 );  /* strobe and turn off beam */
       outb( 0x80, LG_IO_X0 );
       lg_xhi_offset = 0;    /* also reset offsets */
       lg_yhi_offset = 0;
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1000Hz
       return 0;
     case LGDISPLAY:
       lg_qc_flag = 0;   /* set quick check flag to "false" */
       lg_move = 1;
       lg_out_data_index = 0;  /* reset from beginning */
       return 0;
     case LGHWFLAGON:
       // lg_hw_flag = 1;   /* set hardware flag to "true" */
       return 0;
     case LGHWFLAGOFF:
       // lg_hw_flag = 0;   /* set hardware flag to "false" */
       return 0;
     case LGHWTRIGGER:
       lg_hw_trigger = 1;   /* prime hardware trigger */
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1000Hz
       return 0;
     case LGDOSENSOR:  /* everything should be set up before this */
       lg_dark_search = 0;
       lg_move = 2;  /* not moved to read routine */
       lg_in_data_index = 0;
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1msec
       return 0;
     case LGDODARKSENS:  /* everything should be set up before this */
       lg_dark_search = 1;
       lg_move = 2;  /* not moved to read routine */
       lg_in_data_index = 0;
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1msec
       return 0;
     case LGQUICKCHECK:
       lg_move    = 1;
       lg_trigger = 1;
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1msec
       return 0;
     case LGSETDELTA:
       lg_x_delta = local_info.delta.xdel; 
       lg_y_delta = local_info.delta.ydel; 
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1msec
       return 0;
     case LGGOANGLE:
       lg_move = 0;      /* if running, disable LGDISPLAY mode */
       xlo = local_info.ptns.xpos & 0xFF7F;
       xhi = local_info.ptns.xpos >> 16;
       ylo = local_info.ptns.ypos & 0xFFFF;
       yhi = local_info.ptns.ypos >> 16;
       outw( xlo, LG_IO_X0 );
       outw( xhi, LG_IO_X2 );
       outw( ylo, LG_IO_Y0 );
       outw( yhi, LG_IO_Y2 );
       lo = (0xFF & xlo) | 0x80;   /* set strobe high */
       outb( lo, LG_IO_X0 );
                /* outb( 0x80,0x380 );  */
       lg_x_save = local_info.ptns.xpos; 
       lg_y_save = local_info.ptns.ypos; 
       return 0;
     case LGLOADWRITENUMBER:
       lg_out_data_end   = local_info.num_points * 4;
       lg_out_data_max   = local_info.num_points * 4;
       return 0;
     case LGLOADREADNUMBER:
       lg_in_data_end   = local_info.num_points;
       return 0;
     case LGLOADTHRESH:
       lg_threshold = local_info.threshold;
       return 0;
     case LGGETANGLE:
       local_info.ptns.xpos = lg_x_save; 
       local_info.ptns.ypos = lg_y_save; 
       break;
     case LGGETSERVO:
       local_info.servo_status = inb( LG_IO_SERVO );
       break;
     case LGLOADOPTIC:
       outb( local_info.optics_command, LG_IO_OPTIC );
       lg_optic_store = local_info.optics_status;
       break;
     case LGGETOPTIC:
       local_info.optics_status = inb( LG_IO_OPTIC );
       break;
     case LGGETSTORE:
       local_info.optics_status = lg_optic_store;
       if (lg_hw_flag == 1)
	 local_info.optics_status = 1;
       else
	 local_info.optics_status = 0;
       break;
     case LGFASTSCAN:
       lg_optic_store |= 0x80;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGSLOWSCAN:
       lg_optic_store &= 0x7F;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGSETSEARCHB:
       lg_optic_store |= 0x40;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGCLRSEARCHB:
       lg_optic_store &= 0xBF;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGSETSWITCHB:
       lg_optic_store |= 0x20;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGCLRSWITCHB:
       lg_optic_store &= 0xDF;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGSETLEDBIT:
       lg_optic_store |= 0x10;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGCLRLEDBIT:
       lg_optic_store &= 0xEF;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGSETLINKLED:
       lg_optic_store |= 0x04;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGCLRLINKLED:
       lg_optic_store &= 0xFB;
       outb( lg_optic_store, LG_IO_OPTIC );
       break;
     case LGGETBOARD:
       // local_info.board_config = inb( LG_IO_BOARD );
       if ( lg_hw_flag == 1 ) {
           local_info.board_config = 1;
       } else {
           local_info.board_config = 0;
       }
       break;
     case LGGETIP:
       local_info.low_ip_octet = inb( LG_IO_TCPIP );
       break;
     case LGFASTCLOCK:
       timer_expiration = msecs_to_jiffies(1);    // timer expires in ~1 msec
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer to ~1msec
       break;
     case LGSLOWCLOCK:
       timer_expiration = usecs_to_jiffies(local_info.clock_tick);    // timer expires in ~usecs
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer to ~1msec
       break;
     case LGSETCLOCK:
       // TBD, NOT CORRECT THOUGH
       break;
             /* when a quick check is needed, this flag is set to non-zero */
             /* LGDISPLAY  automatically sets this to zero  */
     case LGSETQCFLAG:
        lg_qc_flag = local_info.qcFlag;
       break;
     case LGGETQCFLAG:
        local_info.qcFlag = lg_qc_flag;
       break;
             /* this is a count-down and must be reset each time */
             /*  -1 is reserved for NO quick checks              */
     case LGSETQCCOUNTER:
        lg_qc_counter = local_info.qcCounter;
       break;
     case LGGETQCCOUNTER:
        local_info.qcCounter = lg_qc_counter;
       break;
     case LGSETXOFF:
        lg_xhi_offset = local_info.pos_offset;
       break;
     case LGSETYOFF:
        lg_yhi_offset = local_info.pos_offset;
       break;
     case LGROIOFF:
        lg_roi_test  = 0;
        lg_roi_on    = 0;
        lg_roi_dwell = 6;
        lg_debounce  = 0;
        lg_hit       = 0;
        lg_trig_flag = 0;
        lg_step_neg  = 0;
       break;
     case LGROION:
         if( (lg_roi_del <= 0) || (lg_roi_del >= lg_out_data_max) ) {
           lg_roi_del   = lg_out_data_max / 20;
           lg_roi_del   = lg_roi_del - (lg_roi_del % 4);
         }
         if( lg_roi_on ) {
          if( lg_index_avg < lg_roi_del ) {
            lg_out_data_start =
                            (lg_index_avg + lg_out_data_max) - lg_roi_del;
          } else {
            lg_out_data_start = (lg_index_avg - lg_roi_del);
          }
          if( (lg_index_avg + lg_roi_del) > lg_out_data_max ) {
            lg_out_data_end   =
                            (lg_index_avg + lg_roi_del) - lg_out_data_max;
          } else {
            lg_out_data_end   = (lg_index_avg + lg_roi_del);
          }
                        /* make sure of 4-byte alignment */
          lg_out_data_start = lg_out_data_start - (lg_out_data_start % 4);
          lg_out_data_end   = lg_out_data_end   - (lg_out_data_end % 4);
          if( lg_out_data_start < 4 ) lg_out_data_start = 4;
          if( lg_out_data_end   < 4 ) lg_out_data_end   = 4;
          if( lg_out_data_start > (lg_out_data_max-4) )
                             lg_out_data_start = lg_out_data_max-4;
          if( lg_out_data_end   > (lg_out_data_max-4) )
                             lg_out_data_end   = lg_out_data_max-4;
          lg_out_data_index = lg_index_avg;
         } else {
          lg_roi_test  = 1;
          lg_roi_on    = 0;
          lg_roi_dwell = 6;
          lg_debounce  = 0;
          lg_hit       = 0;
          lg_trig_flag = 0;
          lg_step_neg  = 0;
         }
       break;
     case LGSETROI:
        lg_roi_del = local_info.roi_points;
       break;
     case LGSETROISEARCH:
        lg_roi_search = 1;
       break;
     case LGCLRROISEARCH:
        lg_roi_search = 0;
       break;
     case LGSTARTPULSE:
        lg_pulsecounter  = 0;
        lg_pulsemodeflag = 1;
       break;
     case LGSTOPPULSE:
        lg_pulsemodeflag = 0;
       break;
     case LGSETPULSEON:
        lg_pulsemodeflag = 0;
        lg_pulseonvalue = local_info.pulseonvalue;
       break;
     case LGSETPULSEOFF:
        lg_pulsemodeflag = 0;
        lg_pulseoffvalue = local_info.pulseoffvalue;
       break;
     default:
       return -EINVAL;
  }

  if( _IOC_DIR(cmd) & _IOC_READ ) {
    copy_to_user((void *)param, &local_info, sizeof(union lg_info) );
    return 0;
  }
  else
    return 0;

  return -EINVAL;
}

static int lg_open(struct inode *inode, struct file *file)
{
  return 0;
}

int lg_release(struct inode *inode, struct file *file)
{
  return 0;
}

static void lg_timeout(unsigned long data)
{
  char time_lg_inbuff[8];
  static unsigned short xlo, xhi, ylo, yhi;
  static unsigned char lo;
  static unsigned char b_optic;
  unsigned char tg_find_val;
  unsigned short lg_sum;
  unsigned short lg_00x;
  unsigned short lg_x00;
  int timer_expiration;

  /*
   *  before anything else, check CHDR and, if need be,
   *  change shutter state
   *
   */
  b_optic = inb(LG_IO_OPTIC);
  if ( b_optic & 0x08 )
    {
      if ( lg_shutter_open == 0 )
	{
	  lg_optic_store |= 0x80;
	  outb( lg_optic_store, LG_IO_OPTIC );
	}
      lg_shutter_open = 1;
    }
  else
    {
      if (lg_shutter_open == 1)
	{
	  lg_optic_store &= 0x7F;
	  outb(lg_optic_store, LG_IO_OPTIC);
	}
      lg_shutter_open = 0;
    }

  // Set timer-expiration to 1msec
  timer_expiration = msecs_to_jiffies(1);  //Start with 1000Hz (1msec expiration)

  /*
   *  laser pulse mode
   *   if flag is on, do this and only this section
   */
  if ( lg_pulsemodeflag )
    {
      // first, the sanity checks (turn off pulse mode)
      if ( lg_pulsecounter  > 16384 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulsecounter  <     0 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseonvalue  > 16384 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseonvalue  <     0 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseoffvalue > 16384 )  { lg_pulsemodeflag = 0; }
      if ( lg_pulseoffvalue <     0 )  { lg_pulsemodeflag = 0; }
      // Now get to the business of turning the beam on and off
      del_timer(&lg_timer);
      lg_pulsecounter++;
      if ( lg_pulsecounter <= lg_pulseoffvalue )
	{
	  /* unset strobe and unblank bit */
	  xlo = lg_x_save & 0xFF3F;
	  xhi = lg_x_save >> 16;;
	  ylo = lg_y_save & 0xFFFF;
	  yhi = lg_y_save >> 16;
	  outw(xlo, LG_IO_X0);
	  outw(xhi, LG_IO_X2);
	  outw(ylo, LG_IO_Y0);
	  outw(yhi, LG_IO_Y2);
	  lo = (0xFF & xlo) | 0x80;               /* set strobe bit */
	  outb( lo, LG_IO_X0 );
	}
      else if (lg_pulsecounter <= lg_pulseonvalue)
	{
	  /* unset strobe bit */
	  xlo = lg_x_save & 0xFF7F;
	  /* set unblank bit */
	  xlo = xlo | 0x0040;
	  xhi = lg_x_save >> 16;;
	  ylo = lg_y_save & 0xFFFF;
	  yhi = lg_y_save >> 16;
	  outw(xlo, LG_IO_X0);
	  outw(xhi, LG_IO_X2);
	  outw(ylo, LG_IO_Y0);
	  outw(yhi, LG_IO_Y2);
	  lo = (0xFF & xlo) | 0x80;               /* set strobe bit */
	  outb(lo, LG_IO_X0);
	}
    else
      {
	lg_pulsecounter = 0;
      }
      mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, ~ 1000Hz
      
      lg_first_flag = 1;  /* kludge to set flag */
      /* lg_optic_store = 0x40; */
      return;
  }

  if (lg_move == 1)
    {
      del_timer(&lg_timer);
      lg_out_data_index += 4;
      if (lg_out_data_index >= lg_out_data_end) lg_out_data_index = 0;
      xlo = lg_out_data[lg_out_data_index  ] & 0xFF7F;
      /* unset strobe bit */
      xhi = lg_out_data[lg_out_data_index+1] + lg_xhi_offset;
      ylo = lg_out_data[lg_out_data_index+2];
      yhi = lg_out_data[lg_out_data_index+3] + lg_yhi_offset;
      outw(xlo, LG_IO_X0);
      outw(xhi, LG_IO_X2);
      outw(ylo, LG_IO_Y0);
      outw(yhi, LG_IO_Y2);
      lo = (0xFF & xlo) | 0x80;               /* set strobe bit */
      outb(lo, LG_IO_X0);
      lg_x_save = (((unsigned long)xhi) << 16) | (unsigned long)xlo;
      lg_y_save = (((unsigned long)yhi) << 16) | (unsigned long)ylo;    

      /* a negative lg_qc_counter will never do a quick check */
      if (lg_qc_counter > 0)
	lg_qc_counter--;   /* decrement counter */
      else if ( lg_qc_counter == 0 )
	{
	  lg_qc_flag = 1;   /* set the quick check flag */
	  lg_move = 0;      /*  go into the idle state  */

	  /* turn off beam */
	  lo = 0x00;                              /* unset strobe bit */
	  outb( lo, LG_IO_X0 );
	  lo = 0x80;                              /* set strobe bit */
	  outb( lo, LG_IO_X0 );

	  /* slow down clock to 10 millisec */
	  timer_expiration = msecs_to_jiffies(10);  // 10msec expiration
	}
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, either ~1 or ~10 msec
    }
  else if( lg_move == 4 )
    {
      /* this is intended for searching */
      del_timer(&lg_timer);

      if ( lg_dark_search == 1 )
	xlo = (unsigned short)((lg_x_save) & 0xFF00);    // keep beam dark
      else
	{
	  // set unblank, unset strobe
	  xlo = (unsigned short)((lg_x_save) & 0xFF7F);
	  xlo |=  0x0040;
	}
      xhi = (unsigned short)((lg_x_save >> 16) & 0xFFFF);
      ylo = (unsigned short)((lg_y_save      ) & 0xFFFF);
      yhi = (unsigned short)((lg_y_save >> 16) & 0xFFFF);
      /* NOTE: unblank bit disabled during search */
      outw( xlo, LG_IO_X0 );
      outw( xhi, LG_IO_X2 );
      outw( ylo, LG_IO_Y0 );
      outw( yhi, LG_IO_Y2 );
      if ( lg_dark_search == 1 )
	lo = (0xFF & xlo) | 0x80;    // set strobe bit, keep unblank unset
      else
	lo = (0xFF & xlo) | 0xC0;  // set strobe bit, keep unblank set
      
      outb( lo, LG_IO_X0 );
      diode_data[lg_in_data_index] = inb( LG_IO_OPTIC );
      lg_in_data_index++;   /* increment positions and index */
      lg_x_save      += lg_x_delta;
      lg_y_save      += lg_y_delta;
      if( lg_in_data_index >= lg_in_data_end )
	{
	  lg_move = 3;
	  timer_expiration = msecs_to_jiffies(10);   // timer will expire in ~ 10msec
	  lg_dark_search = 0;
	}
      
       mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer, either ~1 or ~10 msec
    }
  else if ( lg_move == 2 )
    {
      /* this is intended for searching */
      del_timer(&lg_timer);

      if ( lg_dark_search == 1 )
	{
	  /* keep beam dark */
	  xlo = (unsigned short)((lg_x_save) & 0xFF00);
	}
      else
	{
	  xlo = (unsigned short)((lg_x_save) & 0xFF7F);
	  xlo |=  0x0040;      /* set unblank, unset strobe */
	}
      xhi = (unsigned short)((lg_x_save >> 16) & 0xFFFF);
      ylo = (unsigned short)((lg_y_save      ) & 0xFFFF);
      yhi = (unsigned short)((lg_y_save >> 16) & 0xFFFF);
      /* NOTE: unblank bit disabled during search */
      outw( xlo, LG_IO_X0 );
      outw( xhi, LG_IO_X2 );
      outw( ylo, LG_IO_Y0 );
      outw( yhi, LG_IO_Y2 );
      /* set strobe bit, keep unblank set */
      if ( lg_dark_search == 1 )
	lo = 0x80;             // set strobe bit, keep unblank unset 
      else
	lo = 0xC0;           // set strobe bit, keep unblank set
      outb( lo, LG_IO_X0 );
      
      if ( lg_dark_search == 1 )
	lo = 0x00;             // clear strobe bit, keep unblank unset
      else
	lo = 0x40;             // clear strobe bit, keep unblank set 
      outb( lo, LG_IO_X0 );

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
	lg_x_save      += lg_x_delta;
	lg_y_save      += lg_y_delta;
	if ( lg_in_data_index >= lg_in_data_end )
	  {
	    lg_move = 3;
	    timer_expiration = msecs_to_jiffies(10);    // timer expires in 10 msec
	    lg_dark_search = 0;
	  }
    }
  mod_timer(&lg_timer, jiffies + timer_expiration);  // Restart timer to either 1msec or 10msec
  return;   /* skip the rest of the routine */
}

static const struct file_operations lg_fops = {
  .owner	    = THIS_MODULE,
  .read             =  lg_read,        /* lg_read */
  .write            = lg_write,       /* lg_write */
  .unlocked_ioctl   = lg_ioctl,       /* lg_ioctl */
  .open             = lg_open,        /* lg_open */
  .release          = lg_release,     /* lg_release */
};

static int __init lg_init(void )
{
   int eee;

   eee = register_chrdev(LG_MAJOR,"laser",&lg_fops);

   if ( eee ) {
     printk("Laser Guide Controller error: %d Cannot register to major device %d!\n", eee, LG_MAJOR);
     return eee;
   }

   request_region( LG_BASE, LASER_REGION, "laser" );

         /* move to 0,0 position */
   outw( 0, LG_IO_X0 );
   outw( 0, LG_IO_X2 );
   outw( 0, LG_IO_Y0 );
   outw( 0, LG_IO_Y2 );
   outb( 0x80, LG_IO_X0 );

   // Now set up the timer but don't start it yet.
   lg_timer.expires = 0;
   init_timer(&lg_timer);
   
   printk("Laser Guide Controller installed with major %d.\n", LG_MAJOR);
  return 0;
}

static void __exit lg_exit(void)
{
	unregister_chrdev(LG_MAJOR, "lg");
	release_region(LG_BASE, LASER_REGION);
}
module_init(lg_init);
module_exit(lg_exit);

MODULE_AUTHOR("Robert Luoma <rl@assemblyguide.com>");
MODULE_DESCRIPTION("Driver for AGS Laser Guidance System");
MODULE_LICENSE("GPL");

