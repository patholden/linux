/*---------------------------------------------------------------------------
 *
 * Copyright (c) 2015, Assembly Guidance Systems, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as 
 * published by the Free Software Foundation; either version 2 of 
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, 
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * The full text of the license may also be found at:        
 * http://opensource.org/licenses/GPL-2.0
 *
 *---------------------------------------------------------------------------
 */ 

//***************************************************************************
//*
//*  This file contains glue logic for Linux interface.  The purpose is
//*  to allow Linux to honor standard off-the-shelf commands and utilities
//*  wherever possible for interacting with the Congatec processor module.
//*
//***************************************************************************

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/nls.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <CgosInc/CgosDef.h>
#include <CgosInc/Cgos.h>
#include <CgosInc/Cgos_linux_if.h>
//#include <CgosInc/CgosDrv.h>
//#include <CgosInc/CgosBld.h>

int __CgosI2CTransfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
  //	unsigned long orig_jiffies;
  //	int ret, try;
  int ret, i2c_count;
  HCGOS    myCgos=0;
  
  // If board is currently busy, no sense in continuing
  ret = CgosBoardOpen(0, 0, 0, &myCgos);
  if (ret <0)
    return -EBUSY;
  
  // Make sure num is valid I2C device number
  i2c_count = CgosI2CCount(myCgos);
  if (num >= i2c_count)
    {
      CgosBoardClose(myCgos);
      return -ENODEV;
    }
  
  // FIXME PAT----start from here
#if 0
	// First make sure I2C command is supported
	if (adap->quirks && CgosI2cCheckForQuirks(adap, msgs, num))
		return -EOPNOTSUPP;
	/* i2c_trace_msg gets enabled when tracepoint i2c_transfer gets
	 * enabled.  This is an efficient way of keeping the for-loop from
	 * being executed when not needed.
	 */
	if (static_key_false(&i2c_trace_msg)) {
		int i;
		for (i = 0; i < num; i++)
			if (msgs[i].flags & I2C_M_RD)
			  trace_i2c_read(adap, &msgs[i], i);
			else
				trace_i2c_write(adap, &msgs[i], i);
	}
#endif

	// Set up for Cgos Transfer call here
	CgosBoardClose(myCgos);

#if 0
	/* Retry automatically on arbitration loss */
	orig_jiffies = jiffies;
	for (ret = 0, try = 0; try <= adap->retries; try++) {
		ret = adap->algo->master_xfer(adap, msgs, num);
		if (ret != -EAGAIN)
			break;
		if (time_after(jiffies, orig_jiffies + adap->timeout))
			break;
	}
	if (static_key_false(&i2c_trace_msg)) {
		int i;
		for (i = 0; i < ret; i++)
			if (msgs[i].flags & I2C_M_RD)
				trace_i2c_reply(adap, &msgs[i], i);
		trace_i2c_result(adap, i, ret);
	}
#endif
  
  return(0);
}
EXPORT_SYMBOL(CgosI2CTransfer);
