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
#include "CgosDef.h"
#include "Cgos.h"
#include "CgosBld.h"
#include "CgosIoct.h"

////GIANT FIXME
#if 0

//***************************************************************************
unsigned int dwLibInitCount=0;
void *hDriver=((void *)-1);
unsigned int dwDrvVersion=0;

unsigned int dwLastError=0;
unsigned int *pdwLastError=&dwLastError;

//***************************************************************************

#define ErrFALSE ((*pdwLastError=~0), FALSE)

//***************************************************************************
int CgosI2CTransfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
  //	unsigned long orig_jiffies;
  //	int ret, try;
  int ret, i2c_count;
  HCGOS    myCgos=0;
  
  // If board is currently busy, no sense in continuing
  ret = CgosCgebBoardOpen(0, 0, 0, &myCgos);
  if (ret <0)
    return -EBUSY;
  
  // Make sure num is valid I2C device number
  i2c_count = CgosCgebI2CCount(myCgos);
  if (num >= i2c_count)
    {
      CgosCgebBoardClose(myCgos);
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
	CgosCgebBoardClose(myCgos);

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

/*
  The flags are:
    0: return TRUE on success, FALSE on failure
    1: return the first return value pret0
    2: reserved
    4: allow the returned number of bytes in pout to be smaller then lenout
    8: pin and pout contain the length in the first ulong
*/

unsigned int CallCgebDrvPlain(unsigned int flags, unsigned int fct, unsigned int handle,
			  unsigned int type, unsigned int par0, unsigned int par1,
			  unsigned int par2, unsigned int par3, unsigned int *pret0, unsigned int *pret1)
{
  CGOSIOCTLIN cin;
#if 0
  CGOSIOCTLOUT cout;
  unsigned int cb;
#endif
  if (!dwLibInitCount) return ErrFALSE;
  cin.fct=fct;
  cin.handle=handle;
  cin.type=type;
  cin.pars[0]=par0;
  cin.pars[1]=par1;
  cin.pars[2]=par2;
  cin.pars[3]=par3;
//// FIXME---PAT Add cgeb_call() here.
#if 0
  if (!OsaDeviceIoControl(hDriver,CGOS_IOCTL,&cin,sizeof(cin),&cout,sizeof(cout),&cb)) return ErrFALSE;
  if (cb<sizeof(CGOSIOCTLOUT)) return ErrFALSE;
  if (cout.status) return (*pdwLastError=cout.status), FALSE;
  if (pret0) *pret0=cout.rets[0];
  if (pret1) *pret1=cout.rets[1];
  if (flags&1) return cout.rets[0];
#endif
  return TRUE;
}

//***************************************************************************

// Library


//***************************************************************************

// Generic board

cgosret_bool CgosCgebBoardClose(HCGOS hCgos)
{
  return(CallCgebDrvPlain(0,xCgosBoardClose,hCgos,0,0,0,0,0,NULL,NULL));
}

cgosret_ulong CgosCgebBoardCount(unsigned int dwClass, unsigned int dwFlags)
{
  return(CallCgebDrvPlain(1,xCgosBoardCount,0,0,dwClass,dwFlags,0,0,NULL,NULL));
}

cgosret_bool CgosCgebBoardOpen(unsigned int dwClass, unsigned int dwNum, unsigned int dwFlags, HCGOS *phCgos)
{
  return(CallCgebDrvPlain(0,xCgosBoardOpen,0,dwNum,dwClass,dwFlags,0,0,phCgos,NULL));
}
cgosret_bool CgosBoardOpenByNameA(const char *pszName, HCGOS *phCgos)
{
  return(CallDrvStruct(0,xCgosBoardOpenByNameA,0,0,0,0,0,0,phCgos,NULL,(void *)pszName,CgosStrLen(pszName),NULL,0));
}

cgosret_bool CgosBoardGetNameA(HCGOS hCgos, char *pszName, unsigned int dwSize)
{
  return(CallDrvStruct(4,xCgosBoardGetNameA,hCgos,0,0,0,0,0,NULL,NULL,NULL,0,pszName,dwSize));
}

cgosret_bool CgosBoardGetInfoA(HCGOS hCgos, CGOSBOARDINFOA *pBoardInfo)
{
  return(CallDrvStruct(8,xCgosBoardGetInfoA,hCgos,0,0,0,0,0,NULL,NULL,NULL,0,pBoardInfo,0));
}
cgosret_bool CgosBoardGetBootCounter(HCGOS hCgos, unsigned int *pdwCount)
{
  return(CallCgebDrvPlain(0,xCgosBoardGetBootCounter,hCgos,0,0,0,0,0,pdwCount,NULL));
}

cgosret_bool CgosBoardGetRunningTimeMeter(HCGOS hCgos, unsigned int *pdwCount)
{
  return(CallCgebDrvPlain(0,xCgosBoardGetRunningTimeMeter,hCgos,0,0,0,0,0,pdwCount,NULL));
}

cgosret_bool CgosBoardGetOption(HCGOS hCgos, unsigned int dwOption, unsigned int *pdwSetting)
{
  return(CallCgebDrvPlain(0,xCgosBoardGetOption,hCgos,dwOption,0,0,0,0,pdwSetting,NULL));
}

cgosret_bool CgosBoardSetOption(HCGOS hCgos, unsigned int dwOption, unsigned int dwSetting)
{
  return(CallCgebDrvPlain(0,xCgosBoardSetOption,hCgos,dwOption,dwSetting,0,0,0,NULL,NULL));
}

//***************************************************************************

// I2C Bus

cgosret_ulong CgosCgebI2CCount(HCGOS hCgos)
{
  return (CallCgebDrvPlain(1,xCgosI2CCount,hCgos,0,0,0,0,0,NULL,NULL));
}

cgosret_bool CgosCgebI2CIsAvailable(HCGOS hCgos, unsigned int dwUnit)
{
  return(CallCgebDrvPlain(0,xCgosI2CIsAvailable,hCgos,dwUnit,0,0,0,0,NULL,NULL));
}

cgosret_ulong CgosCgebI2CType(HCGOS hCgos, unsigned int dwUnit)
{
  return(CallCgebDrvPlain(1,xCgosI2CType,hCgos,dwUnit,0,0,0,0,NULL,NULL));
}

cgosret_bool CgosCgebI2CRead(HCGOS hCgos, unsigned int dwUnit, unsigned char bAddr, unsigned char *pBytes, unsigned int dwLen)
{
  return(CallDrvStruct(0,xCgosI2CRead,hCgos,dwUnit,bAddr,0,0,0,NULL,NULL,NULL,0,pBytes,dwLen));
}

cgosret_bool CgosCgebI2CWrite(HCGOS hCgos, unsigned int dwUnit, unsigned char bAddr, unsigned char *pBytes, unsigned int dwLen)
{
  return(CallDrvStruct(0,xCgosI2CWrite,hCgos,dwUnit,bAddr,0,0,0,NULL,NULL,pBytes,dwLen,NULL,0));
}

cgosret_bool CgosCgebI2CReadRegister(HCGOS hCgos, unsigned int dwUnit, unsigned char bAddr, unsigned short wReg, unsigned char *pDataByte)
{
  unsigned int ret=0;
  if (!pDataByte) return FALSE;
  if (!CallCgebDrvPlain(0,xCgosI2CReadRegister,hCgos,dwUnit,bAddr,wReg,0,0,&ret,NULL))
    return FALSE;
  *pDataByte=(unsigned char)ret;
  return TRUE;
}

cgosret_bool CgosCgebI2CWriteRegister(HCGOS hCgos, unsigned int dwUnit, unsigned char bAddr, unsigned short wReg, unsigned char bData)
{
  return(CallCgebDrvPlain(0,xCgosI2CWriteRegister,hCgos,dwUnit,bAddr,wReg,bData,0,NULL,NULL));
}

cgosret_bool CgosCgebI2CWriteReadCombined(HCGOS hCgos, unsigned int dwUnit, unsigned char bAddr, unsigned char *pBytesWrite, unsigned int dwLenWrite, unsigned char *pBytesRead, unsigned int dwLenRead)
{
  return(CallDrvStruct(0,xCgosI2CWriteReadCombined,hCgos,dwUnit,bAddr,0,0,0,NULL,NULL,pBytesWrite,dwLenWrite,pBytesRead,dwLenRead));
}

cgosret_bool CgosCgebI2CGetMaxFrequency(HCGOS hCgos, unsigned int dwUnit, unsigned int *pdwSetting) // 1.3
{
  return(CallCgebDrvPlain(0,xCgosI2CGetMaxFrequency,hCgos,dwUnit,0,0,0,0,pdwSetting,NULL));
}

cgosret_bool CgosCgebI2CGetFrequency(HCGOS hCgos, unsigned int dwUnit, unsigned int *pdwSetting) // 1.3
{
  return(CallCgebDrvPlain(0,xCgosI2CGetFrequency,hCgos,dwUnit,0,0,0,0,pdwSetting,NULL));
}

cgosret_bool CgosCgebI2CSetFrequency(HCGOS hCgos, unsigned int dwUnit, unsigned int dwSetting) // 1.3
{
  return(CallCgebDrvPlain(0,xCgosI2CSetFrequency,hCgos,dwUnit,dwSetting,0,0,0,NULL,NULL));
}
#endif
