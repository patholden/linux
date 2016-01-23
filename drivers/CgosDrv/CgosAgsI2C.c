/*
 * CGOS i2c driver
 *
 * (c) 2015 Patricia A. Holden, c/o Assembly Guidance Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/version.h>
#include <linux/module.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
#include <linux/autoconf.h>
#else
#include <generated/autoconf.h>
#endif

#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include <asm/uaccess.h>
#include <linux/printk.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/miscdevice.h>

#include "CgosDrv.h"
#include "DrvOsHdr.h"  // OS specific header files
#include "CgosDef.h"   // Standard definitions
#define NOCGOSAPI
#include "Cgos.h"      // CGOS Library API definitions and functions
#include "CgosPriv.h"  // CGOS Library API private definitions and functions
#include "CgosIoct.h"  // CGOS IO Control Driver Interface
#include "Cgeb.h"      // CGEB definitions
#include "CgebSda.h"   // CGEB Secure Data Area
#include "DrvVars.h"   // Driver Variables
#include "DrvUla.h"    // Driver Upper Layer
#include "DrvOsa.h"    // CGOS OS Abstraction Layer
#include "CgebFct.h"   // CGEB functions
#include "DrvUla.h"
#include "CgosIobd.h"
#include "CgosAgsI2C.h"

static u32 cgos_i2c_func(struct i2c_adapter *adapter)
{
  return (I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK));
}
 
#if 0
static int cgos_i2c_set_speed(CGOS_I2C_PRIV *priv, int speed)
{
  CGEBFPS       fps;
  CGOS_DRV_CGEB *cgeb=(CGOS_DRV_CGEB *)&priv->board->cgeb;

  memset(&fps, 0, sizeof(fps));

  fps.unit = priv->unit;
  fps.pars[0] = speed;

  return(CgebInvokePlain(cgeb, xCgebI2CSetFrequency, (void *)&fps));
}
#endif
static int cgos_i2c_xfer(struct i2c_adapter *adapter,
			 struct i2c_msg *msgs, int num)
{
  CGEBFPS  fps;
  int   i, ret;
  unsigned long flags = CG_I2C_FLAG_START;
  CGOS_I2C_PRIV *priv = (CGOS_I2C_PRIV *)i2c_get_adapdata(adapter);
  CGOS_DRV_CGEB *cgeb=(CGOS_DRV_CGEB *)&priv->board->cgeb;
  unsigned long rdlen, wrlen;
  unsigned char *rdbuf, *wrbuf, *raw_wrbuf;
  unsigned short lmax = 0;

/*
 * With Cgos, the I2C address is part of the write data
 * buffer, so allocate a buffer with the length of the
 * longest write buffer + 1
 */
  for (i = 0; i < num; i++)
    if (!(msgs[i].flags & I2C_M_RD))
      lmax = max(lmax, msgs[i].len);

 raw_wrbuf = kmalloc(lmax + 1, GFP_KERNEL);
 if (!raw_wrbuf)
   return -ENOMEM;

 for (i = 0; i < num; i++) {
   if (msgs[i].flags & I2C_M_RD) {
     rdbuf = msgs[i].buf;
     rdlen = msgs[i].len;
     wrbuf = NULL;
     wrlen = 0;
   } else {
     rdbuf = NULL;
     rdlen = 0;
     wrbuf = msgs[i].buf;
     wrlen = msgs[i].len;
   }

   raw_wrbuf[0] = msgs[i].addr << 1;
   if (wrlen)
     memcpy(&raw_wrbuf[1], wrbuf, wrlen);

   if (msgs[i].flags & I2C_M_RD)
     raw_wrbuf[0] |= 1;
   if (i == num - 1)
     flags |= CG_I2C_FLAG_STOP;

   dev_dbg(&adapter->dev,
	   "%s: rd: %p/%ld wr: %p/%ld flags: 0x%08lx %s\n",
	   __func__, rdbuf, rdlen, raw_wrbuf, wrlen + 1,
	   flags,
	   msgs[i].flags & I2C_M_RD ? "READ" : "WRITE");
   
   memset(&fps, 0, sizeof(fps));

   fps.unit = priv->unit;
   fps.pars[0] = wrlen + 1;
   fps.pars[1] = rdlen;
   fps.pars[2] = flags;
   fps.iptr = raw_wrbuf;
   fps.optr = rdbuf;
   ret = CgebInvokePlain(cgeb, xCgebI2CTransfer, (void *)&fps);
   if (ret) {
     ret = -EREMOTEIO;
     goto out;
   }
 }

 ret = num;

out:
 kfree(raw_wrbuf);
 return ret;
}
 
static struct i2c_algorithm cgos_i2c_algo = {
  .master_xfer = cgos_i2c_xfer,
  .functionality = cgos_i2c_func,
};

static struct i2c_adapter cgos_i2c_adapter_template = {
  .owner = THIS_MODULE,
  .algo = &cgos_i2c_algo,
  .name = "ags-lg-i2c",
};

#if 0
static int ags_lg_register_subdevs(struct platform_device *pdev, CGOS_I2C_PRIV *p_i2c)
{
  struct device_node *i2c_bus, *child;
  struct device      *ppdev = &pdev->dev;
  AGS_LG_PRIV        *ags_lg_data=0;
	
  int index = 0;

  ags_lg_data = platform_get_drvdata(pdev);
  if (!ags_lg_data)
    return(-ENOMEM);

  for_each_compatible_node(i2c_bus, NULL, AGS_LG_I2C_COMPATIBLE) {
    if (index >= AGS_LG_I2C_CLIENTS)
      break;

    // this device belongs to AGS platform driver
    of_platform_populate(child, NULL, NULL, ppdev);
    index++;
  }
  return 0;
}

#endif
int CgosI2CSetup(struct platform_device *pdev, CGOS_I2C_PRIV *p_i2c)
{
  int    error=0;

  p_i2c->adapter = cgos_i2c_adapter_template;
  p_i2c->adapter.dev.parent = &pdev->dev;
  p_i2c->adapter.class = I2C_CLASS_SPD;
  strcpy(p_i2c->adapter.name, AGS_PLATFORM_I2C_NAME);
  error = i2c_add_adapter((struct i2c_adapter *)&p_i2c->adapter);
  if (error < 0)
    {
      printk("failed to add I2C bus %s\n", p_i2c->adapter.name);
      dev_err((struct device *)pdev, "failed to add I2C bus %s\n", p_i2c->adapter.name);
      return error;
    }
#if 0
  error = ags_lg_register_subdevs(pdev, p_i2c);
  if (error)
    {
      printk("failed to register I2C clients for %s, err %x\n", p_i2c->adapter.name, error);
      dev_err((struct device *)pdev,"failed to register I2C clients for %s, err %x\n", p_i2c->adapter.name, error);
      return error;
    }
#endif
  return 0;
}

MODULE_AUTHOR("Patricia A. Holden, Assembly Guidance Inc.");
MODULE_DESCRIPTION("AGS drivers");
MODULE_LICENSE("GPL");
