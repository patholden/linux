/*---------------------------------------------------------------------------
 *
 * Copyright (c) 2015, congatec AG. All rights reserved.
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
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/i2c.h>
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

//***************************************************************************

#define DEVICE_NAME "cgos"
#define AGS_LG_NAME "ags-lg"
#define AGS_LG_VERSION "0.1"
//***************************************************************************

static ssize_t revision_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
  return snprintf(buf, 20, "%s\n", AGS_LG_VERSION);
  return(0);
}
static DEVICE_ATTR(revision, S_IRUGO, revision_show, NULL);
static struct attribute *ags_lg_attrs[] = {
  &dev_attr_revision.attr,
  NULL
};
static struct attribute_group ags_lg_attr_group = {
  .attrs   = ags_lg_attrs,
};

// OS specific driver variables
static struct file_operations cgos_fops;
struct miscdevice cgos_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = DEVICE_NAME,
  .fops = &cgos_fops
};

#if 0
static int data_debug_show(struct seq_file *f, void *offset)
{
  AGS_LG_PRIV *priv = (AGS_LG_PRIV *)f->private;

  seq_printf(f, "Device %s\n", priv->miscdev.name);
  seq_printf(f, "i2c adapters found: %d\n", priv->num_i2c_adapters);
  return 0;
}

static int data_debug_open(struct inode *inode, struct file *file)
{
  return single_open(file, data_debug_show, inode->i_private);
}
static const struct file_operations data_debug_fops = {
	.owner		= THIS_MODULE,
	.open		= data_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int data_debugfs_init(AGS_LG_PRIV *priv)
{
	priv->dbg_entry = debugfs_create_file(DEVICE_NAME, S_IRUGO, NULL, priv,
					      &data_debug_fops);
	return PTR_ERR_OR_ZERO(priv->dbg_entry);
}
#endif
//***************************************************************************

int cgos_open(struct inode *_inode, struct file *f)
  {
//  MOD_INC_USE_COUNT;
  return 0;
  }

int cgos_release(struct inode *_inode, struct file *f)
  {
//  MOD_DEC_USE_COUNT;
  return 0;
  }

//***************************************************************************

#define return_ioctl(ret) { if (pbuf!=buf) kfree(pbuf); return ret; }

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35)
#define ioctl unlocked_ioctl
long cgos_ioctl(struct file *f, unsigned int command, unsigned long arg)
#else
int cgos_ioctl(struct inode *_inode, struct file *f, unsigned int command, unsigned long arg)
#endif
  {
  IOCTL_BUF_DESC iobd;
  unsigned char  buf[512];
  AGS_LG_PRIV    *ags_lg_data;
  unsigned char  *pbuf=buf;
  unsigned int   ret, rlen=0, maxlen;

  if (copy_from_user(&iobd,(IOCTL_BUF_DESC *)arg,sizeof(iobd)))
    return -EFAULT;

  maxlen=iobd.nInBufferSize>iobd.nOutBufferSize?iobd.nInBufferSize:iobd.nOutBufferSize;
  if (maxlen>sizeof(buf))
    pbuf=kmalloc(maxlen,GFP_KERNEL);
  if (!pbuf) return -ENOMEM;

  if (iobd.nInBufferSize) {
    if (!iobd.pInBuffer || copy_from_user(pbuf,iobd.pInBuffer,iobd.nInBufferSize))
      return_ioctl(-EFAULT);
    }

  ags_lg_data = (AGS_LG_PRIV *) container_of(f->private_data, AGS_LG_PRIV, miscdev);
  if (!ags_lg_data)
    return_ioctl(-EFAULT);
  
  down(&ags_lg_data->osDrvVars.semIoCtl);
  ret=UlaDeviceIoControl(ags_lg_data->osDrvVars.hDriver,command,pbuf,iobd.nInBufferSize,
     pbuf,iobd.nOutBufferSize,&rlen);
  up(&ags_lg_data->osDrvVars.semIoCtl);
  if (ret) return_ioctl(-EFAULT);

  if (rlen) {
    if (!iobd.pOutBuffer || copy_to_user(iobd.pOutBuffer,pbuf,rlen))
      return_ioctl(-EFAULT);
    }
  if (pbuf!=buf) kfree(pbuf);
  if (iobd.pBytesReturned)
    if (copy_to_user(iobd.pBytesReturned,&rlen,sizeof(unsigned int)))
      return -EFAULT;

  return 0;
  }

//***************************************************************************
static struct file_operations cgos_fops={
  owner: THIS_MODULE,
  ioctl: cgos_ioctl,
  open: cgos_open,
  release: cgos_release,
  };
//***************************************************************************
//   This function is called once when the ags-lg device is registered.
//   Private device data needed by both misc device "cgos" and i2c
//   adapter(s) and i2c client(s) is kept as platform-device data for ags-lg
//   and no longer located on stack.
//***************************************************************************
static int ags_lg_probe(struct platform_device *pdev)
{
  CGEB_I2C_INFO   i2c_info; 
  CGOS_DRV_VARS   *cdv;
  CGOS_DRV_BOARD  *board;
  CGOS_DRV_CGEB   *cgeb;
  struct device   *this_device;
  struct device   *dev = &pdev->dev;
  int             i,error=0,i2c_master=0;
  AGS_LG_PRIV     *ags_lg_data;
  
  // Init platform device info shared by all
  ags_lg_data = devm_kzalloc(&pdev->dev, sizeof(AGS_LG_PRIV), GFP_KERNEL);
  if (!ags_lg_data)
    {
      dev_dbg(dev, "\nAGS:  Malloc failed");
      error = -ENOMEM;
      return(error);
    }
  memset((char *)ags_lg_data,0,sizeof(AGS_LG_PRIV));
  platform_set_drvdata(pdev, ags_lg_data); 
  ags_lg_data->dev = dev;
  sema_init(&ags_lg_data->osDrvVars.semIoCtl,1);
  kref_init(&ags_lg_data->ref);
  mutex_init(&ags_lg_data->mutex);

  dev_set_drvdata(ags_lg_data->dev, ags_lg_data);
  spin_lock_init(&ags_lg_data->slock);
  INIT_LIST_HEAD(&ags_lg_data->free);
  INIT_LIST_HEAD(&ags_lg_data->used);
  init_waitqueue_head(&ags_lg_data->wait);

  // Setup misc device
  ags_lg_data->miscdev.minor = cgos_device.minor;
  ags_lg_data->miscdev.name = cgos_device.name;
  ags_lg_data->miscdev.fops = cgos_device.fops;
  error = misc_register(&ags_lg_data->miscdev);
  if (error)
    {
      dev_dbg(dev, "AGS:  Failed to register CGOS misc_device \n");
      kfree(ags_lg_data);
      return(error);
    }
  dev_dbg(dev, "AGS:  Registered CGOS miscdev\n");

#if 0
  /* Create the debugfs files */
  error = data_debugfs_init(ags_lg_data);
  if (error)
    {
      dev_err(&pdev->dev, "Unable to create debugfs files\n");
      misc_deregister(&ags_lg_data->miscdev);
      mutex_destroy(&ags_lg_data->mutex);
      kfree(ags_lg_data);
      return(error);
    }
#endif
  this_device = ags_lg_data->miscdev.this_device;
  ags_lg_data->miscdev.parent = &pdev->dev;
  dev_set_drvdata(this_device, ags_lg_data);
  platform_set_drvdata(pdev, ags_lg_data);
#if 0
  error = sysfs_create_group(&this_device->kobj, &ags_lg_attr_group);
  if (error)
    {
      dev_err(&pdev->dev, "Unable to create sysfs files\n");
      misc_deregister(&ags_lg_data->miscdev);
      mutex_destroy(&ags_lg_data->mutex);
      kfree(ags_lg_data);
      return(error);
    }
#endif
  return(0);   // FIXME---PAH---NEED TO ADD BACK & DEBUG

  ags_lg_data->osDrvVars.hDriver=UlaOpenDriver(pdev, 0);
  if (!ags_lg_data->osDrvVars.hDriver)
    {
      dev_dbg(dev, "AGS:  Unable to open CGOS board\n");
      sysfs_remove_group(&this_device->kobj, &ags_lg_attr_group);
      misc_deregister(&ags_lg_data->miscdev);
      mutex_destroy(&ags_lg_data->mutex);
      kfree(ags_lg_data);
      return(ENODEV);
    }
  // Now add the i2c & pwm drivers
  cdv = (CGOS_DRV_VARS *)ags_lg_data->osDrvVars.hDriver;
  board = (CGOS_DRV_BOARD *)&cdv->boards[0];
  if (cdv->boardCount != 1)
    {
      dev_dbg(dev, "\nAGS: Board Count not correct, should be 1 but is %d",cdv->boards[0].i2cCount);
      sysfs_remove_group(&this_device->kobj, &ags_lg_attr_group);
      misc_deregister(&ags_lg_data->miscdev);
      mutex_destroy(&ags_lg_data->mutex);
      kfree(ags_lg_data);
      return(ENODEV);
    }
  if (!board->i2cCount)
    {
      dev_dbg(dev, "\nAGS:  CGOS i2cCount not correct, need at least 1!");
      sysfs_remove_group(&this_device->kobj, &ags_lg_attr_group);
      misc_deregister(&ags_lg_data->miscdev);
      mutex_destroy(&ags_lg_data->mutex);
      kfree(ags_lg_data);
      return(ENODEV);
    }
  dev_dbg(dev, "\nAGS:  Found %d I2C entities", board->i2cCount);
  cgeb = (CGOS_DRV_CGEB *)&board->cgeb;
  for (i = 0; i < cdv->boards[0].i2cCount; i++)
    {
      memset((char *)&i2c_info, 0, sizeof(CGEB_I2C_INFO));
      CgebInvokePlain(cgeb,xCgebI2CGetInfo, (void *)&i2c_info);
      if (i2c_info.type != CGEB_I2C_TYPE_PRIMARY)
	{
	  i2c_master = i;
	  break;
	}
    }

  // Found it?
  if (!i2c_master)
    {
      dev_dbg(dev, "\nAGS:  I2C master not found");
      sysfs_remove_group(&this_device->kobj, &ags_lg_attr_group);
      misc_deregister(&ags_lg_data->miscdev);
      mutex_destroy(&ags_lg_data->mutex);
      kfree(ags_lg_data);
      return(ENODEV);
    }

  // Create I2C platform driver
  ags_lg_data->num_i2c_adapters = 1;
  ags_lg_data->i2c_priv[0].board = board;
  ags_lg_data->i2c_priv[0].unit = i2c_master;
  
  error = CgosI2CSetup(pdev, (CGOS_I2C_PRIV *)&ags_lg_data->i2c_priv[0]);
  // Register PWM device
  return error;
}
static int ags_lg_remove(struct platform_device *pdev)
{
  AGS_LG_PRIV *ags_lg_priv = platform_get_drvdata(pdev);
  int         i,j;
  struct device *this_device;

  if (!ags_lg_priv)
    return(-EBADF);

  for (i = 0; i < ags_lg_priv->num_i2c_adapters; i++) {
    i2c_del_adapter(&ags_lg_priv->i2c_priv[i].adapter);
    for (j = 0; j < ags_lg_priv->i2c_priv[i].num_i2c_clients; j++)
      i2c_unregister_device(&ags_lg_priv->i2c_priv[i].i2c_client[j]);
  }
  
  UlaCloseDriver(ags_lg_priv->osDrvVars.hDriver);
  this_device = ags_lg_priv->miscdev.this_device;
  sysfs_remove_group(&this_device->kobj, &ags_lg_attr_group);
  debugfs_remove(ags_lg_priv->dbg_entry);
  misc_deregister(&ags_lg_priv->miscdev);
  mutex_destroy(&ags_lg_priv->mutex);
  kfree(ags_lg_priv);
  return 0;
}

static struct platform_device ags_lg_dev = {
  .name		= AGS_LG_NAME,
  .id     = 0,
  .num_resources = 0,
};

static struct platform_driver ags_lg_driver = {
  .probe = ags_lg_probe,
  .remove = ags_lg_remove,
  .driver = {
    .name = AGS_LG_NAME,
  },
};

static int __init ags_lg_init(void)
{
  int rc;

  rc = platform_driver_register(&ags_lg_driver);
  if (rc)
    {
      printk("\nAGS:  Unable to register platform driver cgos, err %x", rc);
      return(rc);
    }
  rc = platform_device_register(&ags_lg_dev);
  if (rc)
    {
      printk("platform_device_register cgos, ret %d\n", rc);
      return(rc);
    }
  printk("\nAGS:  Register ags-lc device and driver and i2c driver");
  return 0;
}
static void __exit ags_lg_exit(void)
{
  platform_device_unregister(&ags_lg_dev);
  platform_driver_unregister(&ags_lg_driver);
}

module_init(ags_lg_init);
module_exit(ags_lg_exit);
//***************************************************************************

MODULE_AUTHOR("congatec AG");
MODULE_DESCRIPTION("CGOS driver");
MODULE_LICENSE("GPL");

//***************************************************************************

