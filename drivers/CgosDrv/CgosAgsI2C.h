/*
 * CGOS i2c driver definitions
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
#ifndef _CGOSAGSI2C_H_
#define _CGOSAGSI2C_H_

#define CG_I2C_FLAG_START 0x00080 /* send START condition */
#define CG_I2C_FLAG_STOP 0x00040 /* send STOP condition */
#define CG_I2C_FLAG_ALL_ACK 0x08000 /* send ACK on all read bytes */
#define CG_I2C_FLAG_ALL_NAK 0x04000 /* send NAK on all read bytes */
#define AGS_LG_I2C_ADAPTERS  1    // AGS only has 1 i2c adapter
#define AGS_LG_I2C_CLIENTS   2    // AGS only has 2 i2c clients, INA2XX for fan control
#define AGS_LG_I2C_COMPATIBLE	"ti,ina219"
#define AGS_PLATFORM_NAME "ags-lg"
#define AGS_PLATFORM_I2C_NAME "ags-lg-i2c"


struct i2c_adapter;
struct i2c_client;

typedef struct {
  void *hDriver;
  struct semaphore semIoCtl;
  void *devfs_handle;
  } OS_DRV_VARS;

typedef struct {
  CGOS_DRV_BOARD     *board;
  struct i2c_adapter adapter;
  struct i2c_client  i2c_client[AGS_LG_I2C_CLIENTS];
  int                unit;
  int                num_i2c_clients;
} CGOS_I2C_PRIV;

typedef struct {
  OS_DRV_VARS       osDrvVars;
  struct miscdevice miscdev;
  CGOS_I2C_PRIV     i2c_priv[AGS_LG_I2C_ADAPTERS]; 
  struct mutex      mutex;
  struct kref       ref;
  spinlock_t        slock;
  wait_queue_head_t wait;
  struct list_head  free;
  struct list_head  used;
  struct device     *dev;
  struct dentry     *dbg_entry;
  int               num_i2c_adapters;
} AGS_LG_PRIV;

int CgosI2CSetup(struct platform_device *pdev, CGOS_I2C_PRIV *p_i2c);
#endif
