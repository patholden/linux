/*
 * CGEB driver
 *
 * (c) 2011 Sascha Hauer, Pengutronix
 *
 * Based on code from Congatec AG.
 *
 * CGEB is a BIOS interface found on congatech modules. It consists of
 * code found in the BIOS memory map which is called in a ioctl like
 * fashion. This file contains the basic driver for this interface
 * which provides access to the GCEB interface and registers the child
 * devices like I2C busses and watchdogs.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/congatec-cgeb.h>

#include <generated/autoconf.h>
#include <stddef.h>

#define CGOS_BOARD_MAX_SIZE_ID_STRING 16

#define CGEB_VERSION_MAJOR 1

#define CGEB_GET_VERSION_MAJOR(v) (((unsigned long)(v))>>24)

/* CGEB Low Descriptor located in 0xc0000-0xfffff */
#define CGEB_LD_MAGIC "$CGEBLD$"

struct cgeb_low_desc {
	char magic[8];		/* descriptor magic string */
	u16 size;		/* size of this descriptor */
	u16 reserved;
	char bios_name[8];	/* BIOS name and revision "ppppRvvv" */
	u32 hi_desc_phys_addr;	/* phys addr of the high descriptor, can be 0 */
};

/* CGEB High Descriptor located in 0xfff00000-0xffffffff */
#define CGEB_HD_MAGIC "$CGEBHD$"

struct cgeb_high_desc {
	char magic[8];		/* descriptor magic string */
	u16 size;		/* size of this descriptor */
	u16 reserved;
	u32 data_size;		/* CGEB data area size */
	u32 code_size;		/* CGEB code area size */
	u32 entry_rel;		/* CGEB entry point relative to start */
};

struct cgeb_far_ptr {
	u32 off;
	u16 seg;
	u16 pad;
};

struct cgeb_fps {
	u32 size;		/* size of the parameter structure */
	u32 fct;		/* function number */
	struct cgeb_far_ptr data;	/* CGEB data area */
	u32 cont;		/* private continuation pointer */
	u32 subfps;		/* private sub function parameter
				 * structure pointer
				 */
	u32 subfct;		/* sub function pointer */
	u32 status;		/* result codes of the function */
	u32 unit;		/* unit number or type */
	u32 pars[4];		/* input parameters */
	u32 rets[2];		/* return parameters */
	void *iptr;		/* input pointer */
	void *optr;		/* output pointer */
};

/* continuation status codes */
#define CGEB_SUCCESS            0
#define CGEB_NEXT               1
#define CGEB_DELAY              2
#define CGEB_NOIRQS             3

#define CGEB_DBG_STR        0x100
#define CGEB_DBG_HEX        0x101
#define CGEB_DBG_DEC        0x102

struct cgeb_map_mem {
	unsigned long phys;	/* physical address */
	unsigned long size;	/* size in bytes */
	struct cgeb_far_ptr virt;
};

struct cgeb_map_mem_list {
	unsigned long count;	/* number of memory map entries */
	struct cgeb_map_mem entries[];
};

struct cgeb_boardinfo {
	unsigned long size;
	unsigned long flags;
	unsigned long classes;
	unsigned long primary_class;
	char board[CGOS_BOARD_MAX_SIZE_ID_STRING];
	/* optional */
	char vendor[CGOS_BOARD_MAX_SIZE_ID_STRING];
};

struct cgeb_i2c_info {
	unsigned long size;
	unsigned long type;
	unsigned long frequency;
	unsigned long maxFrequency;
};

/* I2C Types */
#define CGEB_I2C_TYPE_UNKNOWN 0
#define CGEB_I2C_TYPE_PRIMARY 1
#define CGEB_I2C_TYPE_SMB     2
#define CGEB_I2C_TYPE_DDC     3
#define CGEB_I2C_TYPE_BC      4

struct cgeb_board_data {
  void *code;
  void *data;
  unsigned short ds;
  struct cgeb_map_mem_list *map_mem;
  struct platform_device **devices;
  int num_devices;

  /*
   * entry points to a bimodal C style function that expects a far pointer
   * to a fps. If cs is 0 then it does a near return, otherwise a far
   * return. If we ever need a far return then we must not pass cs at all.
   * parameters are removed by the caller.
   */
  void __attribute__((regparm(0)))(*entry)(unsigned short,
			struct cgeb_fps *, unsigned short);
};

static unsigned short get_data_segment(void)
{
  unsigned short ret;

  asm volatile("mov %%ds, %0\n"
	       : "=r"(ret)
	       :
	       : "memory"
	       );
  return ret;
}

/*
 * cgeb_invoke - invoke CGEB BIOS call.
 *
 * @board:	board context data
 * @p:		CGEB parameters for this call
 * @fct:	CGEB function code
 * @return:	0 on success or negative error code on failure.
 *
 * Call the CGEB BIOS code with the given parameters.
 */
unsigned int cgeb_call(struct cgeb_board_data *board,
		struct cgeb_function_parameters *p, enum cgeb_function fct)
{
  struct cgeb_fps fps;
  int i;

  memset(&fps, 0, sizeof(fps));

  fps.size = sizeof(fps);
  fps.fct = fct;
  fps.data.off = (unsigned long) board->data;
  fps.data.seg = board->ds;
  fps.data.pad = 0;
  fps.status = 0;
  fps.cont = fps.subfct = fps.subfps = 0;
  fps.unit = p->unit;
  for (i = 0; i < 4; i++)
    fps.pars[i] = p->pars[i];
  fps.iptr = p->iptr;
  fps.optr = p->optr;
  
  while (1) {
    pr_debug("CGEB: CGEB: ->  size %02x, fct %02x, data %04x:%08x, status %08x\n",
	     fps.size, fps.fct, fps.data.seg, fps.data.off,
	     fps.status);

    board->entry(0, &fps, fps.data.seg);

    switch (fps.status) {
    case CGEB_SUCCESS:
      goto out;
    case CGEB_NEXT:
      break;	/* simply call again */
    case CGEB_NOIRQS:
      current->state = TASK_INTERRUPTIBLE;
      schedule_timeout(0);
      break;
    case CGEB_DELAY:
      usleep_range(fps.rets[0], fps.rets[0] + 1000);
      break;
    case CGEB_DBG_STR:
      if (fps.optr)
	pr_debug("CGEB: %s\n", (char *)fps.optr);
      break;
    case CGEB_DBG_HEX:
      pr_debug("CGEB: 0x%08x\n", fps.rets[0]);
      break;
    case CGEB_DBG_DEC:
      pr_debug("CGEB: %d\n", fps.rets[0]);
      break;

    default:
      /* unknown continuation code */
      return -EINVAL;
    }
  }
out:
  for (i = 0; i < 2; i++)
    p->rets[i] = fps.rets[i];
  p->optr = fps.optr;

  return 0;
}
EXPORT_SYMBOL_GPL(cgeb_call);

/*
 * cgeb_call_simple - convenience wrapper for cgeb_call
 *
 * @board:	board context data
 * @p:		CGEB parameters for this call
 * @fct:	CGEB function code
 * @return:	0 on success or negative error code on failure.
 *
 * Call the CGEB BIOS code with the given parameters.
 */
int cgeb_call_simple(struct cgeb_board_data *board,
		     enum cgeb_function fct, unsigned long unit,
		     unsigned long *optr, unsigned long *result)
{
  struct cgeb_function_parameters p;
  unsigned int ret;

  memset(&p, 0, sizeof(p));
  p.unit = unit;
  p.optr = optr;

  ret = cgeb_call(board, &p, fct);
  if (optr)
    *optr = (unsigned long)p.optr;
  if (result)
    *result = p.rets[0];
  
  return ret;
}
EXPORT_SYMBOL_GPL(cgeb_call_simple);

static void *cgeb_find_magic(void *_mem, size_t len, char *magic)
{
  unsigned long magic0 = ((unsigned long *) magic)[0];
  unsigned long magic1 = ((unsigned long *) magic)[1];
  int i = 0;

  while (i < len) {
    u32 *mem = _mem + i;
    if (mem[0] == magic0 && mem[1] == magic1)
      return mem;
    i += 16;
  }

  return NULL;
}

static void cgeb_unmap_memory(struct cgeb_board_data *board)
{
  struct cgeb_map_mem_list *pmm;
  struct cgeb_map_mem *pmme;
  unsigned long i;

  if (!board->map_mem)
    return;

  pmm = board->map_mem;
  pmme = pmm->entries;
  for (i = 0; i < pmm->count; i++, pmme++) {
    if (pmme->virt.off)
      iounmap((void *)pmme->virt.off);
    pmme->virt.off = 0;
    pmme->virt.seg = 0;
  }
}

static int cgeb_map_memory(struct cgeb_board_data *board)
{
  struct cgeb_map_mem_list *pmm;
  struct cgeb_map_mem *pmme;
  int i;
  int ret;

  ret = cgeb_call_simple(board, CgebMapGetMem, 0, (void *)&board->map_mem,
			 NULL);
  if (ret)
    return ret;
  if (!board->map_mem)
    return 0;

  pmm = board->map_mem;
  pmme = pmm->entries;

  pr_debug("CGEB: Memory Map with %lu entries\n", pmm->count);

  for (i = 0; i < pmm->count; i++, pmme++) {
    if (pmme->phys && pmme->size) {
      pmme->virt.off =
	(unsigned long) ioremap_cache(pmme->phys,
				      pmme->size);
      if (!pmme->virt.off)
	return -ENOMEM;
    } else {
      pmme->virt.off = 0;
    }

    pmme->virt.seg = (pmme->virt.off) ? board->ds : 0;
    pr_debug("CGEB:   Map phys %08lx, size %08lx, virt %04x:%08x\n",
	     pmme->phys, pmme->size, pmme->virt.seg,
	     pmme->virt.off);
  }

  return cgeb_call_simple(board, CgebMapChanged, 0, NULL, NULL);
}

static struct cgeb_board_data *cgeb_open(unsigned long base, unsigned long len)
{
  unsigned long dw;
  struct cgeb_boardinfo *pbi;
  struct cgeb_low_desc *low_desc;
  struct cgeb_high_desc *high_desc = NULL;
  unsigned long high_desc_phys;
  unsigned long high_desc_len;
  void __iomem *pcur, *high_desc_virt;
  int ret;

  struct cgeb_board_data *board;

  board = kzalloc(sizeof(*board), GFP_KERNEL);
  if (!board)
    return NULL;

  pcur = ioremap_cache(base, len);
  if (!pcur)
    goto err_kfree;

  /* look for the CGEB descriptor */
  low_desc = cgeb_find_magic(pcur, len, CGEB_LD_MAGIC);
  if (!low_desc)
    goto err_kfree;

  pr_debug("CGEB: Found CGEB_LD_MAGIC\n");

  if (low_desc->size < sizeof(struct cgeb_low_desc) - sizeof(long))
    goto err_kfree;

  if (low_desc->size >= sizeof(struct cgeb_low_desc)
      && low_desc->hi_desc_phys_addr)
    high_desc_phys = low_desc->hi_desc_phys_addr;
  else
    high_desc_phys = 0xfff00000;

  high_desc_len = (unsigned long) -(long) high_desc_phys;

  pr_debug("CGEB: Looking for CGEB hi desc between phys 0x%lx and 0x%x\n",
	   high_desc_phys, -1);

  high_desc_virt = ioremap_cache(high_desc_phys, high_desc_len);
  if (!high_desc_virt)
    goto err_kfree;

  pr_debug("CGEB: Looking for CGEB hi desc between virt 0x%p and 0x%p\n",
	   high_desc_virt, high_desc_virt + high_desc_len - 1);

  high_desc = cgeb_find_magic(high_desc_virt, high_desc_len,
			      CGEB_HD_MAGIC);
  if (!high_desc)
    goto err_iounmap;

  pr_debug("CGEB: Found CGEB_HD_MAGIC\n");

  if (high_desc->size < sizeof(struct cgeb_high_desc))
    goto err_iounmap;

  pr_debug("CGEB: data_size %u, code_size %u, entry_rel %u\n",
	   high_desc->data_size, high_desc->code_size, high_desc->entry_rel);

  board->code = __vmalloc(high_desc->code_size, GFP_KERNEL,
			  PAGE_KERNEL_EXEC);
  if (!board->code)
    goto err_iounmap;

  memcpy(board->code, high_desc, high_desc->code_size);

  high_desc = board->code;
  board->entry = board->code + high_desc->entry_rel;
  board->ds = get_data_segment();
  ret = cgeb_call_simple(board, CgebGetCgebVersion, 0, NULL, &dw);
  if (ret)
    goto err_vfree;

  if (CGEB_GET_VERSION_MAJOR(dw) != CGEB_VERSION_MAJOR)
    goto err_vfree;

  pr_debug("CGEB: BIOS interface revision: %ld.%ld\n",
	   dw >> 16, dw & 0xffff);

  if (high_desc->data_size) {
    board->data = vmalloc(high_desc->data_size);
    if (!board->data)
      goto err_vfree;
  } else {
    ret = cgeb_call_simple(board, CgebGetDataSize, 0, NULL, &dw);
    if (!ret && dw) {
      board->data = vmalloc(dw);
      if (!board->data)
	goto err_vfree;
    }
  }

  ret = cgeb_call_simple(board, CgebOpen, 0, NULL, NULL);
  if (ret)
    goto err_vfree_data;

  ret = cgeb_map_memory(board);
  if (ret)
    goto err_free_map;

  ret = cgeb_call_simple(board, CgebBoardGetInfo, 0, &dw, NULL);
  if (ret)
    goto err_free_map;

  pbi = (struct cgeb_boardinfo *) dw;

  pr_info("CGEB: Board name: %c%c%c%c\n",
	  pbi->board[0], pbi->board[1],
	  pbi->board[2], pbi->board[3]);

  iounmap(high_desc_virt);
  return board;

err_free_map:
  cgeb_unmap_memory(board);
err_vfree_data:
  vfree(board->data);
err_vfree:
  vfree(board->code);
err_iounmap:
  iounmap(high_desc_virt);
err_kfree:
  kfree(board);
  return NULL;
}

static void cgeb_close(struct cgeb_board_data *board)
{
  cgeb_call_simple(board, CgebClose, 0, NULL, NULL);

  cgeb_unmap_memory(board);

  vfree(board->data);
  vfree(board->code);
}

static unsigned long cgeb_i2c_get_type(struct cgeb_board_data *brd, int unit)
{
  struct cgeb_i2c_info *info;
  int ret;

  ret = cgeb_call_simple(brd, CgebI2CGetInfo, unit, (void *) &info, NULL);
  if (ret)
    return ret;
  if (!info)
    return -EINVAL;
  return info->type;
}

static struct cgeb_board_data *cgeb_board;

static int __init cgeb_init(void)
{
  struct cgeb_board_data *board;
  unsigned long base;
  int i, ret;
  struct cgeb_pdata pdata;
  unsigned long i2c_count, watchdog_count;
  int num_devices = 0;

  for (base = 0xf0000; base >= 0xc0000; base -= 0x10000) {
    board = cgeb_open(base, 0x10000);
    if (board)
      break;
  }

  if (!board)
    return -ENODEV;

  cgeb_board = board;

  pdata.board = board;

  cgeb_call_simple(board, CgebI2CCount, 0, NULL, &i2c_count);
  cgeb_call_simple(board, CgebWDogCount, 0, NULL, &watchdog_count);

  board->num_devices = i2c_count + watchdog_count;
  board->devices = kzalloc(sizeof(void *) * (board->num_devices),
			   GFP_KERNEL);
  if (!board->devices) {
    ret = -ENOMEM;
    goto err_out;
  }

  for (i = 0; i < i2c_count; i++) {
    ret = cgeb_i2c_get_type(board, i);
    if (ret != CGEB_I2C_TYPE_PRIMARY)
      continue;

    pdata.unit = i;
    board->devices[num_devices] =
      platform_device_register_data(NULL, "cgeb-i2c", i,
				    &pdata, sizeof(pdata));
    num_devices++;
  }

  for (i = 0; i < watchdog_count; i++) {
    board->devices[num_devices] =
      platform_device_register_data(NULL, "cgeb-watchdog", i,
				    &pdata, sizeof(pdata));
    pdata.unit = i;
    num_devices++;
  }

  return 0;
err_out:
  cgeb_close(board);
  kfree(board);
  return ret;
}

static void cgeb_exit(void)
{
  struct cgeb_board_data *board = cgeb_board;
  int i;

  for (i = 0; i < board->num_devices; i++)
    if (board->devices[i])
      platform_device_unregister(board->devices[i]);

  cgeb_close(board);
  kfree(board->devices);
  kfree(board);
}

module_init(cgeb_init);
module_exit(cgeb_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("Congatec CGEB driver");
MODULE_LICENSE("GPL");
