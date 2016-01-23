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
#include <linux/efi.h>
#include <asm/efi.h>
#include <generated/autoconf.h>
#include <stddef.h>

#if (!defined(__cdecl))
#define __cdecl
#endif

#define CGOS_BOARD_MAX_SIZE_ID_STRING 16

#define CGEB_VERSION_MAJOR 1

#define CGEB_GET_VERSION_MAJOR(v) (((unsigned int)(v))>>24)
#define CGEB_GET_VERSION_MINOR(v) ((((unsigned int)(v))>>16)&0xff)
#define CGEB_GET_VERSION_BUILD(v) (((unsigned int)(v))&0xffff)

/* CGEB Low Descriptor located in 0xc0000-0xfffff */
#define CGEB_LD_MAGIC "$CGEBLD$"

struct cgeb_low_desc {
	char magic[8];		/* descriptor magic string */
	u16 size;		/* size of this descriptor */
	u16 reserved;
	char bios_name[8];	/* BIOS name and revision "ppppRvvv" */
	unsigned int hi_desc_phys_addr;	/* phys addr of the high descriptor, can be 0 */
};

/* CGEB High Descriptor located in 0xfff00000-0xffffffff */
#define CGEB_HD_MAGIC "$CGEBHD$"

struct cgeb_high_desc {
	char magic[8];		/* descriptor magic string */
	unsigned short size;	/* size of this descriptor */
	unsigned short reserved;
	unsigned int data_size;	/* CGEB data area size */
	unsigned int code_size;	/* CGEB code area size */
	unsigned int entry_rel;	/* CGEB entry point relative to start */
};

struct cgeb_far_ptr {
	unsigned int off;
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
	unsigned int phys;	/* physical address */
	unsigned int size;	/* size in bytes */
	struct cgeb_far_ptr virt;
};

struct cgeb_map_mem_list {
	unsigned int count;	/* number of memory map entries */
	struct cgeb_map_mem entries[];
};

struct cgeb_boardinfo {
	unsigned int size;
	unsigned int flags;
	unsigned int classes;
	unsigned int primary_class;
	char board[CGOS_BOARD_MAX_SIZE_ID_STRING];
	/* optional */
	char vendor[CGOS_BOARD_MAX_SIZE_ID_STRING];
};

struct cgeb_i2c_info {
	unsigned int size;
	unsigned int type;
	unsigned int frequency;
	unsigned int maxFrequency;
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

  void *entry;
};

/*
 * entry points to a bimodal C style function that expects a far pointer
 * to a fps. If cs is 0 then it does a near return, otherwise a far
 * return. If we ever need a far return then we must not pass cs at all.
 * parameters are removed by the caller.
 */
static void cgeb_efi_call(struct cgeb_fps *fps, void *addr)
{
#if !defined(AMD64)
  // addr points to a bimodal C style function that expects a far pointer to an fps.
  // if cs is 0 then it does a near return, otherwise a far return.
  // if we ever need a far return then we must not pass cs at all.
  // parameters are removed by the caller
  ((void (__cdecl *)(unsigned short cs, struct cgeb_fps *fps, unsigned short ds))addr)(0,fps,fps->data.seg);
#else
  // CGEBQ expects FPS in edx (ecx is reserved and must be 0)
  // x64 (MS) calling convention is rcx, rdx, r8, r9
  // ABI (linux) calling convention rdi, rsi, rdx, rcx
  // parameters are removed by the caller
  ((void (*)(void *, struct cgeb_fps *fps, struct cgeb_fps *fps_abi, void *))addr)(NULL,fps,fps,NULL);
#endif
}

static unsigned short get_data_segment(void)
{
    // Refer to Cgeb.c
#if 0
  unsigned short ret;

  asm volatile("mov %%ds, %0\n"
	       : "=r"(ret)
	       :
	       : "memory"
	       );
  return ret;
#else
  static unsigned char get_data_seg_raw[]= { // cs would be 0xC8
    0x8C, 0xD8,         // mov ax,ds
    0xC3                // ret
    };
  return ((unsigned short (*)(void))get_data_seg_raw)();
#endif
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
  fps.data.off = (unsigned int) board->data;
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

    cgeb_efi_call(&fps, board->entry);
    
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
		     enum cgeb_function fct, unsigned int unit,
		     void *optr, unsigned int *result)
{
  struct cgeb_function_parameters p;
  unsigned int ret;

  memset(&p, 0, sizeof(p));
  p.unit = unit;
  p.optr = optr;

  ret = cgeb_call(board, &p, fct);
  optr = (unsigned int)p.optr;
  if (result)
    *result = p.rets[0];
  
  return ret;
}
EXPORT_SYMBOL_GPL(cgeb_call_simple);

static void *cgeb_find_magic(unsigned char *_mem, size_t len, char *magic)
{
  unsigned int magic0=((unsigned int *)magic)[0];
  unsigned int magic1=((unsigned int *)magic)[1];

  int i = 0;

  while (i < len) {
    char *mem = (char *)_mem + i;
    if (*(unsigned int *)mem==magic0 && ((unsigned int *)mem)[1]==magic1)
      return mem;
    i += 16;
  }

  return NULL;
}

static void cgeb_unmap_memory(struct cgeb_board_data *board)
{
  struct cgeb_map_mem_list *pmm;
  struct cgeb_map_mem *pmme;
  unsigned int i;

  if (!board->map_mem)
    return;

  pmm = board->map_mem;
  pmme = pmm->entries;
  for (i = 0; i < pmm->count; i++, pmme++) {
    if (pmme->virt.off)
      iounmap(pmme->virt.off);
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

  pr_debug("CGEB: Memory Map with %u entries\n", pmm->count);

  for (i = 0; i < pmm->count; i++, pmme++) {
    if (pmme->phys && pmme->size) {
      pmme->virt.off =
	(unsigned int) early_ioremap(pmme->phys,
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

static struct cgeb_board_data *cgeb_open(unsigned int base, unsigned int len)
{
  unsigned int dw=0;
  struct cgeb_boardinfo *pbi;
  struct cgeb_low_desc *low_desc;
  struct cgeb_high_desc *high_desc = NULL;
  unsigned int high_desc_phys;
  unsigned int high_desc_len;
  void __iomem *pcur, *high_desc_virt;
  int ret;

  struct cgeb_board_data *board;

  board = kzalloc(sizeof(struct cgeb_board_data), GFP_KERNEL);
  if (!board)
    return NULL;

  memset((char *)board, 0, sizeof(struct cgeb_board_data));
	 
  pcur = early_ioremap(base, len);
  if (!pcur)
    goto err_kfree;
  
  printk("\nbase %x ioremap %x",base,(unsigned int)pcur);

  /* look for the CGEB descriptor */
  low_desc = cgeb_find_magic((unsigned char *)pcur, len, CGEB_LD_MAGIC);
  iounmap(base);
  if (!low_desc)
    goto err_kfree;

    pr_debug("CGEB: Found CGEB_LD_MAGIC\n");

  if (low_desc->size < sizeof(struct cgeb_low_desc) - sizeof(int))
    goto err_kfree;

  printk("\nI2C-CGEB: FOUND LD_MAGIC at base addr %x, mapped %x", base, pcur);
  if (low_desc->size >= sizeof(struct cgeb_low_desc)
      && low_desc->hi_desc_phys_addr)
    high_desc_phys = low_desc->hi_desc_phys_addr;
  else
    high_desc_phys = 0xfff00000;

  high_desc_len = 0x100000;

  pr_debug("CGEB: Looking for CGEB hi desc between phys 0x%x and 0x%x\n",
	   high_desc_phys, high_desc_len);
  high_desc_virt = ioremap_cache(high_desc_phys, high_desc_len);
  if (!high_desc_virt)
    goto err_kfree;

  // At this point, we allocated board memory plus io mem.  Need to release
  // both on error-out.
  pr_debug("CGEB: Looking for CGEB hi desc between virt 0x%p and 0x%p\n",
	   high_desc_virt, high_desc_virt + high_desc_len - 1);

  printk("\nI2C-CGEB: Looking for HD MAGIC at %x, length %x",high_desc_phys, high_desc_len);

  high_desc = (struct cgeb_high_desc *)cgeb_find_magic((unsigned char *)high_desc_virt, high_desc_len, CGEB_HD_MAGIC);
  if (!high_desc)
    goto err_iounmap;

  pr_debug("CGEB: Found CGEB_HD_MAGIC\n");
  printk("\nI2C-CGEB: FOUND HD_MAGIC at base addr %x, mapped %x", high_desc_virt, (unsigned int)high_desc);

  if (high_desc->size < sizeof(struct cgeb_high_desc))
    goto err_iounmap;

  board->code = __vmalloc(high_desc->code_size, GFP_KERNEL,
			  PAGE_KERNEL_EXEC);
  if (!board->code)
    goto err_iounmap;

  memcpy((char *)board->code, (char *)high_desc, high_desc->code_size);
  // At this point we have board allocated, high io-mem still mapped, and now
  // the code space allocated & copied.  Need to free all 3 on error-out.
  
  high_desc = (struct cgeb_high_desc *)board->code;
  board->entry = (void *)((unsigned char *)board->code + high_desc->entry_rel);
  board->ds = get_data_segment();

  printk("\nCGEB: high phys %x, low phys %x, board entry %x, ds %x\n",high_desc_phys,base, board->entry,board->ds);
  ret = cgeb_call_simple(board, CgebGetCgebVersion, 0, NULL, &dw);
  if (ret)
    goto err_vfree;

  printk("\nCGEB: Made it past first call version %x\n",CGEB_GET_VERSION_MAJOR(dw));
  goto err_vfree;
  
  if (CGEB_GET_VERSION_MAJOR(dw) != CGEB_VERSION_MAJOR)
    goto err_vfree;

  pr_debug("CGEB: BIOS interface revision: %d.%d\n",
	   dw >> 16, dw & 0xffff);

  if (high_desc->data_size)
    {
      board->data = vmalloc(high_desc->data_size);
      if (!board->data)
	goto err_vfree;
    }
  else
    {
      ret = cgeb_call_simple(board, CgebGetDataSize, 0, NULL, &dw);
      if (!ret && dw)
	{
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

  iounmap(high_desc_phys);
  return board;

err_free_map:
  cgeb_unmap_memory(board);
err_vfree_data:
  vfree(board->data);
err_vfree:
  vfree(board->code);
err_iounmap:
  iounmap(high_desc_phys);
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

static unsigned int cgeb_i2c_get_type(struct cgeb_board_data *brd, int unit)
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
  struct cgeb_pdata pdata __aligned(8);
  struct cgeb_board_data *board=0;
  unsigned int basenibbles,baseaddr;
  int i, ret;
  unsigned int i2c_count;
  int num_devices = 0;

  //  for (base = 0xf0000; base >= 0xc0000; base -= 0x10000) {
  for (basenibbles=0xfedc0; (baseaddr=basenibbles&0xf0000); basenibbles<<=4)
    {
      printk("\nI2C CGEB board base addr %x", baseaddr);
      board = cgeb_open(baseaddr, 0x10000);
      if (board)
	break;
    }

  if (!board)
    {
      printk(KERN_ERR "There seems to be NO Congatec CGEB interface!\n");
      return -ENODEV;
    }
  cgeb_board = board;

  pdata.board = board;

  cgeb_call_simple(board, CgebI2CCount, 0, NULL, &i2c_count);

  board->num_devices = i2c_count;
  board->devices = kzalloc(sizeof(void *) * (board->num_devices),
			   GFP_KERNEL);
  if (!board->devices) {
    printk(KERN_ERR "There seem to be NO Congatec board devices!\n");
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
MODULE_DESCRIPTION("Congatec CGEB i2c driver");
MODULE_LICENSE("GPL");
