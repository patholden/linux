/*
 * pinctrl-byt.h: BayTrail GPIO pinctrl header file
 *
 * Copyright (C) 2013 Intel Corporation
 * Author: Chew, Kean Ho <kean.ho.chew@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#ifdef CONFIG_PINCTRL_BAYTRAIL_DEVICE
struct byt_pinctrl_port {
	char *unique_id;
};
#endif
