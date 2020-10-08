//
//    Copyright (C) 2018 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//
#ifndef _LCEC_CLASS_RT_SDO_H_
#define _LCEC_CLASS_RT_SDO_H_

#include "lcec.h"

typedef struct {
  hal_s32_t s_parameter;
  hal_u32_t u_parameter;

  ec_sdo_request_t *sdo_request_data;

  int32_t s_parameter_old;
  uint32_t u_parameter_old; 
  
  bool sign;
  int size;

  bool do_init;
  bool do_write; 

} lcec_class_rt_sdo_data_t;

int class_rt_sdo_init(struct lcec_slave *slave, lcec_class_rt_sdo_data_t *hal_data, const char *name, uint16_t index,
   uint8_t subindex, size_t size, bool sign);
void class_rt_sdo_check(lcec_class_rt_sdo_data_t *hal_data);
int class_rt_sdo_read_and_update(lcec_class_rt_sdo_data_t *hal_data);

#endif
