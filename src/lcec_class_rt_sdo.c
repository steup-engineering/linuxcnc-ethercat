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

#include "lcec.h"
#include "lcec_class_rt_sdo.h"


static lcec_pindesc_t slave_params_s[]  = {
  { HAL_S32, HAL_RW, offsetof(lcec_class_rt_sdo_data_t, s_parameter), "%s.%s.%s.sdo-%s" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static lcec_pindesc_t slave_params_u[]  = {
  { HAL_U32, HAL_RW, offsetof(lcec_class_rt_sdo_data_t, u_parameter), "%s.%s.%s.sdo-%s" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

int class_rt_sdo_read_and_update(lcec_class_rt_sdo_data_t *hal_data);

int class_rt_sdo_init(struct lcec_slave *slave, lcec_class_rt_sdo_data_t *hal_data, const char *name, uint16_t index,
   uint8_t subindex, size_t size, bool sign) {
  lcec_master_t *master = slave->master;
  int err;

  if (sign) {
    // export parameters
    if ((err = lcec_param_newf_list(hal_data, slave_params_s, LCEC_MODULE_NAME, master->name, slave->name, name)) != 0) {
      return err;
    }
  } else {
    if ((err = lcec_param_newf_list(hal_data, slave_params_u, LCEC_MODULE_NAME, master->name, slave->name, name)) != 0) {
      return err;
    }
  }
  // create sdo request
  if ((hal_data->sdo_request_data =ecrt_slave_config_create_sdo_request(slave->config,index, subindex, size))== NULL) {
   rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure sdo request slave %s.%s sdo 0x%04 %s \n", master->name, slave->name,
     index, name);
   return -1; 
  }

  hal_data->size = size;
  hal_data->sign = sign;

  hal_data->do_init = 0;
  hal_data->do_write = 0;

  return 0;
}

void class_rt_sdo_check(lcec_class_rt_sdo_data_t *hal_data) {

  //init if no value is given, read from device
  if (!hal_data->do_init && ((hal_data->sign && hal_data->s_parameter == 0) || (!hal_data->sign && hal_data->u_parameter == 0))) {
    //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "do_init false\n");
    if (class_rt_sdo_read_and_update(hal_data) == 1) {
      hal_data->do_init = 1;
          //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "do_init true\n");
    }
  return;
  } else {
    hal_data->do_init = 1;
  }
  //check parameter corners
  switch (hal_data->size) {
    case 1:
      if (hal_data->sign) {
        hal_data->s_parameter = hal_data->s_parameter < INT8_MIN ? INT8_MIN : hal_data->s_parameter ;
        hal_data->s_parameter = hal_data->s_parameter > INT8_MAX ? INT8_MAX : hal_data->s_parameter ;
      } else {
        hal_data->s_parameter = hal_data->s_parameter < 0 ? 0 : hal_data->s_parameter ;
        hal_data->s_parameter = hal_data->s_parameter > UINT8_MAX ? UINT8_MAX : hal_data->s_parameter ;
      }      
    case 2:
      if (hal_data->sign) {
        hal_data->s_parameter = hal_data->s_parameter < INT16_MIN ? INT16_MIN : hal_data->s_parameter ;
        hal_data->s_parameter = hal_data->s_parameter > INT16_MAX ? INT16_MAX : hal_data->s_parameter ;
      } else {
        hal_data->s_parameter = hal_data->s_parameter < 0 ? 0 : hal_data-> s_parameter ;
        hal_data->s_parameter = hal_data->s_parameter > UINT16_MAX ? UINT16_MAX : hal_data->s_parameter ;
      }      
    case 4:
      if (hal_data->sign) {
        hal_data->s_parameter = hal_data->s_parameter < INT32_MIN ? INT32_MIN : hal_data->s_parameter ;
        hal_data->s_parameter = hal_data->s_parameter > INT32_MAX ? INT32_MAX : hal_data->s_parameter ;
      } else {
        hal_data->s_parameter = hal_data->s_parameter < 0 ? 0 : hal_data->s_parameter ;
        hal_data->s_parameter = hal_data->s_parameter > UINT32_MAX ? UINT32_MAX : hal_data->s_parameter ;
      }      
  }

  //check for new value, set request
  if (!hal_data->do_write) {
    if (hal_data->sign) {
      if (hal_data->s_parameter != hal_data->s_parameter_old) {
        //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "changed %i04 \n", hal_data->s_parameter);
        switch (hal_data->size) {
          case 1:
            EC_WRITE_S8(ecrt_sdo_request_data(hal_data->sdo_request_data), hal_data->s_parameter);
          case 2:
            EC_WRITE_S16(ecrt_sdo_request_data(hal_data->sdo_request_data), hal_data->s_parameter);
            //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "size klappt %i04 \n", hal_data->s_parameter);
          case 4:
            EC_WRITE_S32(ecrt_sdo_request_data(hal_data->sdo_request_data), hal_data->s_parameter);
        }
        hal_data->do_write = 1; 
        ecrt_sdo_request_write(hal_data->sdo_request_data); 
        //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "write value\n");

      }
    } else {
        if (hal_data->u_parameter != hal_data->u_parameter_old) { 
          switch (hal_data->size) {
            case 1:
              EC_WRITE_U8(ecrt_sdo_request_data(hal_data->sdo_request_data), hal_data->u_parameter);
            case 2:
              EC_WRITE_U16(ecrt_sdo_request_data(hal_data->sdo_request_data), hal_data->u_parameter);
            case 4:
              EC_WRITE_U32(ecrt_sdo_request_data(hal_data->sdo_request_data), hal_data->u_parameter);
          }
          hal_data->do_write = 1;
          ecrt_sdo_request_write(hal_data->sdo_request_data);  
        }
    }      
  }
  
  //wait for sdo request written 
  switch (ecrt_sdo_request_state(hal_data->sdo_request_data)) {
    case EC_REQUEST_UNUSED: // request was not used yet
      break;
    case EC_REQUEST_BUSY:
      //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "Request to Write,But Still busy...\n");
      break;
    case EC_REQUEST_SUCCESS:
      if (hal_data->do_write) {
        //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "sdo_operation_mode_write write value ok\n");
        hal_data->s_parameter_old = hal_data->s_parameter;
        hal_data->u_parameter_old = hal_data->u_parameter;
        hal_data->do_write = 0;
      }
    break;
    case EC_REQUEST_ERROR:
      //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX  "Failed to read SDO state!\n");
      hal_data->do_write = 0;
    break;
  }    
  
}


int class_rt_sdo_read_and_update(lcec_class_rt_sdo_data_t *hal_data) {
  int success = 0;

  if ((ecrt_sdo_request_state(hal_data->sdo_request_data) == EC_REQUEST_UNUSED)) {
    ecrt_sdo_request_read(hal_data->sdo_request_data); 
    //rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "read request \n");
    return success;
  }

  if ((ecrt_sdo_request_state(hal_data->sdo_request_data) != EC_REQUEST_BUSY)) {
    //check typ and read parameter value
 
    if (hal_data->sign) {
      switch (hal_data->size) {
        case 1:
          hal_data->s_parameter = EC_READ_S8(ecrt_sdo_request_data(hal_data->sdo_request_data));
        case 2:
          hal_data->s_parameter = EC_READ_S16(ecrt_sdo_request_data(hal_data->sdo_request_data));
        case 4:
          hal_data->s_parameter = EC_READ_S32(ecrt_sdo_request_data(hal_data->sdo_request_data));
       }
       hal_data->s_parameter_old = hal_data->s_parameter; 
    } else {
        switch (hal_data->size) {
          case 1:
            hal_data->u_parameter = EC_READ_U8(ecrt_sdo_request_data(hal_data->sdo_request_data));
          case 2:
            hal_data->u_parameter = EC_READ_U16(ecrt_sdo_request_data(hal_data->sdo_request_data));
          case 4:
            hal_data->u_parameter = EC_READ_U32(ecrt_sdo_request_data(hal_data->sdo_request_data));
         }
         hal_data->u_parameter_old = hal_data->u_parameter; 
    }
  
    success = 1; 
    return success;
  }
  return success;
}





