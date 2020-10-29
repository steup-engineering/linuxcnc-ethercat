//
//    Copyright (C) 2019 Sascha Ittner <sascha.ittner@modusoft.de>
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
#include "lcec_nanopd4e.h"
#include "lcec_class_rt_sdo.h"

#define nanopd4e_PULSES_PER_REV_DEFLT 3600
#define nanopd4e_FAULT_AUTORESET_DELAY_NS 100000000LL
#define OPMODE_PROFILE_POSITION 1
#define OPMODE_VELOCITY 2
#define OPMODE_HOMING 6
#define OPMODE_NONE 0

typedef struct {
  hal_float_t *pos_cmd;
  hal_s32_t *pos_cmd_raw;
  hal_float_t *pos_fb;
  hal_s32_t *pos_fb_raw;
  hal_bit_t *fault;
  hal_bit_t *fault_reset;
  hal_bit_t *enable;
 
  hal_bit_t *opmode_no_mode;
  hal_bit_t *opmode_homing;
  hal_bit_t *opmode_profile_position;
  hal_bit_t *opmode_velocity;
  hal_s32_t *opmode_number;

  hal_bit_t *jog_pos;
  hal_bit_t *jog_neg;
  
  hal_bit_t *stat_switchon_ready;
  hal_bit_t *stat_switched_on;
  hal_bit_t *stat_op_enabled;
  hal_bit_t *stat_fault;
  hal_bit_t *stat_volt_enabled;
  hal_bit_t *stat_quick_stop;
  hal_bit_t *stat_switchon_disabled;
  hal_bit_t *stat_warning;
  hal_bit_t *stat_remote;
  hal_bit_t *stat_target_reached;
  hal_bit_t *stat_soft_end_active;
  hal_bit_t *stat_closed_loop_active;
  hal_bit_t *stat_homed;
  hal_bit_t *stat_homing;
  hal_bit_t *stat_home_fault;
  hal_u32_t *stat_error_code;

  hal_bit_t *new_setpoint;
  hal_bit_t *new_setpoint_instant;
  hal_bit_t *home;
  // Parameter
  hal_float_t pos_scale;

  hal_float_t velocity_cmd;

  hal_bit_t auto_fault_reset;

  //sdo parameter
  lcec_class_rt_sdo_data_t pos_softend_min;
  lcec_class_rt_sdo_data_t pos_softend_max;
  lcec_class_rt_sdo_data_t pos_profile_velo;
  lcec_class_rt_sdo_data_t pos_end_velo;
  lcec_class_rt_sdo_data_t pos_accel;
  lcec_class_rt_sdo_data_t pos_deccel;
  lcec_class_rt_sdo_data_t pos_option_code;
  lcec_class_rt_sdo_data_t pos_foll_err_window;
  lcec_class_rt_sdo_data_t pos_foll_err_time;
  lcec_class_rt_sdo_data_t pos_foll_err_reaction;

  lcec_class_rt_sdo_data_t velo_accel;
  lcec_class_rt_sdo_data_t velo_deccel;
  lcec_class_rt_sdo_data_t velo_min_limit;
  lcec_class_rt_sdo_data_t velo_max_limit;

  lcec_class_rt_sdo_data_t home_offset;
  lcec_class_rt_sdo_data_t home_method; 
  lcec_class_rt_sdo_data_t home_search_velo;
  lcec_class_rt_sdo_data_t home_latch_velo;
  lcec_class_rt_sdo_data_t home_block_current;
  lcec_class_rt_sdo_data_t home_block_time;

  lcec_class_rt_sdo_data_t inp_conf_special_function;
  lcec_class_rt_sdo_data_t inp_conf_invert;
  lcec_class_rt_sdo_data_t inp_conf_force_enable;
  lcec_class_rt_sdo_data_t inp_conf_force;
  lcec_class_rt_sdo_data_t inp_conf_voltage_range;
  lcec_class_rt_sdo_data_t inp_conf_diff_input;

  lcec_class_rt_sdo_data_t pos_polarity;

  lcec_class_rt_sdo_data_t error_code;



  // internal
  hal_float_t pos_scale_old;
  hal_bit_t enable_old;
  hal_bit_t jog_pos_old;
  hal_bit_t jog_neg_old;
  hal_bit_t stat_homed_old;
  hal_bit_t stat_fault_old;
  hal_bit_t setpoint_ack_old;
  hal_bit_t read_error;

  double pos_scale_rcpt;

  unsigned int status_pdo_os;
  unsigned int curr_pos_pdo_os;
  unsigned int op_mode_in_pdo_os;
  unsigned int control_pdo_os;
  unsigned int target_pos_pdo_os;
  unsigned int drive_submode_pdo_os;
  unsigned int op_mode_pdo_os;

  long long auto_fault_reset_delay;
  ec_sdo_request_t *velo_sdo;

  

} lcec_nanopd4e_data_t;

static const lcec_pindesc_t slave_pins[] = {
  { HAL_FLOAT, HAL_IN, offsetof(lcec_nanopd4e_data_t, pos_cmd), "%s.%s.%s.pos-cmd" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, pos_fb), "%s.%s.%s.pos-fb" },
  { HAL_S32, HAL_OUT, offsetof(lcec_nanopd4e_data_t, pos_cmd_raw), "%s.%s.%s.pos-cmd-raw" },
  { HAL_S32, HAL_OUT, offsetof(lcec_nanopd4e_data_t, pos_fb_raw), "%s.%s.%s.pos-fb-raw" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, fault), "%s.%s.%s.fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, fault_reset), "%s.%s.%s.fault-reset" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nanopd4e_data_t, enable), "%s.%s.%s.enable" },
  { HAL_BIT, HAL_IO, offsetof(lcec_nanopd4e_data_t, new_setpoint), "%s.%s.%s.new-setpoint" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nanopd4e_data_t, new_setpoint_instant), "%s.%s.%s.new-setpoint-instant" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_switchon_ready), "%s.%s.%s.stat-switchon-ready" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_switched_on), "%s.%s.%s.stat-switched-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_op_enabled), "%s.%s.%s.stat-op-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_fault), "%s.%s.%s.stat-fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_volt_enabled), "%s.%s.%s.stat-volt-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_quick_stop), "%s.%s.%s.stat-quick-stop" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_switchon_disabled), "%s.%s.%s.stat-switchon-disabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_warning), "%s.%s.%s.stat-warning" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_remote), "%s.%s.%s.stat-remote" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_target_reached), "%s.%s.%s.stat-target-reached" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_soft_end_active), "%s.%s.%s.stat-soft-end-active" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_closed_loop_active), "%s.%s.%s.stat-closed-loop-active" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_homed), "%s.%s.%s.stat-homed" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_homing), "%s.%s.%s.stat-homing" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, stat_home_fault), "%s.%s.%s.stat-home-fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, opmode_no_mode), "%s.%s.%s.opmode-no-mode" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, opmode_homing), "%s.%s.%s.opmode-homing" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, opmode_velocity), "%s.%s.%s.opmode-velocity" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nanopd4e_data_t, opmode_profile_position), "%s.%s.%s.opmode-profile-position" },
  { HAL_S32, HAL_OUT, offsetof(lcec_nanopd4e_data_t, opmode_number), "%s.%s.%s.opmode-number" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nanopd4e_data_t, jog_pos), "%s.%s.%s.jog-pos" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nanopd4e_data_t, jog_neg), "%s.%s.%s.jog-neg" },
  { HAL_BIT, HAL_IO, offsetof(lcec_nanopd4e_data_t, home), "%s.%s.%s.home" },


  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_pindesc_t slave_params[] = {
  { HAL_FLOAT, HAL_RW, offsetof(lcec_nanopd4e_data_t, pos_scale), "%s.%s.%s.pos-scale" },
  { HAL_BIT, HAL_RW, offsetof(lcec_nanopd4e_data_t, auto_fault_reset), "%s.%s.%s.auto-fault-reset" },
  { HAL_FLOAT, HAL_RW, offsetof(lcec_nanopd4e_data_t, velocity_cmd), "%s.%s.%s.velocity-cmd" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static ec_pdo_entry_info_t lcec_nanopd4e_in[] = {
   {0x6041, 0x00, 16}, // Status Word (U16)
   {0x6064, 0x00, 32}, // Current Position (S32)
   {0x6061, 0x00, 8}  // Op Mode IN (S8)
};

static ec_pdo_entry_info_t lcec_nanopd4e_out[] = {
   {0x6040, 0x00, 16}, // Control Word (U16)
   {0x607A, 0x00, 32}, // Target Position (S32)
   {0x3202, 0x00, 32}, // drive submode (U32)
   {0x6060, 0x00, 8}  // modes of operation (s8)
};

static ec_pdo_info_t lcec_nanopd4e_pdos_out[] = {
    {0x1600,  4, lcec_nanopd4e_out}
};

static ec_pdo_info_t lcec_nanopd4e_pdos_in[] = {
    {0x1A00, 3, lcec_nanopd4e_in}
};

static ec_sync_info_t lcec_nanopd4e_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT,  0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_nanopd4e_pdos_out},
    {3, EC_DIR_INPUT,  1, lcec_nanopd4e_pdos_in},
    {0xff}
};

ec_sdo_request_t *create_sdo_request(struct lcec_slave *slave, uint16_t index, uint8_t subindex, size_t size);

void lcec_nanopd4e_check_scales(lcec_nanopd4e_data_t *hal_data);
void lcec_nanopd4e_check_parameters(lcec_nanopd4e_data_t *hal_data);

void lcec_nanopd4e_read(struct lcec_slave *slave, long period);
void lcec_nanopd4e_write(struct lcec_slave *slave, long period);

ec_sdo_request_t *create_sdo_request(struct lcec_slave *slave, uint16_t index, uint8_t subindex, size_t size) {
  lcec_master_t *master = slave->master;
  ec_sdo_request_t *result;

  if ((result =ecrt_slave_config_create_sdo_request(slave->config,index, subindex, size))== NULL) {
   rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure sdo request slave %s.%s sdo 0x%04x\n", master->name, slave->name, index);
  }
  return result;
}

int lcec_nanopd4e_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_nanopd4e_data_t *hal_data;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_nanopd4e_read;
  slave->proc_write = lcec_nanopd4e_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_nanopd4e_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_nanopd4e_data_t));
  slave->hal_data = hal_data;

  // initialize sync info
  slave->sync_info = lcec_nanopd4e_syncs;

  // initialize POD entries

  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6041, 0x00, &hal_data->status_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6064, 0x00, &hal_data->curr_pos_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6061, 0x00, &hal_data->op_mode_in_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6040, 0x00, &hal_data->control_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x607A, 0x00, &hal_data->target_pos_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3202, 0x00, &hal_data->drive_submode_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6060, 0x00, &hal_data->op_mode_pdo_os, NULL);

  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // initialize variables
  hal_data->pos_scale = (double) nanopd4e_PULSES_PER_REV_DEFLT;
  hal_data->pos_scale_old = hal_data->pos_scale + 1.0;
  hal_data->pos_scale_rcpt = 1.0;
  hal_data->auto_fault_reset = 1;
  hal_data->velocity_cmd = 1.0;


  // initialize sdo requests 

  hal_data->velo_sdo = create_sdo_request(slave, 0x6042, 0x00, 2);

 // init subclasses for rt sdo
  //position
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_softend_min, "pos-softend-min" ,0x607D,0x01,4,1)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_softend_max, "pos-softend-max" ,0x607D,0x02,4,1)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_profile_velo, "pos-profile-velo" ,0x6081,0x00,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_end_velo, "pos-end-velo" ,0x6082,0x00,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_accel, "pos-accel" ,0x6083,0x0,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_deccel, "pos-deccel" ,0x6084,0x0,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_option_code, "pos-option-code" ,0x60F2,0x0,2,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_polarity, "pos-polarity" ,0x607E,0x00,1,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_foll_err_window, "pos-following-err-window" ,0x6065,0x00,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_foll_err_time, "pos-following-err-time" ,0x6066,0x00,2,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->pos_foll_err_reaction, "pos-following-err-reaction" ,0x3700,0x00,2,1)) != 0) {
    return err;
  }

  //velocity
  if ((err = class_rt_sdo_init(slave, &hal_data->velo_accel, "velo-accel" ,0x6048,0x01,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->velo_deccel, "velo-deccel" ,0x6049,0x01,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->velo_min_limit, "velo-min-limit" ,0x6046,0x01,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->velo_max_limit, "velo-max-limit" ,0x6046,0x02,4,0)) != 0) {
    return err;
  }
  //homing
  if ((err = class_rt_sdo_init(slave, &hal_data->home_offset, "home-offset" ,0x607C,0x00,4,1)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->home_method, "home-method" ,0x6098,0x00,1,1)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->home_search_velo, "home-search-velo" ,0x6099,0x01,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->home_latch_velo, "home-latch-velo" ,0x6099,0x02,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->home_block_current, "home-block-current" ,0x203A,0x01,4,1)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->home_block_time, "home-block-time" ,0x203A,0x02,4,0)) != 0) {
    return err;
  }
  //io conf
  if ((err = class_rt_sdo_init(slave, &hal_data->inp_conf_special_function, "inp-conf-special-function" ,0x3240,0x01,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->inp_conf_invert, "inp-conf-invert" ,0x3240,0x02,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->inp_conf_force_enable, "inp-conf-force_enable" ,0x3240,0x03,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->inp_conf_force, "inp-conf-force" ,0x3240,0x04,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->inp_conf_voltage_range, "inp-conf-voltage-range" ,0x3240,0x05,4,0)) != 0) {
    return err;
  }
  if ((err = class_rt_sdo_init(slave, &hal_data->inp_conf_diff_input, "inp-conf-diff-input" ,0x3240,0x06,4,0)) != 0) {
    return err;
  }
  //misc
  if ((err = class_rt_sdo_init(slave, &hal_data->error_code, "error-code" ,0x603F,0x00,2,0)) != 0) {
    return err;
  }

  return 0;
}



void lcec_nanopd4e_check_scales(lcec_nanopd4e_data_t *hal_data) {
  // check for change in scale value
  if (hal_data->pos_scale != hal_data->pos_scale_old) {

    // scale value has changed, test and update it
    if ((hal_data->pos_scale < 1e-20) && (hal_data->pos_scale > -1e-20)) {
      // value too small, divide by zero is a bad thing
      hal_data->pos_scale = 1.0;
    }

    // save new scale to detect future changes
    hal_data->pos_scale_old = hal_data->pos_scale;

    // we actually want the reciprocal
    hal_data->pos_scale_rcpt = 1.0 / hal_data->pos_scale;
  }
}

void lcec_nanopd4e_check_parameters(lcec_nanopd4e_data_t *hal_data) {

  class_rt_sdo_check(&hal_data->pos_softend_min);
  class_rt_sdo_check(&hal_data->pos_softend_max);
  class_rt_sdo_check(&hal_data->pos_profile_velo);
  class_rt_sdo_check(&hal_data->pos_end_velo);
  class_rt_sdo_check(&hal_data->pos_accel);
  class_rt_sdo_check(&hal_data->pos_deccel);
  class_rt_sdo_check(&hal_data->pos_option_code);
  class_rt_sdo_check(&hal_data->pos_polarity);
  class_rt_sdo_check(&hal_data->pos_foll_err_window);
  class_rt_sdo_check(&hal_data->pos_foll_err_time);
  class_rt_sdo_check(&hal_data->pos_foll_err_reaction);

  class_rt_sdo_check(&hal_data->velo_accel);
  class_rt_sdo_check(&hal_data->velo_deccel);
  class_rt_sdo_check(&hal_data->velo_min_limit);
  class_rt_sdo_check(&hal_data->velo_max_limit);

  class_rt_sdo_check(&hal_data->home_offset);
  class_rt_sdo_check(&hal_data->home_method);
  class_rt_sdo_check(&hal_data->home_search_velo);
  class_rt_sdo_check(&hal_data->home_latch_velo);
  class_rt_sdo_check(&hal_data->home_block_current);
  class_rt_sdo_check(&hal_data->home_block_time);

  class_rt_sdo_check(&hal_data->inp_conf_special_function);
  class_rt_sdo_check(&hal_data->inp_conf_invert);
  class_rt_sdo_check(&hal_data->inp_conf_force_enable);
  class_rt_sdo_check(&hal_data->inp_conf_force);
  class_rt_sdo_check(&hal_data->inp_conf_voltage_range);
  class_rt_sdo_check(&hal_data->inp_conf_diff_input);


}
  
void lcec_nanopd4e_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_nanopd4e_data_t *hal_data = (lcec_nanopd4e_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  int8_t opmodein;
  bool setpoint_ack = 0;

  // check for change in scale value
  lcec_nanopd4e_check_scales(hal_data);
  lcec_nanopd4e_check_parameters(hal_data);

  // read Modes of Operation
  opmodein = EC_READ_S8(&pd[hal_data->op_mode_in_pdo_os]);
  *(hal_data->opmode_no_mode)  = (opmodein == OPMODE_NONE);
  *(hal_data->opmode_homing)   = (opmodein == OPMODE_HOMING);
  *(hal_data->opmode_velocity) = (opmodein == OPMODE_VELOCITY);
  *(hal_data->opmode_profile_position)  = (opmodein == OPMODE_PROFILE_POSITION);
  *(hal_data->opmode_number )  = opmodein;
  
  // read status
  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  *(hal_data->stat_switchon_ready)    = (status >> 0) & 1;
  *(hal_data->stat_switched_on)       = (status >> 1) & 1;
  *(hal_data->stat_op_enabled)        = (status >> 2) & 1;
  *(hal_data->stat_fault)             = (status >> 3) & 1;
  *(hal_data->stat_volt_enabled)      = (status >> 4) & 1;
  *(hal_data->stat_quick_stop)        = (status >> 5) & 1;
  *(hal_data->stat_switchon_disabled) = (status >> 6) & 1;
  *(hal_data->stat_warning)           = (status >> 7) & 1;
  *(hal_data->stat_remote)            = (status >> 9) & 1;

  *(hal_data->stat_soft_end_active)   = (status >> 11) & 1;
  *(hal_data->stat_closed_loop_active)= (status >> 15) & 1;

  if (*(hal_data->opmode_profile_position) || *(hal_data->opmode_velocity)) {
    *(hal_data->stat_target_reached) = (status >> 10) & 1;
    } else {
      *(hal_data->stat_target_reached) = 0;
    }

  //position states 
  if (*hal_data->opmode_profile_position) {
    setpoint_ack = (status >> 12) & 1;
    //reset setpoint command   
    if(*(hal_data->new_setpoint) && (setpoint_ack && !hal_data->setpoint_ack_old)) {
      *(hal_data->new_setpoint) = 0; 
    }
  }
  hal_data->setpoint_ack_old = setpoint_ack; 

  //home states  
  if (*(hal_data->opmode_homing) && *(hal_data->home)) {
    *(hal_data->stat_homed)     = ((status >> 10) & 1) && ((status >> 12) & 1);
    *(hal_data->stat_homing)   = !((status >> 12) & 1) && !((status >> 10) & 1);
    *(hal_data->stat_home_fault)= (status >> 13) & 1;
    // reset home command   
    if (*(hal_data->home) && (*(hal_data->stat_homed) && !hal_data->stat_homed_old) && !*(hal_data->stat_homing)) {
      *(hal_data->home) = 0;
    }
  }
  hal_data->stat_homed_old = *(hal_data->stat_homed);
  
  // read error code
  if (*(hal_data->stat_fault) && !hal_data->stat_fault_old ) {
    class_rt_sdo_read_and_update(&hal_data->error_code);
    hal_data->read_error = 1;
    
  }
  if (hal_data->read_error && class_rt_sdo_read_and_update(&hal_data->error_code)) {
    hal_data->read_error = 0; 
  }
  hal_data->stat_fault_old = *(hal_data->stat_fault);
  *(hal_data->stat_error_code) = hal_data->error_code.u_parameter;

  if (!hal_data->stat_fault) {
    hal_data->stat_error_code = 0;
  }

  // read position feedback
  *(hal_data->pos_fb_raw) = EC_READ_S32(&pd[hal_data->curr_pos_pdo_os]);
  *(hal_data->pos_fb) = ((double) *(hal_data->pos_fb_raw)) * hal_data->pos_scale_rcpt;

  // update fault output
  if (hal_data->auto_fault_reset_delay > 0) {
    hal_data->auto_fault_reset_delay -= period;
    *(hal_data->fault) = 0;
  } else {
    *(hal_data->fault) = *(hal_data->stat_fault) && *(hal_data->enable);
  }
}

void lcec_nanopd4e_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_nanopd4e_data_t *hal_data = (lcec_nanopd4e_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  int enable_edge, jog_rising_edge;
  uint16_t control;
  uint32_t submode = 0;
  int8_t opmode;
  float jogvelo;

  // detect enable edge
  enable_edge = *(hal_data->enable) && !hal_data->enable_old;
  hal_data->enable_old = *(hal_data->enable);

  // write control register
  control = (1 << 2); // quick stop is not supported
  if (*(hal_data->stat_fault)) {
    if (*(hal_data->fault_reset)) {
      control |= (1 << 7); // fault reset
    }
    if (hal_data->auto_fault_reset && enable_edge) {
      hal_data->auto_fault_reset_delay = nanopd4e_FAULT_AUTORESET_DELAY_NS;
      control |= (1 << 7); // fault reset
    }
  } else {
    if (*(hal_data->enable)) {
      control |= (1 << 1); // enable voltage
      if (*(hal_data->stat_switchon_ready)) {
        control |= (1 << 0); // switch on
        if (*(hal_data->stat_switched_on)) {
          control |= (1 << 3); // enable op
        }
      }
    }
  }

  //new set point or jog active or homing
  if (*(hal_data->opmode_profile_position)) {
    control |= (*(hal_data->new_setpoint) << 4);
  } else if (*(hal_data->opmode_velocity)) {
      control |= ((*(hal_data->jog_pos) || *(hal_data->jog_neg)) << 4);
    } else if (*(hal_data->opmode_homing)) {
        control |= (*(hal_data->home) << 4);
    } 
  
  //setpoint instant 
  control |= (*(hal_data->new_setpoint_instant) << 5); 

        
  EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);

  //detect jog rising edge
  jog_rising_edge = (*(hal_data->jog_pos) && !hal_data->jog_pos_old) || (*(hal_data->jog_neg) && !hal_data->jog_neg_old);
  hal_data->jog_pos_old = *(hal_data->jog_pos);
  hal_data->jog_neg_old = *(hal_data->jog_neg);

  //set JOG velocity
  if (*(hal_data->jog_pos)) {
    jogvelo = (hal_data->velocity_cmd);

  } else {
      jogvelo = -(hal_data->velocity_cmd);
  }

  if (jog_rising_edge) {
    EC_WRITE_S16(ecrt_sdo_request_data(hal_data->velo_sdo), jogvelo);
    ecrt_sdo_request_write(hal_data->velo_sdo);
  } 
 
  //OP Mode 

  //jog , set velo mode
  if (*(hal_data->stat_op_enabled) && jog_rising_edge) {
    opmode = OPMODE_VELOCITY;
  }
  //set back to profile position
  if (!*(hal_data->jog_pos) && !*(hal_data->jog_neg) && !*(hal_data->home)) {
    opmode = OPMODE_PROFILE_POSITION;
  }
  //mode Home 
  if (*(hal_data->home)) {
    opmode = OPMODE_HOMING;
  }

  //set drive OP mode
  EC_WRITE_S8(&pd[hal_data->op_mode_pdo_os], opmode);

  //enable closed loop mode
  submode |= (1 << 0); 
  EC_WRITE_U32(&pd[hal_data->drive_submode_pdo_os], submode);

  // write position command
  *(hal_data->pos_cmd_raw) = (int32_t) (*(hal_data->pos_cmd) * hal_data->pos_scale);
  EC_WRITE_S32(&pd[hal_data->target_pos_pdo_os], *(hal_data->pos_cmd_raw));
}

