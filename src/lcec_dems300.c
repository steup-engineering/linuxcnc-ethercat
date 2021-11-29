//
//    Copyright (C) 2021 Dominik Braun <dominik.braun@eventor.de>
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
#include "lcec_dems300.h"

#define MS300_FAULT_AUTORESET_DELAY_NS 100000000LL
#define OPMODE_VELOCITY 2



typedef struct {
  hal_float_t *vel_fb_rpm;
  hal_float_t *vel_fb_rpm_abs;
  hal_float_t *vel_rpm_cmd;

  hal_bit_t *stat_switch_on_ready;
  hal_bit_t *stat_switched_on;
  hal_bit_t *stat_op_enabled;
  hal_bit_t *stat_fault;
  hal_bit_t *stat_volt_enabled;
  hal_bit_t *stat_quick_stoped;
  hal_bit_t *stat_switch_on_disabled;
  hal_bit_t *stat_warning;
  hal_bit_t *stat_remote;
  hal_bit_t *stat_at_speed;
  
  hal_bit_t *quick_stop;
  hal_bit_t *enable;
  hal_bit_t *fault_reset;
  hal_bit_t *halt;
 
  hal_s32_t *mode_op_display; 

  hal_bit_t auto_fault_reset;
  
  unsigned int status_pdo_os;
  unsigned int currvel_pdo_os;
  unsigned int mode_op_display_pdo_os;

  unsigned int control_pdo_os;
  unsigned int cmdvel_pdo_os;
  unsigned int mode_op_pdo_os;


  hal_bit_t enable_old;
  hal_bit_t internal_fault;

  long long auto_fault_reset_delay;  

} lcec_dems300_data_t;

static const lcec_pindesc_t slave_pins[] = {
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_dems300_data_t, vel_fb_rpm), "%s.%s.%s.vel-fb-rpm" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_dems300_data_t, vel_fb_rpm_abs), "%s.%s.%s.vel-fb-rpm-abs" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_switch_on_ready), "%s.%s.%s.stat-switch-on-ready" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_switched_on), "%s.%s.%s.stat-switched-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_op_enabled), "%s.%s.%s.stat-op-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_fault), "%s.%s.%s.stat-fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_volt_enabled), "%s.%s.%s.stat-volt-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_quick_stoped), "%s.%s.%s.stat-quick-stoped" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_switch_on_disabled), "%s.%s.%s.stat-switch-on-disabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_warning), "%s.%s.%s.stat-warning" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_remote), "%s.%s.%s.stat-remote" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_dems300_data_t, stat_at_speed), "%s.%s.%s.stat-at-speed" },
  { HAL_BIT, HAL_IN, offsetof(lcec_dems300_data_t, quick_stop), "%s.%s.%s.quick-stop" },
  { HAL_BIT, HAL_IN, offsetof(lcec_dems300_data_t, enable), "%s.%s.%s.enable" },
  { HAL_BIT, HAL_IN, offsetof(lcec_dems300_data_t, fault_reset), "%s.%s.%s.fault-reset" },
  { HAL_BIT, HAL_IN, offsetof(lcec_dems300_data_t, halt), "%s.%s.%s.halt" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_dems300_data_t, vel_rpm_cmd), "%s.%s.%s.vel-rpm-cmd" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_pindesc_t slave_params[] = {
  { HAL_BIT, HAL_RW, offsetof(lcec_dems300_data_t, auto_fault_reset), "%s.%s.%s.auto-fault-reset" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static ec_pdo_entry_info_t lcec_dems300_in[] = {
   {0x6041, 0x00, 16}, // Status Word
   {0x6043, 0x00, 16}, // Velocity Demand
   {0x6061, 0x00,  8}  // Mode of Operation Display
};

static ec_pdo_entry_info_t lcec_dems300_out[] = {
   {0x6040, 0x00, 16}, // Control Word
   {0x6042, 0x00, 16}, // Target Velocity
   {0x6060, 0x00, 8}   // Modes of Operation
};

static ec_pdo_info_t lcec_dems300_pdos_out[] = {
    {0x1600,  3, lcec_dems300_out}
};

static ec_pdo_info_t lcec_dems300_pdos_in[] = {
    {0x1a00, 3, lcec_dems300_in}
};

static ec_sync_info_t lcec_dems300_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT,  0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_dems300_pdos_out},
    {3, EC_DIR_INPUT,  1, lcec_dems300_pdos_in},
    {0xff}
};

void lcec_dems300_read(struct lcec_slave *slave, long period);
void lcec_dems300_write(struct lcec_slave *slave, long period);

int lcec_dems300_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_dems300_data_t *hal_data;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_dems300_read;
  slave->proc_write = lcec_dems300_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_dems300_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_dems300_data_t));
  slave->hal_data = hal_data;

  // set to velocity mode
  //if (ecrt_slave_config_sdo8(slave->config, 0x6060, 0x00, 2) != 0) {
  //  rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo velo mode\n", master->name, slave->name);
  //}

  // initialize sync info
  slave->sync_info = lcec_dems300_syncs;

  // initialize POD entries
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6041, 0x00, &hal_data->status_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6043, 0x00, &hal_data->currvel_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6061, 0x00, &hal_data->mode_op_display_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6040, 0x00, &hal_data->control_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6042, 0x00, &hal_data->cmdvel_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6060, 0x00, &hal_data->mode_op_pdo_os, NULL);

  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // initialize variables
  hal_data->enable_old = 0;
  hal_data->internal_fault = 0;

  hal_data->auto_fault_reset = 1;

  return 0;
}

void lcec_dems300_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_dems300_data_t *hal_data = (lcec_dems300_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  int8_t opmode_in;
  int32_t speed_raw;
  double rpm;

  // wait for slave to be operational
  if (!slave->state.operational) {
    *(hal_data->stat_switch_on_ready) = 0;
    *(hal_data->stat_switched_on)  = 0;
    *(hal_data->stat_op_enabled) = 0;
    *(hal_data->stat_fault)        = 1;
    *(hal_data->stat_volt_enabled) = 0;
    *(hal_data->stat_quick_stoped) = 0;
    *(hal_data->stat_switch_on_disabled)  = 0;
    *(hal_data->stat_warning)      = 0;
    *(hal_data->stat_remote)       = 0;
    *(hal_data->stat_at_speed)     = 0;
    return;
  }

  // read Modes of Operation
  opmode_in = EC_READ_S8(&pd[hal_data->mode_op_display_pdo_os]);

  // read status word
  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  *(hal_data->stat_switch_on_ready)    = (status >> 0) & 1;
  *(hal_data->stat_switched_on)       = (status >> 1) & 1;
  *(hal_data->stat_op_enabled)        = (status >> 2) & 1;
//  *(hal_data->stat_fault)             = (status >> 3) & 1;
  hal_data->internal_fault 			  = (status >> 3) & 0x01;
  *(hal_data->stat_volt_enabled)      = (status >> 4) & 1;
  *(hal_data->stat_quick_stoped)      = (status >> 5) & 1;
  *(hal_data->stat_switch_on_disabled) = (status >> 6) & 1;
  *(hal_data->stat_warning)           = (status >> 7) & 1;
  *(hal_data->stat_remote)            = (status >> 9) & 1;
  *(hal_data->stat_at_speed) 	      = (status >> 10) & 0x01;

 
  // set fault if op mode is wrong 
  if (opmode_in != 2) {
	  hal_data->internal_fault  = 1;
    rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "MS300 slave %s.%s not sending velo mode\n", master->name, slave->name);
  }

  // update fault output
  if (hal_data->auto_fault_reset_delay > 0) {
    hal_data->auto_fault_reset_delay -= period;
    *(hal_data->stat_fault) = 0;
  } else {
    *(hal_data->stat_fault) = hal_data->internal_fault && *(hal_data->enable);
  }

  // read current speed
  speed_raw = EC_READ_S16(&pd[hal_data->currvel_pdo_os]);
  rpm = (double)speed_raw;
  *(hal_data->vel_fb_rpm) = rpm;
  *(hal_data->vel_fb_rpm_abs) = fabs(rpm);


}

void lcec_dems300_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_dems300_data_t *hal_data = (lcec_dems300_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t control;
  double speed_raw;
  int8_t opmode;
	int enable_edge;

  //set drive OP mode
  opmode = OPMODE_VELOCITY; 	
  EC_WRITE_S8(&pd[hal_data->mode_op_pdo_os], opmode);

  // check for enable edge
  enable_edge = *(hal_data->enable) && !hal_data->enable_old;
  hal_data->enable_old = *(hal_data->enable);


  // write control register
  control = (!*(hal_data->fault_reset) << 2); // quick stop 

  if (*(hal_data->stat_fault)) {
    if (*(hal_data->fault_reset)) {
      control |= (1 << 7); // fault reset
    }
    if (hal_data->auto_fault_reset && enable_edge) {
      hal_data->auto_fault_reset_delay = MS300_FAULT_AUTORESET_DELAY_NS;
      control |= (1 << 7); // fault reset
    }
  } else {
    if (*(hal_data->enable)) {
      control |= (1 << 1); // enable voltage
      if (*(hal_data->stat_switch_on_ready)) {
        control |= (1 << 0); // switch on
        if (*(hal_data->stat_switched_on)) {
          control |= (1 << 3); // enable op
        }
      }
    }
    //set velo control bits 
	if (*(hal_data->stat_op_enabled)) {
      control |= (1 << 4); // rfg enable 
      control |= (1 << 5); // rfg unlock
      control |= (1 << 6); // rfg use ref 
	}
  }

  //halt 
  control |= (*(hal_data->halt) << 8); // halt  

  EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);


  // set RPM
  speed_raw = *(hal_data->vel_rpm_cmd);
  if (speed_raw > (double)0x7fffffff) {
    speed_raw = (double)0x7fffffff;
  }
  if (speed_raw < (double)-0x7fffffff) {
    speed_raw = (double)-0x7fffffff;
  }
  EC_WRITE_S16(&pd[hal_data->cmdvel_pdo_os], (int16_t)speed_raw);
}

