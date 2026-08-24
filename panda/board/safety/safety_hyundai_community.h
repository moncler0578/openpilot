int LKAS11_op = 0;
int MDPS12_op = 0;
int CLU11_op = 0;
int SCC12_op = 0;
int SCC12_car = 0;
int EMS11_op = 0;
int MDPS_bus = -1;
int SCC_bus = -1;

// Raw physical SCC MAIN state, plus a debounced lateral authorization latch.
// Lateral authority must NOT follow controls_allowed: generic_rx_checks drops
// controls_allowed on every moving brake sample, and some platforms blink the
// stock SCC11 MAIN bit during a brake/override transition. Either one would
// make the LKAS11 tx gate flap, which in turn lets the stock camera LKAS11
// interleave with openpilot's on the MDPS bus and latches an MDPS fault.
bool hyundai_community_main_on = false;
bool hyundai_community_lat_allowed = false;
int hyundai_community_main_off_cnt = 0;
// SCC11/EMS16 arrive at 50 Hz, so this is ~0.5 s of sustained MAIN off.
#define HYUNDAI_COMMUNITY_MAIN_OFF_FRAMES 25

// Keep panda's longitudinal envelope aligned with CarControllerParams.
// SCC12 encodes acceleration in 1/100 m/s^2.
const int HYUNDAI_COMMUNITY_MAX_ACCEL = 250;
const int HYUNDAI_COMMUNITY_MIN_ACCEL = -400;

const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {593, 2, 8},                              // MDPS12, Bus 2
//  {897, 0, 8},                              // MDPS11, Bus 0
  {790, 1, 8},                              // EMS11, Bus 1
  {832, 0, 8}, {832, 1, 8},                 // LKAS11, Bus 0, 1
  {1056, 0, 8},                             // SCC11, Bus 0
  {1057, 0, 8},                             // SCC12, Bus 0
  {1290, 0, 8},                             // SCC13, Bus 0
  {905, 0, 8},                              // SCC14, Bus 0
  {909, 0, 8},                              // FCA11 Bus 0
  {1155, 0, 8},                             // FCA12 Bus 0
  {1157, 0, 4},                             // LFAHDA_MFC, Bus 0
  {1186, 0, 8},                             // FRT_RADAR11, Bus 0
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4}, // CLU11, Bus 0, 1, 2
//  {912, 0, 7}, {912,1, 7},                  // SPAS11, Bus 0, 1
//  {1268, 0, 8}, {1268,1, 8},                // SPAS12, Bus 0, 1
//  {2000, 0, 8},                             // radar UDS TX addr Bus 0 (for radar disable)
 };

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_community_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},     // EMS16
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},                                       // E_EMS11
  {.msg = {{902, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},                                // WHL_SPD11
  {.msg = {{916, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},                                // TCS13
//  {.msg = {{1057, 0, 8, .check_checksum = true, .max_counter = 15U, .expected_timestep = 20000U}, { 0 }, { 0 }}},  // SCC12
};

#define HYUNDAI_COMMUNITY_ADDR_CHECK_LEN (sizeof(hyundai_community_addr_checks) / sizeof(hyundai_community_addr_checks[0]))

addr_checks hyundai_community_rx_checks = {hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};

// Track the physical MAIN switch. controls_allowed keeps the standard pedal
// interlock (longitudinal), while hyundai_community_lat_allowed only drops
// after MAIN has been off continuously, so a transient bit drop during a
// brake/override transition cannot revoke steering.
static void hyundai_community_update_main(int cruise_engaged) {
  hyundai_community_main_on = cruise_engaged != 0;

  if (cruise_engaged) {
    hyundai_community_main_off_cnt = 0;
    if (!hyundai_community_lat_allowed) {
      hyundai_community_lat_allowed = true;
      puts("  SCC main on: lateral allowed\n");
    }
  } else {
    if (hyundai_community_main_off_cnt < HYUNDAI_COMMUNITY_MAIN_OFF_FRAMES) {
      hyundai_community_main_off_cnt++;
    } else if (hyundai_community_lat_allowed) {
      hyundai_community_lat_allowed = false;
      puts("  SCC main off: lateral not allowed\n");
    } else {
    }
  }
}

static int hyundai_community_rx_hook(CANPacket_t *to_push) {
  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);
  bool valid = addr_safety_check(to_push, &hyundai_community_rx_checks, hyundai_get_checksum,
                                 hyundai_compute_checksum, hyundai_get_counter);
  if (!valid){
    puts("  CAN RX invalid: "); puth(addr); puts("\n");
  }
  if (bus == 1 && Lcan_bus1) {
    valid = false;
  }

  // check if we have a LCAN on Bus1
  if (bus == 1 && (addr == 1296 || addr == 524)) {
    Lcan_bus1_cnt = 500;
    if (Fwd_bus1 || !Lcan_bus1) {
      Lcan_bus1 = true;
      Fwd_bus1 = false;
      puts("  LCAN on bus1: forwarding disabled\n");
    }
  }

  // check if LKAS11 on Bus0
  if (addr == 832) {
    if (bus == 0 && Fwd_bus2) {
      Fwd_bus2 = false;
      LKAS11_bus0_cnt = 20;
      puts("  LKAS11 on bus0: forwarding disabled\n");
    }
    if (bus == 2) {
      if (LKAS11_bus0_cnt > 0) {
        LKAS11_bus0_cnt--;
        } else if (!Fwd_bus2) {
          Fwd_bus2 = true;
          puts("  LKAS11 on bus2: forwarding enabled\n");
      }
      if (Lcan_bus1_cnt > 0) {
        Lcan_bus1_cnt--;
        } else if (Lcan_bus1) {
          Lcan_bus1 = false;
          puts("  Lcan not on bus1\n");
      }
    }
  }

  // check MDPS12 or MDPS11 on Bus
  if ((addr == 593 || addr == 897) && MDPS_bus != bus) {
    if (bus != 1 || (!Lcan_bus1 || Fwd_obd)) {
      MDPS_bus = bus;
      if (bus == 1 && !Fwd_obd) {
        puts("  MDPS on bus1\n");
        if (!Fwd_bus1 && !Lcan_bus1) {
          Fwd_bus1 = true;
          puts("  bus1 forwarding enabled\n");
          }
        } else if (bus == 1) {
          puts("  MDPS on obd bus\n");
      }
    }
  }

  // check SCC11 or SCC12 on Bus
  if ((addr == 1056 || addr == 1057) && SCC_bus != bus) {
    if (bus != 1 || !Lcan_bus1) {
      SCC_bus = bus;
      if (bus == 1) {
        puts("  SCC on bus1\n");
        if (!Fwd_bus1) {
          Fwd_bus1 = true;
          puts("  bus1 forwarding enabled\n");
        }
      }
      if (bus == 2) {
        puts("  SCC bus = bus2\n");
      }
    }
  }

  // MDPS12
  if (valid) {
    if (addr == 593 && bus == MDPS_bus) {
      int torque_driver_new = ((GET_BYTES_04(to_push) & 0x7ff) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // The physical SCC MAIN state is authoritative. Never gate this on
    // SCC12_op: openpilot replaces SCC12 from boot when longitudinal control
    // is configured, so an SCC12_op gate freezes this handler forever and
    // controls_allowed can never be set.
    if (addr == 1056) {
      // 2 bits: 13-14
      int cruise_engaged = GET_BYTES_04(to_push) & 0x1; // ACC main_on signal
      hyundai_community_update_main(cruise_engaged);
      if (cruise_engaged && !controls_allowed && !brake_pressed) {
        controls_allowed = 1;
        puts("  SCC main on: controls allowed\n");
      }
      // Longitudinal authority follows the physical MAIN bit with no delay.
      // Only the lateral latch (hyundai_community_lat_allowed) is debounced,
      // so a transient bit drop cannot revoke steering while MAIN off still
      // stops acceleration requests immediately.
      if (!cruise_engaged) {
        if (controls_allowed) {
          puts("  SCC main off: controls not allowed\n");}
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // cruise control for car without SCC ( EMS16 )
    if (addr == 608 && bus == 0 && SCC_bus == -1) {
      // bit 25
      int cruise_engaged = (GET_BYTES_04(to_push) >> 25 & 0x1); // ACC main_on signal
      hyundai_community_update_main(cruise_engaged);
      if (cruise_engaged && !controls_allowed && !brake_pressed) {
        controls_allowed = 1;
        puts("  non-SCC w/ long control: controls allowed\n");
      }
      if (!cruise_engaged) {
        if (controls_allowed) {
          puts("  non-SCC w/ long control: controls not allowed\n");}
        controls_allowed = 0;
      }
      cruise_engaged_prev = cruise_engaged;
    }

    // sample wheel speed, averaging opposite corners ( WHL_SPD11 )
    if (addr == 902) {
      int hyundai_speed = (GET_BYTES_04(to_push) & 0x3FFFU) + ((GET_BYTES_48(to_push) >> 16) & 0x3FFFU);  // FL + RR
      hyundai_speed /= 2;
      vehicle_moving = hyundai_speed > HYUNDAI_STANDSTILL_THRSLD;
    }

    // Keep Panda's pedal interlock independent from controlsd. EMS16 carries
    // the ICE accelerator and E_EMS11 carries the EV/HEV accelerator. Accept
    // either E_EMS11 layout since community safety spans both powertrains.
    if (addr == 608) {
      gas_pressed = (GET_BYTE(to_push, 7) >> 6) != 0U;
    } else if (addr == 881) {
      bool ev_gas = ((((GET_BYTE(to_push, 4) & 0x7FU) << 1) |
                      (GET_BYTE(to_push, 3) >> 7)) != 0U);
      bool hev_gas = GET_BYTE(to_push, 7) != 0U;
      gas_pressed = ev_gas || hev_gas;
    } else {
    }

    // Use only the physical driver-brake bit, matching carstate DriverBraking.
    // generic_rx_checks applies the standard pedal interlock to
    // controls_allowed (longitudinal). Lateral authority is tracked separately
    // by hyundai_community_lat_allowed and is intentionally not touched here.
    if (addr == 916) {
      brake_pressed = (GET_BYTE(to_push, 6) >> 7) != 0U;
    }
    generic_rx_checks((addr == 832 && bus == 0)); // LKAS11
  }
  return valid;
}

static int hyundai_community_tx_hook(CANPacket_t *to_send, bool longitudinal_allowed) {
  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (!msg_allowed(to_send, HYUNDAI_COMMUNITY_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_TX_MSGS[0]))) {
    tx = 0;
    puts("  CAN TX not allowed: "); puth(addr); puts(", "); puth(bus); puts("\n");
  }

  if (relay_malfunction) {
    tx = 0;
    puts("  CAN TX not allowed LKAS on bus0\n");
  }

  // FCA11: never allow openpilot to request AEB actuation.
  if (addr == 909) {
    int aeb_decel_cmd = GET_BYTE(to_send, 1);
    int fca_cmd_act = (GET_BYTE(to_send, 2) >> 5) & 1U;
    int aeb_decel_cmd_act = (GET_BYTE(to_send, 3) >> 7) & 1U;

    if ((aeb_decel_cmd != 0) || (fca_cmd_act != 0) || (aeb_decel_cmd_act != 0)) {
      tx = 0;
    }
  }

  // SCC12: enforce the same -4.0 to +2.5 m/s^2 range used by controls.
  // When longitudinal actuation is not allowed, both requests must be zero.
  // carcontroller must already zero the request while the driver brake is
  // applied, otherwise this rejection hands the SCC12 stream back to the
  // stock ECU for a frame and the cluster reports it as a fault.
  if (addr == 1057) {
    int desired_accel_raw = (((GET_BYTE(to_send, 4) & 0x7U) << 8) | GET_BYTE(to_send, 3)) - 1023;
    int desired_accel_val = ((GET_BYTE(to_send, 5) << 3) | (GET_BYTE(to_send, 4) >> 5)) - 1023;
    int aeb_decel_cmd = GET_BYTE(to_send, 2);
    int aeb_req = (GET_BYTE(to_send, 6) >> 6) & 1U;

    bool violation = false;
    if (!longitudinal_allowed || brake_pressed_prev) {
      violation |= (desired_accel_raw != 0) || (desired_accel_val != 0);
    }
    violation |= max_limit_check(desired_accel_raw, HYUNDAI_COMMUNITY_MAX_ACCEL, HYUNDAI_COMMUNITY_MIN_ACCEL);
    violation |= max_limit_check(desired_accel_val, HYUNDAI_COMMUNITY_MAX_ACCEL, HYUNDAI_COMMUNITY_MIN_ACCEL);
    violation |= (aeb_decel_cmd != 0) || (aeb_req != 0);

    if (violation) {
      tx = 0;
    }
  }

  // LKA STEER: gated on the debounced physical MAIN latch, not on
  // controls_allowed. Longitudinal actuation stays independently blocked by
  // controls_allowed/brake_pressed_prev in the SCC12 check above.
  if (addr == 832) {
    int desired_torque = ((GET_BYTES_04(to_send) >> 16) & 0x7ff) - 1024;
    uint32_t ts = microsecond_timer_get();
    bool violation = 0;
    bool lateral_allowed = hyundai_community_lat_allowed;

    if (lateral_allowed) {
      // *** global torque limit check ***
      bool torque_check = 0;
      violation |= torque_check = max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);
      if (torque_check) {
        puts("  LKAS TX not allowed: torque limit check failed!\n");}

      // *** torque rate limit check ***
      bool torque_rate_check = 0;
      violation |= torque_rate_check = driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);
      if (torque_rate_check) {
        puts("  LKAS TX not allowed: torque rate limit check failed!\n");}

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      bool torque_rt_check = 0;
      violation |= torque_rt_check = rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);
      if (torque_rt_check) {
        puts("  LKAS TX not allowed: torque real time rate limit check failed!\n");}

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if lateral is not allowed
    if (!lateral_allowed && (desired_torque != 0)) {
      violation = 1;
      puts("  LKAS torque not allowed: lateral not allowed!\n");
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !lateral_allowed) {
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 1265 && !controls_allowed && (bus != MDPS_bus && MDPS_bus == 1)) {
    if ((GET_BYTES_04(to_send) & 0x7) != 4) {
      tx = 0;
    }
  }

  // Keep suppressing the stock counterpart even when a frame is rejected.
  // Making these conditional on tx lets the stock camera LKAS11 (or the stock
  // SCC12) interleave with openpilot's stream for a single frame, which the
  // MDPS sees as an AliveCounter/checksum break and latches as a fault.
  if (addr == 832) { LKAS11_op = 20; }
  if (addr == 593) { MDPS12_op = 20; }
  if (addr == 1265 && bus == 1) { CLU11_op = 20; } // only count messages created for MDPS
  if (addr == 1057) {
    SCC12_op = 20;
    if (SCC12_car > 0) { SCC12_car -= 1; }
  }
  if (addr == 790) { EMS11_op = 20; }

  // 1 allows the message through
  return tx;
}

static int hyundai_community_fwd_hook(int bus_num, CANPacket_t *to_fwd) {
  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  int fwd_to_bus1 = -1;
  if (Fwd_bus1 || Fwd_obd){fwd_to_bus1 = 1;}

  // forward cam to ccan and viceversa, except lkas cmd
  if (Fwd_bus2) {
    if (bus_num == 0) {
      if (!CLU11_op || addr != 1265 || MDPS_bus == 0) {
        if (!MDPS12_op || addr != 593) {
          if (!EMS11_op || addr != 790) {
            bus_fwd = fwd_to_bus1 == 1 ? 12 : 2;
          } else {
            bus_fwd = 2;  // OP create EMS11 for MDPS
            EMS11_op -= 1;
          }
        } else {
          bus_fwd = fwd_to_bus1;  // OP create MDPS for LKAS
          MDPS12_op -= 1;
        }
      } else {
        bus_fwd = 2; // OP create CLU12 for MDPS
        CLU11_op -= 1;
      }
    }
    if (bus_num == 1 && (Fwd_bus1 || Fwd_obd)) {
      if (!MDPS12_op || addr != 593) {
        if (!SCC12_op || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 20;
        } else {
          bus_fwd = 2;  // OP create SCC11 SCC12 SCC13 SCC14 for Car
          SCC12_op -= 1;
        }
      } else {
        bus_fwd = 0;  // OP create MDPS for LKAS
        MDPS12_op -= 1;
      }
    }
    if (bus_num == 2) {
      if (!LKAS11_op || (addr != 832 && addr != 1157)) {
        if (!SCC12_op || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = fwd_to_bus1 == 1 ? 10 : 0;
        } else {
          bus_fwd = fwd_to_bus1;  // OP create SCC12 for Car
          SCC12_op -= 1;
        }
      } else if (MDPS_bus == 0) {
        bus_fwd = fwd_to_bus1; // OP create LKAS and LFA for Car
        LKAS11_op -= 1;
      } else {
        LKAS11_op -= 1; // OP create LKAS and LFA for Car and MDPS
      }
    }
  } else {
    if (bus_num == 0) {
      bus_fwd = fwd_to_bus1;
    }
    if (bus_num == 1 && (Fwd_bus1 || Fwd_obd)) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static const addr_checks* hyundai_community_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  hyundai_community_main_on = false;
  hyundai_community_lat_allowed = false;
  hyundai_community_main_off_cnt = 0;
  relay_malfunction_reset();

  if (current_board->has_obd && Fwd_obd) {
    current_board->set_can_mode(CAN_MODE_OBD_CAN2);
    puts("  MDPS or SCC on OBD2 CAN: setting can mode obd\n");
  }

  hyundai_community_rx_checks = (addr_checks){hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};
  return &hyundai_community_rx_checks;
}

const safety_hooks hyundai_community_hooks = {
  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
};
