// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      accel rising edge
//      brake rising edge
//      brake > 0mph

const int FORD_MAX_STEER = 2048;//find rt torque limit 
const int FORD_MAX_RT_DELTA = 940;//find max delta torque rt checks
const uint32_t FORD_RT_INTERVAL = 250000; //250ms between rt checks
const int FORD_MAX_RATE_UP = 10;//find rate up
const int FORD_MAX_RATE_DOWN = 25;//find rate down
//const int FORD_MAX_TORQUE_ERORR = 15; find torque error remark line to test. 
const int FORD_DRIVER_TORQUE_ALLOWANCE = 15;//find torque allowance
const int FORD_DRIVER_TORQUE_FACTOR = 1;//find torque factor
const AddrBus FORD_TX_MSGS[] = {{0x83, 0}, {0x3CA, 2}, {0x3D8, 2}};

AddrCheckStruct ford_rx_checks[] = {
	{.addr = {0x83},.bus = 0,.expected_timestep = 10000U}, //find timestep
	{.addr = {0x3CA},.bus = 2,.expected_timestep = 10000U}, //find timestep
	{.addr = {0x3D8},.bus = 2,.expected_timestep = 10000U}}, //find timestep
};
const int FORD_RX_CHECK_LEN = sizeof(ford_rx_checks) / sizeof(ford_rx_checks[0]);

int ford_brake_prev = 0;
int ford_gas_prev = 0;
bool ford_moving = false;
int ford_rt_torque_last = 0;
int ford_desired_torque_last = 0;
int ford_cruise_engaged_last = 0;
uint32_t ford_ts_last = 0;
struct sample_t ford_torque_meas;

//TODO: compute and get checksum static uints

static int ford_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
	bool valid = addr_safety_check(to_push, ford_rx_checks, FORD_RX_CHECK_LEN,
									NULL, NULL, NULL);
									
	if (valid) {
		int addr = GET_ADDR(to_push);
		int bus = GET_BUS(to_push);
	
		if (addr == 0x217) {
			// wheel speeds are 14 bits every 16
			ford_moving = false;
			for (int i = 0; i < 8; i += 2) {
				ford_moving |= GET_BYTE(to_push, i) | (GET_BYTE(to_push, (int)(i + 1)) & 0xFCU);
			}
		}
	}

	// state machine to enter and exit controls
	if (addr == 0x83) {
		bool cancel = GET_BYTE(to_push, 1) & 0x1;
		bool set_or_resume = GET_BYTE(to_push, 3) & 0x30; //0x1A
		if (cancel) {
			controls_allowed = 0;
		}
		if (set_or_resume) {
			controls_allowed = 1;
		}
	}

	// exit controls on rising edge of brake press or on brake press when
	// speed > 0
	if (addr == 0x165) {
		int brake = GET_BYTE(to_push, 0) & 0x20;
		if (brake && (!(ford_brake_prev) || ford_moving)) {
			controls_allowed = 0;
		}
		ford_brake_prev = brake;
	}

	// exit controls on rising edge of gas press
	if (addr == 0x204) {
		int gas = (GET_BYTE(to_push, 0) & 0x03) | GET_BYTE(to_push, 1);
		if (gas && !(ford_gas_prev)) {
			controls_allowed = 0;
		}
		ford_gas_prev = gas;
	}
	// bus check for messages where they shouldnt be. 
	if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && (bus == 0) && (addr == 0x3CA) || (addr == 0x3D8)) {
		relay_malfunction = true;
	}
	return valid;
}

// all commands: just steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int ford_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

	int tx = 1;
	int addr = GET_ADDR(to_send);

	// disallow actuator commands if gas or brake (with vehicle moving) are pressed
	// and the the latching controls_allowed flag is True
	int pedal_pressed = ford_gas_prev || (ford_brake_prev && ford_moving);
	bool current_controls_allowed = controls_allowed && !(pedal_pressed);

	if (!msg_allowed(addr, bus, FORD_TX_MSGS, sizeof(FORD_TX_MSGS) / sizeof(FORD_TX_MSGS[0])) {
		tx = 0;
	}
	
	if (relay_malfunction) {
		tx = 0;
	}

	// STEER: safety check
	if ((addr == 0x3CA) || (addr == 0x3D8)) {
		int desired_torque = ((GET_BYTE(to_send, 0) & 0xF0) << 8) + GET_BYTE(to_send, 1);
		uint32_t ts = TIM2->CNT;
		bool violation = 0;
		desired_torque = to_signed(desired_torque_new, 11)

		if (current_controls_allowed){
			
			//Global torque limit check
			violation |= max_limit_check(desired torque, FORD_MAX_STEER, -FORD_MAX_STEER);
			
			//torque rate check
			violation |= driver_limit_check(desired_torque, ford_desired_torque_last, &ford_torque_driver, 
			FORD_MAX_STEER, FORD_MAX_RATE_UP, FORD_MAX_RATE_DOWN, 
			FORD_DRIVER_TORQUE_ALLOWANCE, FORD_DRIVER_TORQUE_FACTOR);
			
			//compare last desired torque to desired torque
			ford_desired_torque_last = desired_torque;
			
			//torque rt rate check
			violation |= rt_rate_limit_check(desired_torque, ford_rt_torque_last. FORD_MAX_RT_DELTA);
			
			//rt interval resets limits
			uint32_t ts_elapsed = get_ts_elapsed(ts, ford_ts_last);
			if (ts_elapsed > FORD_RT_INTERVAL) {
				ford_rt_torque_last = desired_torque;
				ford_ts_last = ts;
			}
		}
		
		//torque not allowed if controls not allowed
		if (!controls_allowed && (desired_torque != 0)) {
			violation = 1;
		}
		
		//reset if controls not allowed or violation
		if (violation || !controls_allowed) {
			ford_desired_torque_last = 0;
			ford_rt_torque_last = 0;
			ford_ts_last = ts;
		}
		
		if (violation) {
			tx = 0;
		}
	}
	return tx;
}
	// FORCE CANCEL: safety check only relevant when spamming the cancel button
	// ensuring that set and resume aren't sent
	//if (addr == 0x83) {
	//	if ((GET_BYTE(to_send, 3) & 0x1A) != 0) {
	//		tx = 0;
	//	}
	//}

	// 1 allows the message through


static int ford_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

	int bus_fwd = -1;

	int(!relay_malfunction) {
		// forward CAN 0 -> 2 so stock LKAS camera sees messages
		if (bus_num == 0) {
			bus_fwd = 2;
		}
		// forward all messages from camera except Lane_Keep_Assist_Control and Lane_Keep_Assist_Ui
		if (bus_num == 2) {
			int addr = GET_ADDR(to_fwd);
			int block_msg = (addr == 0x3CA) || (addr == 0x3D8);
			if (!block_msg) {
				bus_fwd = 0;
			}
		}
	}
	return bus_fwd;
}

const safety_hooks ford_hooks = {
  .init = nooutput_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = ford_fwd_hook,
  .addr_check = ford_rx_checks,
  .addr_check_len = sizeof(ford_rx_checks) / sizeof(ford_rx_checks[0]),
};