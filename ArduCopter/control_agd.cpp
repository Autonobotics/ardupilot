/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"
#include <string>

/*
* control_agd.pde - init and run calls for agd flight mode
*/

// agd_init - initialise agd controller
bool Copter::agd_init(bool ignore_checks)
{
	// set target altitude to zero for reporting
	// To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
	pos_control.set_alt_target(0);
	agd_mode = Agd_StartGuide;
	agd_prev_mode = Agd_StartUp;
	agd_pitch_con_prev = startup;
	agd_roll_con_prev = startup;
	agd_throttle_con_prev = startup;
	//agd_started = true;
	gcs_send_text_P(SEVERITY_LOW, PSTR("AGD INIT"));
	// stabilize should never be made to fail
	agd_nav_init();
	return true;
}

bool Copter::agd_check_input() {
	bool userInput = true;
	/*int16_t throttle_control = channel_throttle->control_in;
	int16_t mid_stick_throttle = channel_throttle->get_control_mid();
	int16_t pitch_control = channel_pitch->control_in;
	int16_t mid_stick_pitch = channel_pitch->get_control_mid();
	int16_t roll_control = channel_roll->control_in;
	int16_t mid_stick_roll = channel_roll->get_control_mid();
	int16_t yaw_control = channel_yaw->control_in;
	int16_t mid_stick_yaw = channel_yaw->get_control_mid();*/
	int16_t agd_input_mode = g.rc_6.control_in;
	//char rc6_value[8];


	//gcs_send_text_P(SEVERITY_LOW, PSTR(itoa(agd_input_mode,rc6_value,10)));

	//if channel 6 (agd_mode) returned value is above 1500 go into auto mode
	if (agd_input_mode > AGD_AUTO_MODE_THRESHOLD){
		//gcs_send_text_P(SEVERITY_LOW, PSTR("AGD_AUTO_MODE"));
		userInput = false;
	}
	// check rc inputs are in the deadband
	/*if ((throttle_control < (mid_stick_throttle + 15)) && (throttle_control > (mid_stick_throttle - 15)) \
		&& (pitch_control < (mid_stick_pitch + 15)) && (pitch_control > (mid_stick_pitch - 15)) \
		&& (roll_control < (mid_stick_roll + 15)) && (roll_control > (mid_stick_roll - 15)) \
		&& (yaw_control < (mid_stick_yaw + 15)) && (yaw_control > (mid_stick_yaw - 15))) {
		
		userInput = false;
	}*/
	return userInput;
}

// agd_run - runs the main agd controller
// should be called at 100hz or more
void Copter::agd_run()
{
	run_nav();
	// if there is a user input, override the navigation board's
	if (agd_check_input()) {
		agd_mode = Agd_UserInput;
	}
	else if (agd_get_nav_info()){
		agd_mode = Agd_NavInput;
	}
	else {
		agd_mode = Agd_SearchBeacon;
	}

	if (agd_mode == agd_prev_mode) {
		agd_change_mode = false;
	}
	else {
		agd_change_mode = true;
	}

	switch (agd_mode) {

	case Agd_StartUp: // just a placeholder for agd_prev_mode initialization agd_mode should not set to this value.
	case Agd_StartGuide:
		if (agd_change_mode) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("Change mode: AGD START"));
		}
		agd_start_guide_run();
		agd_prev_mode = Agd_StartGuide;
		break;

	case Agd_SearchBeacon:
		if (agd_change_mode) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("Change mode: SEARCH BEACON"));
		}
		agd_search_beacon_run();
		agd_prev_mode = Agd_SearchBeacon;
		break;

	case Agd_NavInput:
		if (agd_change_mode) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("Change mode: NAV INPUT"));
		}
		agd_nav_input_run();
		agd_prev_mode = Agd_NavInput;
		break;

	case Agd_UserInput:
		if (agd_change_mode) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("Change mode: USER INPUT"));
		}
		agd_user_input_run();
		agd_prev_mode = Agd_UserInput;
		break;
	}


}

bool Copter::agd_get_nav_info() {
	//get navigation info from STM board
	//pitch_con, roll_con, yaw_con, throttle_con
	//pitch = forward and backword
	//roll = left and right
	//yaw = rotation
	//throttle = up and down
	agd_pitch_con = y_inten;
	agd_roll_con = x_inten;
	agd_throttle_con = z_inten;
	agd_yaw_con = rotation_abs;

	if (agd_yaw_con == 0x7ffe) {
		return false;
	}

	return true;
}

void Copter::agd_calc_throttle(int16_t &throttle_val) {
	throttle_val = channel_throttle->get_control_mid();
	switch (agd_throttle_con) {
	case startup:
		break;
	case posHigh:
		throttle_val = AGD_THROTTLE_POS_HIGH;
		if (agd_throttle_con == agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_POS_HIGH"));
			agd_throttle_con_prev = agd_throttle_con;
		}
		break;
	case negHigh:
		throttle_val = AGD_THROTTLE_NEG_HIGH;
		if (agd_throttle_con == agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_NEG_HIGH"));
			agd_throttle_con_prev = agd_throttle_con;
		}
		break;
	case posLow:
		throttle_val = AGD_THROTTLE_POS_LOW;
		if (agd_throttle_con == agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_POS_LOW"));
			agd_throttle_con_prev = agd_throttle_con;
		}
		break;
	case negLow:
		throttle_val = AGD_THROTTLE_NEG_LOW;
		if (agd_throttle_con == agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_NEG_LOW"));
			agd_throttle_con_prev = agd_throttle_con;
		}
		break;
	case idle:
		throttle_val = channel_throttle->get_control_mid();
		if (agd_throttle_con == agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_idle"));
			agd_throttle_con_prev = agd_throttle_con;
		}
		break;
	}
}

// based on the desired direction and magnitude of roll and pitch from navigation board, set the target roll and pitch angle to predefined values.
void Copter::agd_calc_desired_lean_angles(float &roll_out, float &pitch_out)
{
	// from the controller, pitch and roll is betweent [1100, 1900]
	float angle_max = constrain_float(aparm.angle_max, 1000, 8000);
	float scaler = (float)angle_max / (float)ROLL_PITCH_INPUT_MAX;
	float roll_val = channel_roll->get_control_mid();
	float pitch_val = channel_pitch->get_control_mid();

	switch (agd_roll_con) {
	case startup:
		break;
	case posHigh:
		roll_val = AGD_ROLL_POS_HIGH;
		if (agd_roll_con_prev == agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_POS_HIGH"));
		}
		break;
	case negHigh:
		roll_val = AGD_ROLL_NEG_HIGH;
		if (agd_roll_con_prev == agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_NEG_HIGH"));
		}
		break;
	case posLow:
		roll_val = AGD_ROLL_POS_LOW;
		if (agd_roll_con_prev == agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_POS_LOW"));
		}
		break;
	case negLow:
		roll_val = AGD_ROLL_NEG_LOW;
		if (agd_roll_con_prev == agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_NEG_LOW"));
		}
		break;
	case idle:
		roll_val = channel_roll->get_control_mid();
		if (agd_roll_con_prev == agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_idle"));
		}
		break;
	}
	
	switch (agd_pitch_con) {
	case startup:
		break;
	case posHigh:
		pitch_val = AGD_PITCH_POS_HIGH;
		if (agd_pitch_con_prev == agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_POS_HIGH"));
		}
		break;
	case negHigh:
		pitch_val = AGD_PITCH_NEG_HIGH;
		if (agd_pitch_con_prev == agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_NEG_HIGH"));
		}
		break;
	case posLow:
		pitch_val = AGD_PITCH_POS_LOW;
		if (agd_pitch_con_prev == agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_POS_LOW"));
		}
		break;
	case negLow:
		pitch_val = AGD_PITCH_NEG_LOW;
		if (agd_pitch_con_prev == agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_NEG_LOW"));
		}
		break;
	case idle:
		pitch_val = channel_pitch->get_control_mid();
		if (agd_pitch_con_prev == agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_idle"));
		}
		break;
	}

	// scale roll_in, pitch_in to correct units
	roll_val *= scaler;
	pitch_val *= scaler;

	// do circular limit
	float total_in = pythagorous2((float)pitch_val, (float)roll_val);
	if (total_in > angle_max) {
		float ratio = angle_max / total_in;
		roll_val *= ratio;
		pitch_val *= ratio;
	}

	// do lateral tilt to euler roll conversion
	roll_val = (18000 / M_PI_F) * atanf(cosf(pitch_val*(M_PI_F / 18000))*tanf(roll_val*(M_PI_F / 18000)));

	// return
	roll_out = roll_val;
	pitch_out = pitch_val;

}


void Copter::agd_start_guide_run()
{
	float target_roll, target_pitch;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled = g.throttle_mid;
	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if (!motors.armed() || ap.throttle_zero) {
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		return;
	}

	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();

	get_pilot_desired_lean_angles(channel_roll->get_control_mid(), channel_pitch->get_control_mid(), target_roll, target_pitch);
	target_yaw_rate = 1300 * g.acro_yaw_p;

	// call attitude controller
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	// body-frame rate controller is run directly from 100hz loop

	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}


void Copter::agd_search_beacon_run()
{
	float target_roll, target_pitch;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled = g.throttle_mid;
	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if (!motors.armed() || ap.throttle_zero) {
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		return;
	}

	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();

	get_pilot_desired_lean_angles(channel_roll->get_control_mid(), channel_pitch->get_control_mid(), target_roll, target_pitch);
	target_yaw_rate = 1300 * g.acro_yaw_p;

	// call attitude controller
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	// body-frame rate controller is run directly from 100hz loop

	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

void Copter::agd_nav_input_run()
{
	float target_roll, target_pitch;
	int16_t throttle_control;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled;
	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if (!motors.armed()) {
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		return;
	}
	
	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();

	agd_calc_desired_lean_angles(target_roll, target_pitch);
	target_yaw_rate = agd_yaw_con;
	agd_calc_throttle(throttle_control);
	pilot_throttle_scaled = get_pilot_desired_throttle(throttle_control);

	// call attitude controller
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	agd_prev_roll = target_roll;
	agd_prev_pitch = target_pitch;
	agd_prev_yaw_rate = target_yaw_rate;
	agd_prev_throttle_scaled = pilot_throttle_scaled;

	// body-frame rate controller is run directly from 100hz loop
	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

void Copter::agd_user_input_run()
{
	float target_roll, target_pitch;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled;
	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if (!motors.armed() || ap.throttle_zero) {
		attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
		return;
	}

	// apply SIMPLE mode transform to pilot inputs
	update_simple_mode();
	
	// convert pilot input to lean angles
	// To-Do: convert get_pilot_desired_lean_angles to return angles as floats
	get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);
	
	// get pilot's desired yaw rate
	target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

	// get pilot's desired throttle
	pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);
	
	// call attitude controller
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	// body-frame rate controller is run directly from 100hz loop
	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

