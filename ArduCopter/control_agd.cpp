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
	pos_control.set_alt_target(0);
	agd_mode = Agd_StartGuide;
	agd_prev_mode = Agd_StartUp;
	agd_pitch_con_prev = startup;
	agd_roll_con_prev = startup;
	agd_throttle_con_prev = startup;
    agd_prev_throttle = 0;
    agd_prev_roll = 0;
    agd_prev_pitch = 0;
    agd_prev_yaw_rate = 0;
	//agd_started = true;
	gcs_send_text_P(SEVERITY_LOW, PSTR("AGD INIT"));
	// stabilize should never be made to fail
	agd_nav_init();

    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // stop takeoff if running
    takeoff_stop();

	return true;
}

bool Copter::agd_check_input() {
	bool userInput = true;
	int16_t agd_input_mode = g.rc_6.control_in;

    //if channel 6 (agd_mode) returned value is above 1500 go into auto mode
	if (agd_input_mode > AGD_AUTO_MODE_THRESHOLD){
		userInput = false;
	}

    return userInput;
}

// agd_run - runs the main agd controller
// should be called at 100hz or more
void Copter::agd_run()
{
    //const Vector3f& velocity = inertial_nav.get_velocity();
    if (!motors.armed()) {
        attitude_control.set_throttle_out_unstabilized(0, true, g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in) - throttle_average);
        return;
    }
    agd_pixarm_counter++;
    if ((agd_pixarm_counter % 2 == 0) || (agd_nav_state == sync) || (agd_nav_state == ack)){
        run_nav();
    }
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
        if (agd_mode != Agd_SearchBeacon) {
            gcs_send_text_P(SEVERITY_LOW, PSTR("STM sent no beacon"));
        }
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
        throttle_val = agd_prev_throttle + AGD_THROTTLE_POS_HIGH;
		//if (agd_throttle_con != agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_POS_HIGH"));
			agd_throttle_con_prev = agd_throttle_con;
		//}
		break;
	case negHigh:
        throttle_val = agd_prev_throttle + AGD_THROTTLE_NEG_HIGH;
		//if (agd_throttle_con != agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_NEG_HIGH"));
			agd_throttle_con_prev = agd_throttle_con;
		//}
		break;
	case posLow:
        throttle_val = agd_prev_throttle + AGD_THROTTLE_POS_LOW;
		//if (agd_throttle_con != agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_POS_LOW"));
			agd_throttle_con_prev = agd_throttle_con;
		//}
		break;
	case negLow:
        throttle_val = agd_prev_throttle + AGD_THROTTLE_NEG_LOW;
		//if (agd_throttle_con != agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_NEG_LOW"));
			agd_throttle_con_prev = agd_throttle_con;
		//}
		break;
	case idle:
        throttle_val = agd_prev_throttle;
		//if (agd_throttle_con != agd_throttle_con_prev) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("TH_idle"));
			agd_throttle_con_prev = agd_throttle_con;
		//}
		break;
	}
    if (throttle_val > (1.8*(channel_throttle->get_control_mid()))) {
        throttle_val = 1.8*(channel_throttle->get_control_mid());
    }
    if (throttle_val < (0.4*(channel_throttle->get_control_mid()))) {
        throttle_val = 0.4*(channel_throttle->get_control_mid());
    }
    char temp[8];
    itoa(throttle_val, temp, 10);
    gcs_send_text_P(SEVERITY_LOW, PSTR(temp));
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
        roll_val = channel_roll->get_control_mid() + AGD_ROLL_POS_HIGH;
		if (agd_roll_con_prev != agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_POS_HIGH"));
		}
		break;
	case negHigh:
        roll_val = channel_roll->get_control_mid() + AGD_ROLL_NEG_HIGH;
		if (agd_roll_con_prev != agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_NEG_HIGH"));
		}
		break;
	case posLow:
        roll_val = channel_roll->get_control_mid() + AGD_ROLL_POS_LOW;
		if (agd_roll_con_prev != agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_POS_LOW"));
		}
		break;
	case negLow:
        roll_val = channel_roll->get_control_mid() + AGD_ROLL_NEG_LOW;
		if (agd_roll_con_prev != agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_NEG_LOW"));
		}
		break;
	case idle:
		roll_val = channel_roll->get_control_mid();
		if (agd_roll_con_prev != agd_roll_con) {
			agd_roll_con_prev = agd_roll_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("RL_idle"));
		}
		break;
	}
	
	switch (agd_pitch_con) {
	case startup:
		break;
	case posHigh:
        pitch_val = channel_pitch->get_control_mid() + AGD_PITCH_POS_HIGH;
		if (agd_pitch_con_prev != agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_POS_HIGH"));
		}
		break;
	case negHigh:
        pitch_val = channel_pitch->get_control_mid() + AGD_PITCH_NEG_HIGH;
		if (agd_pitch_con_prev != agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_NEG_HIGH"));
		}
		break;
	case posLow:
        pitch_val = channel_pitch->get_control_mid() + AGD_PITCH_POS_LOW;
		if (agd_pitch_con_prev != agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_POS_LOW"));
		}
		break;
	case negLow:
        pitch_val = channel_pitch->get_control_mid() + AGD_PITCH_NEG_LOW;
		if (agd_pitch_con_prev != agd_pitch_con) {
			agd_pitch_con_prev = agd_pitch_con;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PT_NEG_LOW"));
		}
		break;
	case idle:
		pitch_val = channel_pitch->get_control_mid();
		if (agd_pitch_con_prev != agd_pitch_con) {
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
    target_yaw_rate = ((channel_yaw->get_control_mid())+20) * g.acro_yaw_p;

	// call attitude controller
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	// body-frame rate controller is run directly from 100hz loop

	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

void Copter::agd_nav_input_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;
    int16_t throttle_control;

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    agd_calc_desired_lean_angles(target_roll, target_pitch);
  
    // get pilot's desired yaw rate
    float target_yaw_rate = agd_yaw_con;

    // get pilot desired climb rate
    agd_calc_throttle(throttle_control);
    float target_climb_rate = get_pilot_desired_climb_rate(throttle_control);
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    bool takeoff_triggered = (ap.land_complete && (throttle_control > get_takeoff_trigger_throttle()));

    // Alt Hold State Machine Determination
    if (!ap.auto_armed || !motors.get_interlock()) {
        althold_state = AltHold_Disarmed;
    }
    else if (takeoff_state.running || takeoff_triggered){
        althold_state = AltHold_Takeoff;
    }
    else if (ap.land_complete){
        althold_state = AltHold_Landed;
    }
    else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_Disarmed:
        break;

    case AltHold_Takeoff:

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
        break;

    case AltHold_Landed:
        break;

    case AltHold_Flying:
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call throttle controller
        if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }

        // call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
        pos_control.update_z_controller();
        break;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*float target_roll, target_pitch;
	int16_t throttle_control;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled;

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
    agd_prev_throttle = throttle_control;

	// body-frame rate controller is run directly from 100hz loop
	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);*/
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
	
    //char temp[8];
	// get pilot's desired yaw rate
	target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
    //itoa(ahrs.yaw_sensor, temp, 10);
    //gcs_send_text_P(SEVERITY_LOW, PSTR(temp));

	// get pilot's desired throttle
	pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);
	
	// call attitude controller
	attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

	// body-frame rate controller is run directly from 100hz loop
	// output pilot's throttle
	attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    agd_prev_roll = channel_roll->control_in;
    agd_prev_pitch = channel_pitch->control_in;
    agd_prev_yaw_rate = target_yaw_rate;
    agd_prev_throttle = channel_yaw->control_in;
}

