/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

void Copter::agd_nav_init()
{
	// put your initialisation code here
	// this will be called once at start-up
	AGD_NAV_PORT->begin(115200, 8, 8); // 57600 baud rate, 8 byte input, 8 byte output buffer
	AGD_NAV_PORT->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
	AGD_NAV_PORT->set_blocking_writes(true);
	agd_nav_state = sync;
	//agd_started = false;
	x_inten = idle;
	y_inten = idle;
	z_inten = idle;
	rotation_abs = 0x7ffe;
	if (AGD_NAV_PORT->is_initialized()) {
		gcs_send_text_P(SEVERITY_LOW, PSTR("UART init success"));
	}

	int16_t incoming[PIXARM_MSG_SIZE];
	int16_t size;
	char temp[PIXARM_MSG_SIZE];

	size = AGD_NAV_PORT->available();   // Number of bytes available in rx buffer
	for (int16_t i = 0; i < size; ++i) {
		incoming[i] = AGD_NAV_PORT->read();
	}
	for (int16_t i = 0; i < size; ++i) {
		itoa(incoming[i], temp, 16);
		gcs_send_text_P(SEVERITY_LOW, PSTR(temp));
	}


}

// Write ARM-Pix transfer protocol messages
/*void Copter::Log_Write_ARMPixT() {
	struct log_ARMPixT pkt = {
		LOG_PACKET_HEADER_INIT(LOG_ARMPixT_MSG),
        xVal    : x_inten,
        yVal    : y_inten,
        zVal    : z_inten,
        rVal    : rotation_abs
	};
	DataFlash.WriteBlock(&pkt, sizeof(pkt));
}*/

/*void Copter::Log_Write_ARMPixT_error(int error) {
	struct log_ARMPixT pkt = {
		LOG_PACKET_HEADER_INIT(LOG_ARMPixT_MSG),
		xVal    : x_inten,
		yVal	: y_inten,
		zVal	: z_inten,
		rVal	: rotation_abs
	};
	DataFlash.WriteBlock(&pkt, sizeof(pkt));
}*/

void Copter::nav_sync() {
	sAPP_PIXARM_SYNC sync_msg;
	sync_msg.cmd = PIXARM_CMD_SYNC;
	sync_msg.flag = PIXARM_FLAG_END;
	//char temp[PIXARM_MSG_SIZE];
	uint8_t* txbuf = (uint8_t*)(&sync_msg);
	//itoa(*txbuf, temp, 16);
	//gcs_send_text_P(SEVERITY_LOW, PSTR(temp));
	int16_t size = AGD_NAV_PORT->write(txbuf, PIXARM_MSG_SIZE);
	gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_start_SYNC"));
	//itoa(size, temp, 16);
	//gcs_send_text_P(SEVERITY_LOW, PSTR(temp));
	if (size != PIXARM_MSG_SIZE) {
		gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_ERROR_SYNC"));
		//Log_Write_ARMPixT_error(PIXARM_ERROR_SYNC);
		agd_nav_state = sync;
	}
	else {
		agd_nav_state = ack;
	}
}

void Copter::nav_ack() {
	int16_t incoming[PIXARM_MSG_SIZE];
	sAPP_PIXARM_SYNC* ack_msg;
	int16_t size;
	char temp[PIXARM_MSG_SIZE];

	size = AGD_NAV_PORT->available();   // Number of bytes available in rx buffer

	/*if (size != 0) {
		for (int16_t i = 0; i < size; ++i) {
			incoming[i] = AGD_NAV_PORT->read();
		}
		gcs_send_text_P(SEVERITY_LOW, PSTR("ack msg"));
		for (int16_t i = 0; i < size; ++i) {
			itoa(incoming[i], temp, 16);
			gcs_send_text_P(SEVERITY_LOW, PSTR(temp));
		}
		gcs_send_text_P(SEVERITY_LOW, PSTR("ack size"));
		itoa(size, temp, 16);
		gcs_send_text_P(SEVERITY_LOW, PSTR(temp));
	}*/
	
	if (size != PIXARM_MSG_SIZE) {
		agd_nav_state = ack;
		//gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_no_ACK"));
	}
	else {
		for (int16_t i = 0; i < size; ++i) {
			incoming[i] = AGD_NAV_PORT->read();
		}
		ack_msg = (sAPP_PIXARM_SYNC*) incoming;
		if (ack_msg->cmd != PIXARM_CMD_SYNC) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_ERROR_ACK"));
			//Log_Write_ARMPixT_error(PIXARM_ERROR_ACK);
			agd_nav_state = sync;
		}
		else {
			agd_nav_state = request;
		}
	}
}

void Copter::nav_req() {
	sAPP_PIXARM_READ_REQ req_msg;
	req_msg.cmd = PIXARM_CMD_READ_REQ;
	req_msg.rotation_absolute = ahrs.yaw_sensor;
	req_msg.flag = PIXARM_FLAG_END;
	uint8_t* txbuf = (uint8_t*)(&req_msg);

	int16_t size = AGD_NAV_PORT->write(txbuf, PIXARM_MSG_SIZE);
	if (size != PIXARM_MSG_SIZE) {
		gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_ERROR_REQ"));
		//Log_Write_ARMPixT_error(PIXARM_ERROR_REQ);
		agd_nav_state = request;
	}
	else {
		agd_nav_state = read_data;
	}
}

void Copter::nav_read() {
	char incoming[8];
	sAPP_PIXARM_READ_DATA* data;
	int16_t size;

	size = AGD_NAV_PORT->available();   // Number of bytes available in rx buffer

	if (size != PIXARM_MSG_SIZE) {
		agd_nav_state = read_data;
	}
	else {
		for (int16_t i = 0; i < size; ++i) {
			incoming[i] = AGD_NAV_PORT->read();
		}
		data = (sAPP_PIXARM_READ_DATA*) incoming;
		if (data->cmd != PIXARM_CMD_READ_DATA) {
			gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_ERROR_READ"));
			//Log_Write_ARMPixT_error(PIXARM_ERROR_READ);
			agd_nav_state = read_data;
		}
		else {
			agd_nav_state = request;
			x_inten = data->x_intensity;
			y_inten = data->y_intensity;
			z_inten = data->z_intensity;
			rotation_abs = data->rotation_absolute;
			gcs_send_text_P(SEVERITY_LOW, PSTR("PIXARM_READ_SUCCESS"));
			//Log_Write_ARMPixT();

		}
	}
}


void Copter::run_nav()
{
	switch (agd_nav_state) {
	case sync:
		nav_sync();
		break;
	case ack:
		nav_ack();
		break;
	case request:
		nav_req();
		break;
	case read_data:
		nav_read();
		break;
	}
}
