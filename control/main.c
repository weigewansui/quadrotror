/**
 * Hovering for hummingbird with PD controller
 */

#include "../include/asctecCommIntf.h"

#include <stdio.h>
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>
#include <stdint.h>

#include <lcm/lcm.h>
#include "../lcmtypes/state_t.h"
#include "../lcmtypes/transmitter_t.h"
#include "../lcmtypes/vicon_state_t.h"
#include "../lcmtypes/command_t.h"

#include "../include/util.h"
#include "../include/filter_util.h"
#define NUM_SAMPLES_MED 3
#define NUM_SAMPLES_AVG 3

pthread_mutex_t navigation_mutex;
pthread_mutex_t USB_port_mutex;
//////////////////////////////////
///control stuffs
float HEIGHT = 2.0;


#define THRUST_0 1840
#define COEFF_H 165
#define GAIN_P_H -2.4142135
#define GAIN_D_H -2.210
#define saturation_up 2400
#define saturation_down 1700

float YAW_DES = 0;
#define COEFF_YAW 717
#define yaw_0 0
#define GAIN_P_YAW -2
#define yaw_saturation 1500


float X_DES = 0.0;

#define COEFF_ROLL 2200
#define roll_0 0
#define GAIN_P_X -2.4142135
#define GAIN_D_X -2.210
#define roll_saturation 1500

float Y_DES = 0.0;

#define COEFF_PITCH 2200
#define pitch_0 0
#define GAIN_P_Y -2.4142135
#define GAIN_D_Y -2.210
#define pitch_satuarion 1500



///////////////////////////////////////

int fd;

unsigned char ctrl_mode = 0x02;
unsigned char ctrl_enabled = 0;
unsigned char disable_motor_onoff_by_stick = 0;

float x, y, z;
float vx, vy, vz;
float phi, theta, psi;
float v_phi, v_theta, v_psi;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;
int32_t pitch_vel, roll_vel, yaw_vel;
int32_t acc_x, acc_y, acc_z;
int16_t ch0, ch1, ch2, ch3, ch4, ch5, ch6, ch7;

unsigned char var_getted;

int16_t pitch=0;
int16_t roll=0;
int16_t yaw=0;
int16_t thrust=0;
int16_t ctrl = 0xF;

unsigned char cmd_ready = 0;
int16_t motor_start=1;

void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished();
void cmdListUpdateFinished();
void paramListUpdateFinished();
void *aciThread(void);
void *viconStateThread(void);
void startStopMotor();
void startMotors();
void *sendCmmdThread(void);
void *fetchDataThread(void);
void *publishCmmdThread(void);


lcm_t * lcm;
command_t cmmdMsg;
vicon_state_t vicon_state;
state_t state_msg;
transmitter_t trans_msg;


float desired_x_accel, desired_y_accel, desired_z_accel;
float desired_yaw_rate, desired_pitch_angle, desired_roll_angle;

int main(int argc, char *argv[]) {

	pthread_t p_viconThread, p_sendCmmdThread, p_publishCmmdThread, p_fetchDataThread;
	pthread_t p_acithread;

	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	struct termios port_settings; // structure to store the port settings in

	cfsetispeed(&port_settings, B57600); // set baud rates
	port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
	port_settings.c_iflag = IGNPAR;
	port_settings.c_oflag = 0;
	port_settings.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

	aciInit();
	aciSetSendDataCallback(&transmit);
	aciSetVarListUpdateFinishedCallback(&varListUpdateFinished);
	aciSetCmdListUpdateFinishedCallback(&cmdListUpdateFinished);
	aciSetParamListUpdateFinishedCallback(&paramListUpdateFinished);
	aciSetEngineRate(100, 10);

	lcm = lcm_create(NULL);
 	
 	if(!lcm)
    	return 1;

	pthread_create(&p_acithread, NULL, aciThread, NULL);
	
	aciGetDeviceVariablesList();
	while(!var_getted) usleep(1000);
	aciGetDeviceCommandsList();

	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);

	printf("starting motors!\n");
	//turn on motors()
	startMotors();
	usleep(1000000);
	
	//start getting data from Vicon
	pthread_create(&p_viconThread, NULL, viconStateThread, NULL);
	pthread_create(&p_sendCmmdThread, NULL, sendCmmdThread, NULL);
	// pthread_create(&p_fetchDataThread, NULL, fetchDataThread, NULL);
	pthread_create(&p_publishCmmdThread, NULL, publishCmmdThread, NULL);
	
  	pthread_join(p_viconThread, NULL);
  	pthread_join(p_sendCmmdThread, NULL);
  	// pthread_join(p_fetchDataThread, NULL);
  	pthread_join(p_publishCmmdThread, NULL);
  	pthread_join(p_acithread, NULL);

}


static void state_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const vicon_state_t * msg, void * user)
{
    
	vicon_state = *msg;

}

void *viconStateThread(void) {

	vicon_state_t_subscribe(lcm, "vicon_state", &state_handler, NULL);

	while(1) {
	
        lcm_handle(lcm);
		usleep(10000);

	}

	return NULL;

}

void *aciThread(void)
{
	int result = 0;
	unsigned char data = 0;
	while(1)
	{

		result = read(fd, &data, 1);

		while (result > 0) {
			aciReceiveHandler(data);
			result = read(fd, &data, 1);
		}

		aciEngine();
		usleep(10000);
	}
	return NULL;
}


void startMotors(int hi) {

	ctrl = 0xF;
	thrust = 0;
	roll = 0;
	pitch = 0;
	yaw = -2047;
	aciUpdateCmdPacket(0);

}


void transmit(void* byte, unsigned short cnt) {

	unsigned char *tbyte = (unsigned char *) byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
}


void *publishCmmdThread(void) {

	while(1){

		cmmdMsg.timestamp = utime_now();
		cmmdMsg.thrust = thrust;
		cmmdMsg.roll = roll;
		cmmdMsg.pitch = pitch;
		cmmdMsg.yaw = yaw;

		command_t_publish(lcm, "command_msg", &cmmdMsg);
		usleep(10000);

	}

	return NULL;

}


// store previous command in order to filter out unreasonable command jump
int pre_thrust, pre_roll, pre_pitch, pre_yaw;
//Threshold for unreasonable jump
int THRUST_JUMP_THRES = 100, ROLL_JUMP_THRES = 50, PITCH_JUMP_THRES = 100, YAW_JUMP_THRES = 50;

float PITCH_CMMD[NUM_SAMPLES_MED], PITCH_MED[NUM_SAMPLES_AVG], PITCH_AVG[NUM_SAMPLES_AVG];

void *sendCmmdThread(void) {

	while(1) {

		//lock the mutex while transmitting data to control loop
		pthread_mutex_lock(&navigation_mutex);

		x = vicon_state.position[0]/1000;
		y = vicon_state.position[1]/1000;
		z = vicon_state.position[2]/1000;

		psi = vicon_state.attitude[2];

		vx = vicon_state.velocity[0];
		vy = vicon_state.velocity[1];
		vz = vicon_state.velocity[2];
    	
        pthread_mutex_unlock(&navigation_mutex);

        //////////////generate command based on the state
        // thrust = (int)(thrust_0 + (z - HEIGHT)*atoi(argv[0]) + vz*atoi(argv[1]));
    	// Height control
    	
    	desired_z_accel = (z - HEIGHT) * GAIN_P_H + vz * GAIN_D_H;
        thrust = (int)(COEFF_H * desired_z_accel + THRUST_0);
        // printf("Thrust : %d\n", thrust);
        // thrust = 1600;
        if(thrust >= saturation_up) thrust = saturation_up;
        if(thrust <= saturation_down) thrust = saturation_down;


        // get the desired acceleration through LQR gains
        desired_x_accel = (x - X_DES) * GAIN_P_X + vx * GAIN_D_X;
        desired_y_accel = (y - Y_DES) * GAIN_P_Y + vy * GAIN_D_Y;

        //get angle through small angle approximation
        //
        // for x direction, pitch angle is the opposite sign as acceleration
        // for y direction, roll angle is the same sign as acceleration
        // 
        // //////////////////////////////////////
        
        desired_roll_angle = -desired_y_accel / (desired_z_accel + 9.8); 
        desired_pitch_angle = -desired_x_accel / (desired_z_accel + 9.8);

        //generation roll pitch command based on desired angle
        pitch = (int)(pitch_0 + desired_pitch_angle * COEFF_PITCH);
        // if(abs(pitch - pre_pitch) >= PITCH_JUMP_THRES) pitch = pre_pitch;

        // printf("Pitch: %d, Previous Pitch: %d: \n", pitch, pre_pitch);
        // pitch = 1000;
        roll  = (int)(roll_0 + desired_roll_angle * COEFF_ROLL);

        //yaw angle is controlled by velocity
        desired_yaw_rate = (psi - deg2rad(YAW_DES))*GAIN_P_YAW;
        yaw = (int)(yaw_0 + COEFF_YAW * desired_yaw_rate);

        // printf("current psi: %f:\t, designed yaw:%f\n", psi, deg2rad(YAW_DES));
        
        // set command saturation 
        if (roll >= roll_saturation) roll = roll_saturation;
        if (roll <= -roll_saturation) roll = -roll_saturation;

        if (pitch >= pitch_satuarion) pitch = pitch_satuarion;
        if (pitch <= -pitch_satuarion) pitch = -pitch_satuarion;

        if (yaw >= yaw_saturation) yaw = yaw_saturation;
        if (yaw <= -yaw_saturation) yaw = -yaw_saturation;

        ////////////////////
        //set control mode here
        //
        //0x4 for yaw only
        //0x8 for thrust only
        //0x1 for pitch only
        //0x2 for roll only
        //
        ////////////////////
        
        ctrl = 0xF;

        //send command out
        // pthread_mutex_lock(&USB_port_mutex);
        aciUpdateCmdPacket(0);
		// pthread_mutex_unlock(&USB_port_mutex);
        
        pre_thrust = thrust;
        pre_pitch = pitch;
        pre_roll = roll;

        usleep(10000);

    }

    return NULL;
}

short int firstPoint = 0, SecondPoint = 0, ThirdPoint = 0, finished = 0;
int TimeCounter = 0;

void* pointLoiteringThread(void) {

	while(1) {

		if (abs(x) <= 0.1 && abs(y) <= 0.1 && abs(z - HEIGHT) <= 0.1) {

			firstPoint = 1;
			SecondPoint = 0;
			ThirdPoint = 0;
			TimeCounter = 0;
		}

		if (abs(x - 0.5) <= 0.1 && abs(y - 0.5) <= 0.1 && abs(z - HEIGHT) <= 0.1) {

			firstPoint = 0;
			SecondPoint = 1;
			ThirdPoint = 0;
			TimeCounter = 0;
		}

		if (abs(x - 0.5) <= 0.1 && abs(y + 0.5) <= 0.1 && abs(z - HEIGHT) <= 0.1) {

			firstPoint = 0;
			SecondPoint = 0;
			ThirdPoint = 1;
			TimeCounter = 0;
		}

		if( !firstPoint && !SecondPoint && !ThirdPoint ) usleep(1000);

		if(firstPoint && !SecondPoint && !ThirdPoint && TimeCounter != 5) {

			TimeCounter += 1;
			usleep(1e6); 

		} else if (firstPoint && !SecondPoint && !ThirdPoint && TimeCounter == 5) {

			X_DES = 0.5;
			Y_DES = 0.5;

		}

		if(firstPoint && SecondPoint && !ThirdPoint && TimeCounter != 5) {

			TimeCounter += 1;
			usleep(1e6);

		} else if (firstPoint && SecondPoint && !ThirdPoint && TimeCounter == 5) {

			X_DES = 0.5;
			Y_DES = -0.5;
		}

	}

}

void *fetchDataThread(void) {

	while(1) {

		if(var_getted) {

			pthread_mutex_lock(&USB_port_mutex);
			aciSynchronizeVars();
			pthread_mutex_unlock(&USB_port_mutex);
			
			state_msg.timestamp = utime_now();
			state_msg.position[0] = 0;
			state_msg.position[1] = 0;
			state_msg.position[2] = 0;
			state_msg.velocity[0] = 0;
			state_msg.velocity[1] = 0;
			state_msg.velocity[2] = 0;
			state_msg.accel[0] = acc_x;
			state_msg.accel[1] = acc_y;
			state_msg.accel[2] = acc_z;
			state_msg.angle[0] = angle_roll;
			state_msg.angle[1] = angle_pitch;
			state_msg.angle[2] = angle_yaw;
			state_msg.angular_vel[0] = roll_vel;
			state_msg.angular_vel[1] = pitch_vel;
			state_msg.angular_vel[2] = yaw_vel;
			state_msg.angular_accel[0] = 0;
			state_msg.angular_accel[1] = 0;
			state_msg.angular_accel[2] = 0;

			trans_msg.timestamp = utime_now();
			trans_msg.ch0 = ch0;
			trans_msg.ch1 = ch1;
			trans_msg.ch2 = ch2;
			trans_msg.ch3 = ch3;
			trans_msg.ch4 = ch4;
			trans_msg.ch5 = ch5;
			trans_msg.ch6 = ch6;
			trans_msg.ch7 = ch7;
			
			state_t_publish(lcm, "state", &state_msg);
			transmitter_t_publish(lcm, "transmitter", &trans_msg);

		}

		usleep(10000);

	}

	return NULL;

}
 
void cmdListUpdateFinished() {
	
	printf("command list getted!\n");
	aciAddContentToCmdPacket(0, 0x050A, &pitch);
	aciAddContentToCmdPacket(0, 0x050B, &roll);
	aciAddContentToCmdPacket(0, 0x050C, &yaw);
	aciAddContentToCmdPacket(0, 0x050D, &thrust);
	aciAddContentToCmdPacket(0, 0x050E, &ctrl);
	aciAddContentToCmdPacket(1, 0x0600, &ctrl_mode);
	aciAddContentToCmdPacket(1, 0x0601, &ctrl_enabled);
	aciAddContentToCmdPacket(1, 0x0602, &disable_motor_onoff_by_stick);
	aciSendCommandPacketConfiguration(0, 0);
	// second variable, if set this not zero, it will send the last command until it gets an acknowledge
	aciSendCommandPacketConfiguration(1, 1);

	pitch = 0;
	roll = 0;
	yaw = 0;
	thrust = 0;
	ctrl = 0xF;

	ctrl_mode=0x02;
	ctrl_enabled=1;
	// disable_motor_onoff_by_stick=1;
	aciUpdateCmdPacket(0);
	aciUpdateCmdPacket(1);
	cmd_ready=1;
}

void paramListUpdateFinished() {
 
}


void varListUpdateFinished() {

	printf("Start getting variables!\n");

	aciAddContentToVarPacket(2, 0x0300, &angle_pitch);
	aciAddContentToVarPacket(2, 0x0301, &angle_roll);
	aciAddContentToVarPacket(2, 0x0302, &angle_yaw);
	aciAddContentToVarPacket(2, 0x0200, &pitch_vel);
	aciAddContentToVarPacket(2, 0x0201, &roll_vel);
	aciAddContentToVarPacket(2, 0x0202, &yaw_vel);
	aciAddContentToVarPacket(2, 0x0203, &acc_x);
	aciAddContentToVarPacket(2, 0x0204, &acc_y);
	aciAddContentToVarPacket(2, 0x0205, &acc_z);

	aciAddContentToVarPacket(2, 0x0600, &ch0);
	aciAddContentToVarPacket(2, 0x0601, &ch1);
	aciAddContentToVarPacket(2, 0x0602, &ch2);
	aciAddContentToVarPacket(2, 0x0603, &ch3);
	aciAddContentToVarPacket(2, 0x0604, &ch4);
	aciAddContentToVarPacket(2, 0x0605, &ch5);
	aciAddContentToVarPacket(2, 0x0606, &ch6);
	aciAddContentToVarPacket(2, 0x0607, &ch7);
	aciSetVarPacketTransmissionRate(2,10);
	aciVarPacketUpdateTransmissionRates();
	aciSendVariablePacketConfiguration(2);
	var_getted=1;
 
}