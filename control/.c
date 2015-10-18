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

//////////////////////////////////
///control stuffs
float HEIGHT = 1.0;
int thrust_0 = 1820;
int GAIN_P_H = -150;
int GAIN_D_H = -100;
int saturation_up = 2300;
int saturation_down = 1700;

float X_DES = 0.0;
int roll_0 = 0;
int GAIN_P_ROLL = -20;
int GAIN_D_ROLL = -10;

float Y_DES = 0.0;
int pitch_0 = 0;
int GAIN_P_PITCH = -20;
int GAIN_D_PITCH = -10;

float YAW_DES = 0.0;
int yaw_0 = 0;
int GAIN_P_YAW = -30;
int GAIN_D_YAW = -10;
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
// void *viconStateThread(void);
void startStopMotor();
void startMotors();
void increase();
// void *sendCmmdThread(void);
// void *publishCmmdThread(void);


// lcm_t * lcm;
// command_t cmmdMsg;

int main(int argc, char *argv[]) {

	// pthread_t p_viconThread, p_sendCmmdThread, p_publishCmmdThread;
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

	// lcm = lcm_create(NULL);
 	
 // 	if(!lcm)
 //    	return 1;

	// pthread_create(&p_viconThread, NULL, viconStateThread, NULL);
	// pthread_create(&p_sendCmmdThread, NULL, sendCmmdThread, NULL);
	// pthread_create(&p_publishCmmdThread, NULL, publishCmmdThread, NULL);
	pthread_create(&p_acithread, NULL, aciThread, NULL);

	startMotors();
	usleep(1000000);
	
	aciGetDeviceCommandsList();
	
	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);



  	// pthread_join(p_viconThread, NULL);
  	// pthread_join(p_sendCmmdThread, NULL);
  	// pthread_join(p_publishCmmdThread, NULL);
  		// pthread_join(p_acithread, NULL);

}


// static void state_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
//         const vicon_state_t * msg, void * user)
// {
    
// 	x = msg->position[0]/1000;
// 	y = msg->position[1]/1000;
// 	z = msg->position[2]/1000;

// 	vx = msg->velocity[0];
// 	vy = msg->velocity[1];
// 	vz = msg->velocity[2];

// }

// void *viconStateThread(void) {

// 	vicon_state_t_subscribe(lcm, "vicon_state", &state_handler, NULL);

// 	while(1) {

//         lcm_handle(lcm);
// 		usleep(1e4);

// 	}

// 	return NULL;

// }

void *aciThread(void) {

	int result = 0;
 	int timecounter = 0;
 	unsigned char data = 0;
 
	while (1) {
  	
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

void increase() {

	ctrl = 0xF;
	thrust = 100;
	roll = 0;
	pitch = 0;
	yaw = 0;
	aciUpdateCmdPacket(0);

}


void transmit(void* byte, unsigned short cnt) {

	unsigned char *tbyte = (unsigned char *) byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
}


// void *publishCmmdThread(void) {

// 	while(1){

// 		cmmdMsg.timestamp = utime_now();
// 		cmmdMsg.thrust = thrust;
// 		cmmdMsg.roll = roll;
// 		cmmdMsg.pitch = pitch;
// 		cmmdMsg.yaw = yaw;

// 		command_t_publish(lcm, "command_msg", &cmmdMsg);
// 		usleep(1e4);
// 	}

// }

// void *sendCmmdThread(void) {

// 	while(1) {
//     // thrust = (int)(thrust_0 + (z - HEIGHT)*atoi(argv[0]) + vz*atoi(argv[1]));
//     // Height control
//         thrust = (int)(thrust_0 + (z - HEIGHT) * GAIN_P_H + vz * GAIN_D_H);
//         // printf("Thrust : %d\n", thrust);
//         // thrust = 1600;
//         if(thrust >= saturation_up) thrust = saturation_up;
//         if(thrust <= saturation_down) thrust = saturation_down;

//         pitch = (int)(pitch_0 + (x - X_DES) * GAIN_P_PITCH + vx * GAIN_D_PITCH);
//         //(int)(pitch_0 + (pitch_angle - PITCH) * GAIN_P_PITCH + pitch_vel * GAIN_D_PITCH);
//         roll  = (int)(roll_0 + (y - Y_DES) * GAIN_P_ROLL + vy * GAIN_D_ROLL);
//         //(int)(roll_0 + (roll_angle - ROLL) * GAIN_P_ROLL + roll_vel * GAIN_D_ROLL);
//         yaw = (int)(yaw_0 + (psi - YAW_DES));
//         //(int)(yaw_0 + (yaw_angle - YAW) * GAIN_P_YAW + yaw_vel * GAIN_D_YAW);
        
//         //enable height control here
//         ctrl = 0x8;
//         aciUpdateCmdPacket(0);

//         cmmdMsg.timestamp = utime_now();
// 		cmmdMsg.thrust = thrust;
// 		cmmdMsg.roll = roll;
// 		cmmdMsg.pitch = pitch;
// 		cmmdMsg.yaw = yaw;

// 		// command_t_publish(lcm, "command_msg", &cmmdMsg);

//         usleep(1e4);
//     }

// }

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
 
}