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
float HEIGHT = 1.7;


#define THRUST_0 1925
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
unsigned char vicon_available = 0;

short roll_angle, pitch_angle, yaw_angle;
short roll_vel, pitch_vel, yaw_vel;

unsigned char var_getted;
short U_z;
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
void *sendCmmdThread(void);

int main(int argc, char *argv[]) {

	pthread_t p_acithread, p_sendCmmdThread;

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
	aciSetEngineRate(200, 20);

	pthread_create(&p_acithread, NULL, aciThread, NULL);

	aciGetDeviceCommandsList();

	printf("Waiting for command list...\n");
	while(!cmd_ready) usleep(1000);

	usleep(1000000);
	pthread_create(&p_sendCmmdThread, NULL, sendCmmdThread, NULL);
	printf("Testing begins!\n");
  	pthread_join(p_acithread, NULL);
  	pthread_join(p_sendCmmdThread, NULL);

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
		usleep(50000);
	}
	return NULL;
}

void transmit(void* byte, unsigned short cnt) {

	unsigned char *tbyte = (unsigned char *) byte;
	for (int i = 0; i < cnt; i++) {
		write(fd, &tbyte[i], 1);
	}
}


void *sendCmmdThread(void) {
	aciUpdateCmdPacket(0);
	usleep(10000);
}
 
void cmdListUpdateFinished() {
	
	printf("command list getted!\n");
	aciAddContentToCmdPacket(0, 0x0F00, &roll_angle);
	aciAddContentToCmdPacket(0, 0x0F01, &pitch_angle);
	aciAddContentToCmdPacket(0, 0x0F02, &yaw_angle);

	aciAddContentToCmdPacket(0, 0x0F03, &roll_vel);
	aciAddContentToCmdPacket(0, 0x0F04, &pitch_vel);
	aciAddContentToCmdPacket(0, 0x0F05, &yaw_vel);

	aciAddContentToCmdPacket(0, 0x0F06, &U_z);

	// aciAddContentToCmdPacket(0, 0x0F06, &roll_feedback);
	// aciAddContentToCmdPacket(0, 0x0F07, &roll_feedback);
	// aciAddContentToCmdPacket(0, 0x0F08, &yaw_feedback);


	aciAddContentToCmdPacket(1, 0x0600, &ctrl_mode);
	aciAddContentToCmdPacket(1, 0x0601, &ctrl_enabled);
	aciAddContentToCmdPacket(1, 0x0602, &disable_motor_onoff_by_stick);
	aciAddContentToCmdPacket(1, 0x0603, &vicon_available);
	aciSendCommandPacketConfiguration(0, 0);
	// second variable, if set this not zero, it will send the last command until it gets an acknowledge
	aciSendCommandPacketConfiguration(1, 1);

	pitch_angle = 0;
	roll_angle = 0;
	yaw_angle = 0;

	roll_vel = 0;
	pitch_vel = 0;
	yaw_vel = 0;

	U_z = 0;


	ctrl_mode=0x04;
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