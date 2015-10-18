/**
 * Generate control command based on guidance
 * Command will published via LCM and excuted by sendCommnad
 *  Wei Ding
 *  
 */

#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <mutex>
#include <iostream>
 #include <stdint.h>
#include "../lcmtypes/state_t.hpp"
#include "../lcmtypes/vicon_state_t.hpp"
#include "../lcmtypes/command_t.hpp"

#include "../include/util.h"


//////////////////////////////////
///Set up values
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
///


/////////////////////////////////////
///state variables
///
///

float x, y, z;
float vx, vy, vz;
float phi, theta, psi;
float v_phi, v_theta, v_psi;

std::mutex vicon_mutex;

/////////////////////////////////////
///should be states of UAV after navigation
///codes handled all the states through fusing
///all sensors
///
///here only use Vicon first
///
//////////////////////////////

lcm::LCM lcm;
vicon_state_t vicon_state;
command_t cmmdMsg;



int16_t pitch=0;
int16_t roll=0;
int16_t yaw=0;
int16_t thrust=0;
int16_t ctrl = 0xF;


class Handler 
{
    public:
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vicon_state_t* msg)
        {
 			vicon_state = *msg;
        }

};


void viconStateUpdateThread(void) {


    Handler handlerObject;
    lcm.subscribe("vicon_state", &Handler::handleMessage, &handlerObject);

	while(1) {

		vicon_mutex.lock();
        lcm_handle(lcm);
        vicon_mutex.unlock();

		usleep(0.9e4);

	}

}

void cmmdGenerationThread(void) {

	while(1) {

		//lock the mutex while transmitting data to control loop
		vicon_mutex.lock();

		x = vicon_state.position[0]/1000;
		y = vicon_state.position[1]/1000;
		z = vicon_state.position[2]/1000;

		vx = vicon_state.velocity[0];
		vy = vicon_state.velocity[1];
		vz = vicon_state.velocity[2];
    	
        vicon_mutex.unlock();

        //////////////generate command based on the state
        // thrust = (int)(thrust_0 + (z - HEIGHT)*atoi(argv[0]) + vz*atoi(argv[1]));
    	// Height control
        thrust = (int)(thrust_0 + (z - HEIGHT) * GAIN_P_H + vz * GAIN_D_H);
        // printf("Thrust : %d\n", thrust);
        // thrust = 1600;
        if(thrust >= saturation_up) thrust = saturation_up;
        if(thrust <= saturation_down) thrust = saturation_down;

        pitch = (int)(pitch_0 + (x - X_DES) * GAIN_P_PITCH + vx * GAIN_D_PITCH);
        //(int)(pitch_0 + (pitch_angle - PITCH) * GAIN_P_PITCH + pitch_vel * GAIN_D_PITCH);
        roll  = (int)(roll_0 + (y - Y_DES) * GAIN_P_ROLL + vy * GAIN_D_ROLL);
        //(int)(roll_0 + (roll_angle - ROLL) * GAIN_P_ROLL + roll_vel * GAIN_D_ROLL);
        yaw = (int)(yaw_0 + (psi - YAW_DES));
        //(int)(yaw_0 + (yaw_angle - YAW) * GAIN_P_YAW + yaw_vel * GAIN_D_YAW);
        
        //enable height control here
        ctrl = 0x8;

        cmmdMsg.timestamp = utime_now();
		cmmdMsg.thrust = thrust;
		cmmdMsg.roll = roll;
		cmmdMsg.pitch = pitch;
		cmmdMsg.yaw = yaw;

		lcm.publish("command_msg", &cmmdMsg);

        usleep(5000);

    }


}

int main() {

	if (!lcm.good()) return 1;


}

	// x = msg->position[0]/1000;
	// y = msg->position[1]/1000;
	// z = msg->position[2]/1000;

	// vx = msg->velocity[0];
	// vy = msg->velocity[1];
	// vz = msg->velocity[2];