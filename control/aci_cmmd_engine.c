#include "../include/asctecCommIntf.h"
 
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <pthread.h>
#include <time.h>

#include <lcm/lcm.h>
#include "../lcmtypes/DMC_cmmd_t.h"
#include "../lcmtypes/var_t.h"

#include "../include/util.h"
 
int fd;
 
unsigned char ctrl_mode = 0;
unsigned char ctrl_enabled = 0;
unsigned char disable_motor_onoff_by_stick = 0;
 
int16_t motor1=0;
int16_t motor2=0;
int16_t motor3=0;
int16_t motor4=0;

int32_t angle_pitch;
int32_t angle_roll;
int32_t angle_yaw;
int32_t pitch_vel, roll_vel, yaw_vel;
int32_t acc_x, acc_y, acc_z;
 
unsigned char cmd_ready;
unsigned char var_getted;
unsigned char cmmd_msg_get;
 
void transmit(void* byte, unsigned short cnt);
void varListUpdateFinished(void);
void cmdListUpdateFinished();
void *aciThread(void);
void *subscribeDMCCmmdThread(void);
void *publishVarThread(void);
void *sendOutDMCcmmdThread(void);
void startStopMotor(int);
void *fetchDataThread(void);



//lcm stuffs

lcm_t * lcm;
var_t var_msg;
 
int main(int argc, char *argv[]) {

  var_getted = 0;
  cmd_ready = 0;
  cmmd_msg_get = 0;
  
  lcm = lcm_create(NULL);

  if(!lcm){
    printf("lcm initialize failed!\n");
    return 1;
  }
 
  pthread_t p_acithread, p_subscribeDMCCmmdThread, p_sendOutDMCcmmdThread, p_publishVarThread, p_fetchDataThread;
 
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

  aciSetEngineRate(100, 10);
 
  //start running acithread
  pthread_create(&p_acithread, NULL, aciThread, NULL);

  aciGetDeviceVariablesList();
  printf("Waiting for variable list.. \n");
  while(!var_getted) usleep(1000);

  // start fetching data and publish via LCM
  pthread_create(&p_fetchDataThread, NULL, fetchDataThread, NULL);

  aciGetDeviceCommandsList();
  printf("Waiting for command list...\n");
  while(!cmd_ready) usleep(1000);
  
  pthread_create(&p_subscribeDMCCmmdThread, NULL, subscribeDMCCmmdThread, NULL);
  pthread_create(&p_sendOutDMCcmmdThread, NULL, sendOutDMCcmmdThread, NULL);

  pthread_join(p_acithread, NULL);
  pthread_join(p_fetchDataThread, NULL);
  pthread_join(p_subscribeDMCCmmdThread, NULL);
  pthread_join(p_sendOutDMCcmmdThread, NULL);

}
 

static void DMC_cmmd_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
        const DMC_cmmd_t * cmmd_msg, void * user)
{
  
  motor1 = cmmd_msg->rpm[0];
  motor2 = cmmd_msg->rpm[1];
  motor3 = cmmd_msg->rpm[2];
  motor4 = cmmd_msg->rpm[3];

}

void *subscribeDMCCmmdThread(void) {

  DMC_cmmd_t_subscribe(lcm, "DMC_cmmd", &DMC_cmmd_handler, NULL);

  while(1) {

    lcm_handle(lcm);
    usleep(10000);

  }

}

void *sendOutDMCcmmdThread(void) {

  while(1) {
    
    aciUpdateCmdPacket(0);
    usleep(10000);

  }

}
 
void transmit(void* byte, unsigned short cnt) {
 
  unsigned char *tbyte = (unsigned char *) byte;

  for (int i = 0; i < cnt; i++) {

    write(fd, &tbyte[i], 1);
 
  }

}
 
void *aciThread(void) {

  int result = 0;
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
 

void *fetchDataThread(void) {

  while(1) {

    if(var_getted) {

      aciSynchronizeVars();
      
      var_msg.timestamp = utime_now();

      var_msg.accel[0] = acc_x;
      var_msg.accel[1] = acc_y;
      var_msg.accel[2] = acc_z;

      var_msg.angle[0] = angle_roll;
      var_msg.angle[1] = angle_pitch;
      var_msg.angle[2] = angle_yaw;

      var_msg.angular_vel[0] = roll_vel;
      var_msg.angular_vel[1] = pitch_vel;
      var_msg.angular_vel[2] = yaw_vel;
      
      var_t_publish(lcm, "variables", &var_msg);

    }

    usleep(10000);

  }

  return NULL;

}
 
void varListUpdateFinished(void) {

  printf("Start getting variables!\n");

  aciAddContentToVarPacket(0, 0x0300, &angle_pitch);
  aciAddContentToVarPacket(0, 0x0301, &angle_roll);
  aciAddContentToVarPacket(0, 0x0302, &angle_yaw);
  aciAddContentToVarPacket(0, 0x0200, &pitch_vel);
  aciAddContentToVarPacket(0, 0x0201, &roll_vel);
  aciAddContentToVarPacket(0, 0x0202, &yaw_vel);
  aciAddContentToVarPacket(0, 0x0203, &acc_x);
  aciAddContentToVarPacket(0, 0x0204, &acc_y);
  aciAddContentToVarPacket(0, 0x0205, &acc_z);

  aciSetVarPacketTransmissionRate(0,10);
  aciVarPacketUpdateTransmissionRates();
  aciSendVariablePacketConfiguration(0);
  var_getted=1;
 
}
 
void cmdListUpdateFinished() {

  printf("command list getted!\n");
  aciAddContentToCmdPacket(0, 0x0500, &motor1);
  aciAddContentToCmdPacket(0, 0x0501, &motor2);
  aciAddContentToCmdPacket(0, 0x0502, &motor3);
  aciAddContentToCmdPacket(0, 0x0503, &motor4);
  aciAddContentToCmdPacket(1, 0x0600, &ctrl_mode);
  aciAddContentToCmdPacket(1, 0x0601, &ctrl_enabled);
  aciAddContentToCmdPacket(1, 0x0602, &disable_motor_onoff_by_stick);
  aciSendCommandPacketConfiguration(0, 0);
  aciSendCommandPacketConfiguration(1, 1);
  motor1 = 0;
  motor2 = 0;
  motor3 = 0;
  motor4 = 0;
  ctrl_mode=0;
  ctrl_enabled=1;
  disable_motor_onoff_by_stick=1;
  aciUpdateCmdPacket(0);
  aciUpdateCmdPacket(1);
  cmd_ready=1;

}