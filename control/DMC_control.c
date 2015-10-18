#include <lcm/lcm.h>
#include "../lcmtypes/DMC_cmmd_t.h"

int main () {

	float counter = 0.0;
	lcm_t * lcm = lcm_create(NULL);
	if(!lcm)
		return 1;

	DMC_cmmd_t msg = {

		.timestamp = 0,
		.rpm = {10, 10, 10, 10},
	};

	while (1){

		msg.rpm[0] = 30+10*sin(counter);
		msg.rpm[1] = 30+10*sin(counter);
		msg.rpm[2] = 30+10*sin(counter);
		msg.rpm[3] = 30+10*sin(counter);

		DMC_cmmd_t_publish(lcm, "DMC_cmmd", &msg);
		counter += 0.1;

		usleep(10000);

	}

	lcm_destroy(lcm);
	return 0;

}