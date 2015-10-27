#pragma once

/*
 * PID configuration
 */

struct pid_conf {
	float k;
	float ti;
	float td;
	float ts;
};

msg_t pid_node(void * arg);
