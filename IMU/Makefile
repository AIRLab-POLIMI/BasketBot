MODULE = IMU_module
MODULE_NAME = imu

USE_VERBOSE_COMPILE = yes

PACKAGES = led

PRJ_CSRC = 

PRJ_CPPSRC =  \
	nodes/madgwick_node.cpp \
	main.cpp

ULIBS += -lm

include $(R2P_ROOT)/core/r2p.mk

