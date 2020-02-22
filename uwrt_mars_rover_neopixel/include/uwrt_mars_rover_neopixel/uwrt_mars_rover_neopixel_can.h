#pragma once
#include <string>
// Some includes needed for the CAN lib
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
// ros.h is included here and in the other header file?
// Need to include ros.h for some ROS_ERROR statements
#include "ros/ros.h"

// This is a library the Neopixel Node uses to send messages over CAN.
// If a general CAN library is added to the repo, delete this and use the general CAN library.
class neopixelCan{
private:
    int _s;
	// Build the CAN Socket
	sockaddr_can _addr;
	// Build my CAN packet
	can_frame _packet;
	// What is this for?
	ifreq _ifr;
    const char* _ifname;
public:
    // Constructor for my CAN class
    neopixelCan(uint16_t c_i, uint8_t fpl, const char* name);
    // sendCAN function that only sends 1 integer for neopixels
    void sendCAN(unsigned const int data);
};