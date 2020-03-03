#pragma once
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
// CAN lib includes done
#include "ros/ros.h"

// This is a library the Neopixel Node uses to send messages over CAN.
// If a general CAN library is added to the repo, delete this and use the general CAN library.
class neopixelCan{
private:
    int _s;
	// Build the CAN Socket
	sockaddr_can _addr;
	// Build the outgoing CAN packet
	can_frame _outgoing_packet{};
    // Build the incoming CAN packet
    can_frame _incoming_packet{};
	ifreq _ifr;
    const char* _ifname;
public:
    // Constructor for my CAN class
    neopixelCan(uint16_t c_i, uint8_t fpl, const char* name);
    // sendCAN function that only sends 1 integer for neopixels
    void sendCAN(const uint8_t data);
    // checks if the acknowledgement message was sent by the gimbtonomy board
    bool waitforAck();
};