#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

neopixelCan::neopixelCan(uint16_t c_i, uint8_t fpl, const char* name){
	_packet.can_id = c_i;
	_packet.can_dlc = fpl;
	_ifname = name;
	
    if((_s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Error while opening socket\n");
		throw -1;
	}

	strcpy(_ifr.ifr_name, _ifname);
	ioctl(_s, SIOCGIFINDEX, &_ifr);
	
	_addr.can_family  = AF_CAN;
	_addr.can_ifindex = _ifr.ifr_ifindex;

	//printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
	// Bind socket CAN to an interface
	if(bind(_s, (struct sockaddr *)&_addr, sizeof(_addr)) < 0) {
		ROS_ERROR("Error in socket bind");
		throw -2;
	}
}
void neopixelCan::sendCAN(const uint8_t data){
    // Store data into the data potion of the CAN packet
    _packet.data[0] = data;
    // Send out the CAN packet
    write(_s, &_packet, sizeof(_packet));
}