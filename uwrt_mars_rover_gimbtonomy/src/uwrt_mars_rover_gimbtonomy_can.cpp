#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_can.h"

neopixel_can::neopixel_can(uint16_t c_i, uint8_t fpl, const char* name){
	packet.can_id = c_i;
	packet.can_dlc = fpl;
	ifname = name;
	
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Error while opening socket\n");
		throw -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	//printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
	// Bind socket CAN to an interface
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ROS_ERROR("Error in socket bind");
		throw -2;
	}
}
void neopixel_can::sendCAN(unsigned int data){
    // Store data into the data potion of the CAN packet
    packet.data[0] = data;
    // Send out the CAN packet
    write(s, &packet, sizeof(packet));
}