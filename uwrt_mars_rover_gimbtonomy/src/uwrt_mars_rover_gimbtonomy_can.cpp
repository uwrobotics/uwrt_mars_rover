#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_can.h"

neopixel_can::neopixel_can(int can_id, int dlc, char* ifname) : packet.can_id(can_id), packet.can_dlc(dlc), ifname(ifname){
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_INFO("Error while opening socket\n");
		throw -1;
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	//printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
	// Bind socket CAN to an interface
	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ROS_INFO("Error in socket bind");
		throw -2;
	}
}
void neopixel_can::sendCAN(unsigned int data) const{
    // Store data into the data potion of the CAN packet
    packet.data[0] = data;
    // Send out the CAN packet
    write(s, &packet, sizeof(struct can_frame));
}