#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

constexpr uint16_t NEOPIXEL_CAN_ID_INCOMING = 0x785;

neopixelCan::neopixelCan(uint16_t c_i, uint8_t fpl, const char* name){
	// Prepare the outgoing can packet
	_outgoing_packet.can_id = c_i;
	_outgoing_packet.can_dlc = fpl;
	// General work for socket binding
	_ifname = name;
    if((_s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		ROS_ERROR("Error while opening socket\n");
		throw -1;
	}
	strcpy(_ifr.ifr_name, _ifname);
	ioctl(_s, SIOCGIFINDEX, &_ifr);
	_addr.can_family  = AF_CAN;
	_addr.can_ifindex = _ifr.ifr_ifindex;
	// Bind socket CAN to an interface
	if(bind(_s, (struct sockaddr *)&_addr, sizeof(_addr)) < 0) {
		ROS_ERROR("Error in socket bind");
		throw -2;
	}
}
void neopixelCan::sendCAN(const uint8_t data){
    // Store data into the data potion of the CAN packet
    _outgoing_packet.data[0] = data;
    // Send out the CAN packet
    write(_s, &_outgoing_packet, sizeof(_outgoing_packet));
}
bool neopixelCan::waitforAck(){
	ROS_INFO("Now waiting for acknowledgement message.");
	bool first_pass = true;
	do
	{
		if(!first_pass){
			ROS_INFO("The received CAN message did not match the expected acknowledgment message. Waiting again.");
		}
		recv(_s, &_incoming_packet, sizeof(_incoming_packet), 0);
		ROS_INFO("CAN message received. Checking validity...");
		first_pass = false;
	} while (_incoming_packet.can_id != NEOPIXEL_CAN_ID_INCOMING || _incoming_packet.data[0] != 1);
	ROS_INFO("The expected acknowledgement message was received.");
	return true;
	// for now, this function cannot time out and return false
	// this will be implemented in a wrapper lib for can later
}