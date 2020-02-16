// This is a library the Neopixel Node uses to send messages over CAN.
// If a general CAN library is added to the repo, delete this and use the general CAN library.
class neopixel_can{
private:
    int s;
	// Build the CAN Socket
	struct sockaddr_can addr;
	// Build my CAN packet
	struct can_frame packet;
	// What is this for?
	struct ifreq ifr;
    const char* ifname;
public:
    // Constructor for my CAN class
    neopixel_can(int can_id, int frame_payload_length, char* ifname);
    // sendCAN function that only sends 1 integer for neopixels
    void sendCAN(unsigned const int data) const;
};