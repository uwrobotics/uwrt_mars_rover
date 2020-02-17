#include <cstdlib>
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

#include "ros/ros.h"

#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_can.h"
#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_neopixel.h"

int main(int argc, char** argv){
    neopixel neopixel_obj(argc, argv);
    neopixel_obj.run();
    return EXIT_SUCCESS;
}