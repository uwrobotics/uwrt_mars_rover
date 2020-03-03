#include <cstdlib>

#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

int main(int argc, char** argv){
    char* can_interface = "vcan0";
    neopixel neopixel_obj(argc, argv, can_interface);
    neopixel_obj.run();
    return EXIT_SUCCESS;
}