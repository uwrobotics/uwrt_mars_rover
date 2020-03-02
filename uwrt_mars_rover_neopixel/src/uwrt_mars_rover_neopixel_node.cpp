#include <cstdlib>

#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_control.h"

int main(int argc, char** argv){
    neopixel neopixel_obj(argc, argv, "vcan0");
    neopixel_obj.run();
    return EXIT_SUCCESS;
}