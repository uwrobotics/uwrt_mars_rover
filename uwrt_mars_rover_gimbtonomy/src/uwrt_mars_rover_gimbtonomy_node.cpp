#include <cstdlib>

#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_can.h"
#include "uwrt_mars_rover_gimbtonomy/uwrt_mars_rover_gimbtonomy_neopixel.h"

int main(int argc, char** argv){
    neopixel neopixel_obj(argc, argv);
    neopixel_obj.run();
    return EXIT_SUCCESS;
}