#pragma once


#include <cmath>
#include <string>

namespace dinolite {

    struct CameraContext {
        const bool file_ = false;
        const int fps_ = 30;
	const double delay_ms_double_ = 1 / fps_ * 1000;
	const int delay_ms_ = (int) delay_ms_double_; // duration takes int
        const std::string filename_ = "";
        const int index_ = 0; // 0 is default index of webcam
        const int width_ = 640;
        const int height_ = 480;
        const std::string camera_info_path_ = "install/dinolite/share/dinolite/config/info.ini";
        const std::string camera_frame_id_ = "camera_frame";
    };

}
