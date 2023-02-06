#pragma once

#include <cmath>
#include <string>

namespace dinolite
{
struct CameraContext
{
  // note: fps is not exact, camera sends less frames when image are more different
  const bool file_ = false;
  const double fps_ = 60;                        // dinolite is capped at 30 fps
  const int delay_ms_ = (int)(1 / fps_ * 1000);  // duration takes int
  const std::string filename_ = "";
  const int index_ = 0;  // 0 is default index of webcam
  const int width_ = 640;
  const int height_ = 480;
  const std::string camera_info_path_ = "install/dinolite/share/dinolite/config/info.ini";
  const std::string camera_frame_id_ = "camera_frame";
};

}  // namespace dinolite
