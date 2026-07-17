#include "camera.hpp"

#include <stdexcept>

#include "tools/yaml.hpp"
#include "galaxy/galaxy.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto gain = tools::read<double>(yaml, "gain");
  auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  camera_ = std::make_unique<Galaxy>(exposure_ms, gain, vid_pid);
}
void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
