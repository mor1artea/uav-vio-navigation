// Configuration: config/vio.yaml, optionally with override files on top (for example
// config/replay_euroc.yaml), and the settings structs built from it.
#pragma once

#include "vio/filter.h"
#include "vio/frontend.h"
#include "vio/msckf.h"

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace vio {

// Loads the files in order; a later file overrides single keys of an earlier one, at any depth.
YAML::Node load_config(const std::vector<std::string>& files);

// Filter settings. The IMU noise comes from the config, or from the session's imu0/sensor.yaml if
// filter.imu_noise_from_session is true (datasets).
FilterConfig filter_config(const YAML::Node& cfg, const ImuNoise& session_noise);

MsckfConfig msckf_config(const YAML::Node& cfg);

// Tracker and sky mask settings (sections tracker: and sky_mask:).
TrackerConfig tracker_config(const YAML::Node& cfg);

}  // namespace vio
