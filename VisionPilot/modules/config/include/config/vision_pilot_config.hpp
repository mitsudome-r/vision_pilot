#pragma once

#include <string>
#include <engine/onnx_engine.hpp>
#include <models/inference.hpp>
#include <common/utils.hpp>

namespace vpe = visionpilot::engine;
namespace vpm = visionpilot::models;

enum class SourceMode { Ros2 = 0, V4l2 = 1, Video = 2 };

struct SourceConfig {
    SourceMode  mode         = SourceMode::Video;
    std::string input_video;
    std::string input_vehicle_speed;
    std::string dataset;
    bool        video_realtime = true;
    bool        video_loop     = false;
    std::string input_camera_topic   = "/camera/image";
    std::string v4l2_device  = "/dev/video0";
    int         v4l2_fps     = 10;
};

// struct PipelineConfig {
//     bool initial_inference_check = true;
// };

struct Config {
    vpe::Config engine;
    vpm::Config inference;
    SourceConfig      source;
    // PipelineConfig    pipeline;
    // Print per-frame fusion debug logs
    bool              fusion_debug   = false;
    // Directory with wheel_white.png / wheel_green.png for steering HUD
    std::string       wheel_dir;

    std::string vehicle_speed_topic;
    std::string vehicle_steering_topic;
    std::string vehicle_acceleration_topic;

    double speed_limit;
    double L = 0.0;   // Wheelbase (m). 0 = not set in the .conf (ROS 2: taken from vehicle_info wheel_base)

    // ── Vehicle / planner tuning (defaults = original passenger-car values) ──
    double max_steer_angle     = 0.442;   // tyre-angle limit (rad)
    double understeer_gradient = 0.0015;  // K_us (rad / (m/s^2))
    double lon_max_accel       = 1.5;     // IDM a  (m/s^2)
    double lon_comfort_decel   = 3.0;     // IDM b  (m/s^2)
    double lon_max_decel       = 5.0;     // hard bound on the commanded deceleration (m/s^2)
    double lon_time_headway    = 1.5;     // IDM T  (s)
    double lon_min_gap         = 2.0;     // IDM s0 (m)
    double lon_road_friction   = 0.2;     // mu used for curve speed limit

    // ── ROS 2 middleware selection (only used with ENABLE_ROS2_INTERFACE) ──
    // "generic"  : std_msgs/Float64 topics (original VisionPilot interface)
    // "autoware" : autoware_vehicle_msgs/VelocityReport in,
    //              autoware_control_msgs/Control out
    std::string ros2_vehicle_interface = "generic";
    // "reliable" (original) or "sensor_data" (best effort, matches most camera drivers)
    std::string ros2_camera_qos = "reliable";
    // "raw"        : sensor_msgs/Image on source.input_camera_topic
    // "compressed" : sensor_msgs/CompressedImage (JPEG / PNG) on source.input_camera_topic,
    //                decoded in-process (no image_transport republish node needed)
    // Can be overridden on the command line with --camera-transport raw|compressed.
    std::string ros2_camera_transport = "raw";
    std::string autoware_velocity_topic    = "/vehicle/status/velocity_status";
    std::string autoware_control_cmd_topic = "/vision_pilot/control_cmd";
    std::string autoware_frame_id          = "base_link";

    bool visualization_on = false;
    bool webrtc_on = false;
    int webrtc_port;

    // Rerun logging: when enabled, per-frame data is streamed directly to an
    // .rrd recording ready to open in Rerun viewer.
    bool        rrd_on  = false;
    std::string rrd_log = "visionpilot.rrd";
};


// Load from key=value .conf file. Expands ~ to $HOME.
// Throws std::runtime_error on missing or invalid config.
Config load_vision_pilot_config();

// Resolve config path from --config <path>, VISIONPILOT_CONFIG env var,
// or default candidates. Returns empty string if nothing found.
std::string resolve_vision_pilot_config_path(int argc, char** argv);

SourceMode parse_source_mode(const std::string& value);

// Short label for debug overlay (video / device path / ROS topic).
std::string source_label(const SourceConfig& source);
