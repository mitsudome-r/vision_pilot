#ifndef VISIONPILOT_AUTOWARE_VEHICLE_ROS2_INTERFACE_HPP
#define VISIONPILOT_AUTOWARE_VEHICLE_ROS2_INTERFACE_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <vehicle_interface/vehicle_interface.hpp>

// ── Topics ────────────────────────────────────────────────────────────────────
//
//  Subscribe  <velocity_topic>     autoware_vehicle_msgs/VelocityReport
//                                  (default: /vehicle/status/velocity_status)
//
//  Publish    <control_cmd_topic>  autoware_control_msgs/Control
//                                  lateral.steering_tire_angle   = tyre angle (rad, left +)
//                                  longitudinal.acceleration     = acceleration (m/s^2)
//                                  longitudinal.velocity         = max(0, v + a * T_preview)
//
//  Typical remaps in Autoware:
//    shadow mode  : /vision_pilot/control_cmd                (monitor only)
//    control mode : /control/trajectory_follower/control_cmd (feeds vehicle_cmd_gate)
//
// ─────────────────────────────────────────────────────────────────────────────

class AutowareVehicleRos2Interface : public VehicleInterface
{
public:
    struct Config {
        std::string velocity_topic    = "/vehicle/status/velocity_status";
        std::string control_cmd_topic = "/vision_pilot/control_cmd";
        // Look-ahead used to derive longitudinal.velocity from the acceleration command.
        double velocity_preview_time  = 1.0;   // s
        // Velocity reports older than this are considered stale (speed treated as unknown).
        double velocity_timeout       = 0.5;   // s
        std::string node_name         = "vision_pilot_autoware_vehicle_interface";
    };

    explicit AutowareVehicleRos2Interface(const Config& config);
    ~AutowareVehicleRos2Interface() override;

    // Latest ego longitudinal speed (m/s). Returns 0 and warns when stale.
    // write() publishes nothing while the speed is stale.
    double read() override;

    // Publish tyre angle (rad) and acceleration (m/s^2) as autoware_control_msgs/Control.
    void write(double steering, double acceleration) override;

private:
    double velocity_age() const;  // seconds since the last VelocityReport

    Config config_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_sub_;
    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_pub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread spin_thread_;

    std::atomic<double> speed_{0.0};
    std::atomic<int64_t> last_velocity_ns_{0};
};

#endif  // VISIONPILOT_AUTOWARE_VEHICLE_ROS2_INTERFACE_HPP
