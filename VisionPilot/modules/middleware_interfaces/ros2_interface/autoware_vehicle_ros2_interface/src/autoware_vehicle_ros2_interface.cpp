#include <autoware_vehicle_ros2_interface/autoware_vehicle_ros2_interface.hpp>

#include <algorithm>

AutowareVehicleRos2Interface::AutowareVehicleRos2Interface(const Config& config)
    : config_(config)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp::Node>(config_.node_name);

    // VelocityReport is published with the default (reliable, depth 1) QoS by Autoware's
    // vehicle interface. A best-effort subscriber is compatible with both reliable and
    // best-effort publishers and we only need the freshest sample.
    const auto vel_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    velocity_sub_ = node_->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
        config_.velocity_topic, vel_qos,
        [this](const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg)
        {
            speed_.store(static_cast<double>(msg->longitudinal_velocity), std::memory_order_relaxed);
            last_velocity_ns_.store(node_->now().nanoseconds(), std::memory_order_relaxed);
        });

    // Same QoS as Autoware's trajectory follower output (reliable, depth 1).
    control_pub_ = node_->create_publisher<autoware_control_msgs::msg::Control>(
        config_.control_cmd_topic, rclcpp::QoS(rclcpp::KeepLast(1)));

    RCLCPP_INFO(node_->get_logger(), "AutowareVehicleRos2Interface ready");
    RCLCPP_INFO(node_->get_logger(), "  sub  %s (VelocityReport)", velocity_sub_->get_topic_name());
    RCLCPP_INFO(node_->get_logger(), "  pub  %s (Control)", control_pub_->get_topic_name());

    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
}

AutowareVehicleRos2Interface::~AutowareVehicleRos2Interface()
{
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
}

double AutowareVehicleRos2Interface::velocity_age() const
{
    const int64_t last = last_velocity_ns_.load(std::memory_order_relaxed);
    return last == 0 ? 1e9 : (node_->now().nanoseconds() - last) * 1e-9;
}

double AutowareVehicleRos2Interface::read()
{
    const double age = velocity_age();
    if (age > config_.velocity_timeout)
    {
        if (last_velocity_ns_.load(std::memory_order_relaxed) == 0)
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "No velocity report received yet on %s. Using 0 m/s.",
                                 velocity_sub_->get_topic_name());
        else
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Velocity report on %s is stale (%.2f s). Using 0 m/s.",
                                 velocity_sub_->get_topic_name(), age);
        return 0.0;
    }
    return speed_.load(std::memory_order_relaxed);
}

void AutowareVehicleRos2Interface::write(const double steering, const double acceleration)
{
    // Without a fresh ego speed the plan is meaningless (the IDM would command free-road
    // acceleration). Publish nothing so that Autoware's command timeouts take over instead
    // of forwarding an unsafe command.
    if (velocity_age() > config_.velocity_timeout)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "Suppressing control command: no fresh velocity on %s",
                             velocity_sub_->get_topic_name());
        return;
    }

    const auto stamp = node_->now();
    const double v = std::max(0.0, speed_.load(std::memory_order_relaxed));

    autoware_control_msgs::msg::Control cmd;
    cmd.stamp = stamp;
    cmd.control_time = stamp;

    cmd.lateral.stamp = stamp;
    cmd.lateral.control_time = stamp;
    cmd.lateral.steering_tire_angle = static_cast<float>(steering);
    cmd.lateral.steering_tire_rotation_rate = 0.0f;
    cmd.lateral.is_defined_steering_tire_rotation_rate = false;

    cmd.longitudinal.stamp = stamp;
    cmd.longitudinal.control_time = stamp;
    cmd.longitudinal.acceleration = static_cast<float>(acceleration);
    cmd.longitudinal.velocity =
        static_cast<float>(std::max(0.0, v + acceleration * config_.velocity_preview_time));
    cmd.longitudinal.jerk = 0.0f;
    cmd.longitudinal.is_defined_acceleration = true;
    cmd.longitudinal.is_defined_jerk = false;

    control_pub_->publish(cmd);
}
