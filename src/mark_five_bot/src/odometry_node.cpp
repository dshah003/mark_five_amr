/**
 * @file odometry_node.cpp
 * @brief Differential drive odometry node for Mark Five AMR (ROS2 Jazzy)
 *
 * This node subscribes to encoder tick counts from the Arduino and computes
 * odometry using differential drive kinematics. It publishes:
 * - nav_msgs/Odometry on /odom topic
 * - TF transform from odom -> base_footprint
 *
 * @author Darshan Shah
 * @date December 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode() : Node("odometry_node")
    {
        // Declare and get parameters
        this->declare_parameter("wheel_base", 0.14);
        this->declare_parameter("ticks_per_meter", 3125.0);
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_footprint");
        this->declare_parameter("publish_tf", true);

        // Covariance parameters
        this->declare_parameter("pose_covariance.x", 0.01);
        this->declare_parameter("pose_covariance.y", 0.01);
        this->declare_parameter("pose_covariance.z", 1.0e6);
        this->declare_parameter("pose_covariance.roll", 1.0e6);
        this->declare_parameter("pose_covariance.pitch", 1.0e6);
        this->declare_parameter("pose_covariance.yaw", 0.03);
        this->declare_parameter("twist_covariance.x", 0.01);
        this->declare_parameter("twist_covariance.y", 1.0e6);
        this->declare_parameter("twist_covariance.z", 1.0e6);
        this->declare_parameter("twist_covariance.roll", 1.0e6);
        this->declare_parameter("twist_covariance.pitch", 1.0e6);
        this->declare_parameter("twist_covariance.yaw", 0.03);

        wheel_base_ = this->get_parameter("wheel_base").as_double();
        ticks_per_meter_ = this->get_parameter("ticks_per_meter").as_double();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // Load covariance values
        pose_cov_x_ = this->get_parameter("pose_covariance.x").as_double();
        pose_cov_y_ = this->get_parameter("pose_covariance.y").as_double();
        pose_cov_z_ = this->get_parameter("pose_covariance.z").as_double();
        pose_cov_roll_ = this->get_parameter("pose_covariance.roll").as_double();
        pose_cov_pitch_ = this->get_parameter("pose_covariance.pitch").as_double();
        pose_cov_yaw_ = this->get_parameter("pose_covariance.yaw").as_double();
        twist_cov_x_ = this->get_parameter("twist_covariance.x").as_double();
        twist_cov_y_ = this->get_parameter("twist_covariance.y").as_double();
        twist_cov_z_ = this->get_parameter("twist_covariance.z").as_double();
        twist_cov_roll_ = this->get_parameter("twist_covariance.roll").as_double();
        twist_cov_pitch_ = this->get_parameter("twist_covariance.pitch").as_double();
        twist_cov_yaw_ = this->get_parameter("twist_covariance.yaw").as_double();

        // Initialize state
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        left_ticks_ = 0;
        right_ticks_ = 0;
        prev_left_ticks_ = 0;
        prev_right_ticks_ = 0;

        left_ticks_received_ = false;
        right_ticks_received_ = false;
        first_reading_ = true;

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);

        // Subscribers
        left_ticks_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/left_ticks", 10,
            std::bind(&OdometryNode::leftTicksCallback, this, std::placeholders::_1));

        right_ticks_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/right_ticks", 10,
            std::bind(&OdometryNode::rightTicksCallback, this, std::placeholders::_1));

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Odometry node initialized");
        RCLCPP_INFO(this->get_logger(), "  wheel_base: %.4f m", wheel_base_);
        RCLCPP_INFO(this->get_logger(), "  ticks_per_meter: %.1f", ticks_per_meter_);
        RCLCPP_INFO(this->get_logger(), "  odom_frame: %s", odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  base_frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish_tf: %s", publish_tf_ ? "true" : "false");
    }

private:
    void leftTicksCallback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        left_ticks_ = msg->data;
        left_ticks_received_ = true;
        tryComputeOdometry();
    }

    void rightTicksCallback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        right_ticks_ = msg->data;
        right_ticks_received_ = true;
        tryComputeOdometry();
    }

    void tryComputeOdometry()
    {
        // Wait until we have both tick values
        if (!left_ticks_received_ || !right_ticks_received_)
        {
            return;
        }

        rclcpp::Time current_time = this->now();

        // On first reading, just store the values
        if (first_reading_)
        {
            prev_left_ticks_ = left_ticks_;
            prev_right_ticks_ = right_ticks_;
            last_time_ = current_time;
            first_reading_ = false;
            RCLCPP_INFO(this->get_logger(), "Odometry: First encoder reading received");
            return;
        }

        // Compute time delta
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0)
        {
            return;
        }

        // Compute tick deltas (handle Int16 overflow)
        int16_t delta_left = left_ticks_ - prev_left_ticks_;
        int16_t delta_right = right_ticks_ - prev_right_ticks_;

        // Convert ticks to distance traveled by each wheel
        double dist_left = static_cast<double>(delta_left) / ticks_per_meter_;
        double dist_right = static_cast<double>(delta_right) / ticks_per_meter_;

        // Differential drive kinematics
        double dist_center = (dist_left + dist_right) / 2.0;
        double delta_theta = (dist_right - dist_left) / wheel_base_;

        // Update pose using midpoint integration
        double avg_theta = theta_ + delta_theta / 2.0;
        x_ += dist_center * cos(avg_theta);
        y_ += dist_center * sin(avg_theta);
        theta_ += delta_theta;

        // Normalize theta to [-pi, pi]
        theta_ = normalizeAngle(theta_);

        // Compute velocities
        double v_linear = dist_center / dt;
        double v_angular = delta_theta / dt;

        // Publish odometry message
        publishOdometry(current_time, v_linear, v_angular);

        // Publish TF
        if (publish_tf_)
        {
            publishTransform(current_time);
        }

        // Update previous values
        prev_left_ticks_ = left_ticks_;
        prev_right_ticks_ = right_ticks_;
        last_time_ = current_time;

        // Reset received flags
        left_ticks_received_ = false;
        right_ticks_received_ = false;
    }

    void publishOdometry(const rclcpp::Time& timestamp, double v_linear, double v_angular)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // Set position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Set orientation (quaternion from yaw)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        // Set velocity
        odom.twist.twist.linear.x = v_linear;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = v_angular;

        // Set covariance (diagonal, from parameters)
        // Position covariance
        odom.pose.covariance[0] = pose_cov_x_;     // x
        odom.pose.covariance[7] = pose_cov_y_;     // y
        odom.pose.covariance[14] = pose_cov_z_;    // z (not measured)
        odom.pose.covariance[21] = pose_cov_roll_; // roll (not measured)
        odom.pose.covariance[28] = pose_cov_pitch_;// pitch (not measured)
        odom.pose.covariance[35] = pose_cov_yaw_;  // yaw

        // Velocity covariance
        odom.twist.covariance[0] = twist_cov_x_;   // linear x
        odom.twist.covariance[7] = twist_cov_y_;   // linear y (not measured)
        odom.twist.covariance[14] = twist_cov_z_;  // linear z (not measured)
        odom.twist.covariance[21] = twist_cov_roll_;  // angular x (not measured)
        odom.twist.covariance[28] = twist_cov_pitch_; // angular y (not measured)
        odom.twist.covariance[35] = twist_cov_yaw_;   // angular z

        odom_pub_->publish(odom);
    }

    void publishTransform(const rclcpp::Time& timestamp)
    {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = timestamp;
        odom_tf.header.frame_id = odom_frame_;
        odom_tf.child_frame_id = base_frame_;

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, theta_);
        odom_tf.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_tf);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr left_ticks_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr right_ticks_sub_;

    // Parameters
    double wheel_base_;
    double ticks_per_meter_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    // Covariance parameters
    double pose_cov_x_, pose_cov_y_, pose_cov_z_;
    double pose_cov_roll_, pose_cov_pitch_, pose_cov_yaw_;
    double twist_cov_x_, twist_cov_y_, twist_cov_z_;
    double twist_cov_roll_, twist_cov_pitch_, twist_cov_yaw_;

    // State
    double x_, y_, theta_;
    int16_t left_ticks_, right_ticks_;
    int16_t prev_left_ticks_, prev_right_ticks_;
    bool left_ticks_received_, right_ticks_received_;
    bool first_reading_;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryNode>());
    rclcpp::shutdown();
    return 0;
}
