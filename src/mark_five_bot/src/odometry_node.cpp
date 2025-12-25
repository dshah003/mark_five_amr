/**
 * @file odometry_node.cpp
 * @brief Differential drive odometry node for Mark Five AMR
 *
 * This node subscribes to encoder tick counts from the Arduino and computes
 * odometry using differential drive kinematics. It publishes:
 * - nav_msgs/Odometry on /odom topic
 * - TF transform from odom -> base_footprint
 *
 * @author Darshan Shah
 * @date December 2025
 */

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

class OdometryNode
{
public:
    OdometryNode() : nh_("~")
    {
        // Load parameters with defaults from Arduino code
        nh_.param<double>("wheel_base", wheel_base_, 0.14);           // meters
        nh_.param<double>("ticks_per_meter", ticks_per_meter_, 3125.0);
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_footprint");
        nh_.param<bool>("publish_tf", publish_tf_, true);

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

        // Publishers
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);

        // Subscribers
        left_ticks_sub_ = nh_.subscribe("/left_ticks", 10,
                                         &OdometryNode::leftTicksCallback, this);
        right_ticks_sub_ = nh_.subscribe("/right_ticks", 10,
                                          &OdometryNode::rightTicksCallback, this);

        last_time_ = ros::Time::now();

        ROS_INFO("Odometry node initialized");
        ROS_INFO("  wheel_base: %.4f m", wheel_base_);
        ROS_INFO("  ticks_per_meter: %.1f", ticks_per_meter_);
        ROS_INFO("  odom_frame: %s", odom_frame_.c_str());
        ROS_INFO("  base_frame: %s", base_frame_.c_str());
        ROS_INFO("  publish_tf: %s", publish_tf_ ? "true" : "false");
    }

    void leftTicksCallback(const std_msgs::Int16::ConstPtr& msg)
    {
        left_ticks_ = msg->data;
        left_ticks_received_ = true;
        tryComputeOdometry();
    }

    void rightTicksCallback(const std_msgs::Int16::ConstPtr& msg)
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

        ros::Time current_time = ros::Time::now();

        // On first reading, just store the values
        if (first_reading_)
        {
            prev_left_ticks_ = left_ticks_;
            prev_right_ticks_ = right_ticks_;
            last_time_ = current_time;
            first_reading_ = false;
            ROS_INFO("Odometry: First encoder reading received");
            return;
        }

        // Compute time delta
        double dt = (current_time - last_time_).toSec();
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

    void publishOdometry(const ros::Time& timestamp, double v_linear, double v_angular)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // Set position
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Set orientation (quaternion from yaw)
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
        odom.pose.pose.orientation = odom_quat;

        // Set velocity
        odom.twist.twist.linear.x = v_linear;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = v_angular;

        // Set covariance (diagonal, rough estimates)
        // Position covariance
        odom.pose.covariance[0] = 0.01;   // x
        odom.pose.covariance[7] = 0.01;   // y
        odom.pose.covariance[14] = 1e6;   // z (not measured)
        odom.pose.covariance[21] = 1e6;   // roll (not measured)
        odom.pose.covariance[28] = 1e6;   // pitch (not measured)
        odom.pose.covariance[35] = 0.03;  // yaw

        // Velocity covariance
        odom.twist.covariance[0] = 0.01;  // linear x
        odom.twist.covariance[7] = 1e6;   // linear y (not measured)
        odom.twist.covariance[14] = 1e6;  // linear z (not measured)
        odom.twist.covariance[21] = 1e6;  // angular x (not measured)
        odom.twist.covariance[28] = 1e6;  // angular y (not measured)
        odom.twist.covariance[35] = 0.03; // angular z

        odom_pub_.publish(odom);
    }

    void publishTransform(const ros::Time& timestamp)
    {
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = timestamp;
        odom_tf.header.frame_id = odom_frame_;
        odom_tf.child_frame_id = base_frame_;

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
        odom_tf.transform.rotation = odom_quat;

        tf_broadcaster_.sendTransform(odom_tf);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

private:
    ros::NodeHandle nh_;

    // Publishers and subscribers
    ros::Publisher odom_pub_;
    ros::Subscriber left_ticks_sub_;
    ros::Subscriber right_ticks_sub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Parameters
    double wheel_base_;
    double ticks_per_meter_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    // State
    double x_, y_, theta_;
    int16_t left_ticks_, right_ticks_;
    int16_t prev_left_ticks_, prev_right_ticks_;
    bool left_ticks_received_, right_ticks_received_;
    bool first_reading_;
    ros::Time last_time_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_node");

    OdometryNode odometry_node;

    ros::spin();

    return 0;
}
