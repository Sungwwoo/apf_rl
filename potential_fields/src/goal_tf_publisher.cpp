#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace potential_fields{
    class GoalTFPublisher : public rclcpp::Node {
    public:
        GoalTFPublisher()
        : Node("goal_tf_publisher")
        {
            // Initialize the transform broadcaster
            tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10,
            std::bind(&GoalTFPublisher::cbNav2Goal, this, std::placeholders::_1));

            // Timer to publish the TF at 10 Hz
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&GoalTFPublisher::sendGoalTF, this));
        }

    private:
        void cbNav2Goal(const geometry_msgs::msg::PoseStamped& msg){
            RCLCPP_INFO(this->get_logger(), "Got new goal pose");
            trans.x = msg.pose.position.x;
            trans.y = msg.pose.position.y;
            trans.z = msg.pose.position.z;

            quat = msg.pose.orientation;

            send = true;
        }

        void sendGoalTF(){
            if (!send){
                // cnt ++;
                // if (cnt == 20){
                //     RCLCPP_INFO(this->get_logger(), "Waiting for goal point to be received...");
                //     cnt = 0;
                // }
                return;
            }

            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "goal";

            t.transform.translation = trans;
            t.transform.rotation = quat;

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
            
            // TODO stop sending transform if robot is in goal location

        }

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::Vector3 trans;
        geometry_msgs::msg::Quaternion quat;
        bool send = false;
        int cnt = 0;
    };
}
int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<potential_fields::GoalTFPublisher>());
    rclcpp::shutdown();
    return 0;
}