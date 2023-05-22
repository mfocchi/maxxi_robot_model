#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "robot_model/motionModels.h"

using namespace std::chrono_literals;
typedef double data_t;

#define MILLISECONDS 1000.0

class Unicycle_robot : public rclcpp::Node
{
private:
    UnicycleModelPtr RobotModel;      // model of the robot for the trajectory generation
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer_;
    VectorX_t pose; // [x,y,theta]
    size_t count_; 

    void timer_callback()
    {
		geometry_msgs::msg::TransformStamped msg;
        geometry_msgs::msg::Quaternion q_msg;
        tf2::Quaternion q;

        RobotModel->integrate(pose, 3, &pose); // update robot's position
        // conversion from Yaw-Pitch-Roll to quaternions
        q.setRPY(0, 0,  pose(2));
        q.normalize();
        msg.header.frame_id = "Doretta";
        msg.header.stamp = this->get_clock()->now();


        msg.transform.translation.x = pose(0);
        msg.transform.translation.y = pose(1);
        msg.transform.translation.z = 0.0;
        msg.transform.rotation.x = q.x();
        msg.transform.rotation.y = q.y();
        msg.transform.rotation.z = q.z();
        msg.transform.rotation.w = q.w();
		pub->publish(msg);
    }

    /* when a msg of twist type is published, it is converted into left and right wheel velocities and published back*/
    void vel_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) 
    {
        VectorX_t u_tmp(2);
        u_tmp << msg->twist.linear.x, msg->twist.angular.z;
        this->RobotModel->setU(u_tmp);
    }

public:
    Unicycle_robot(Vector3_t pose_init)
    : Node("unicycle_robot"), count_(0)
    {
		declare_parameter("pub_dt_ms", 50);
		int dt = this->get_parameter("pub_dt_ms").as_int();
        pose = pose_init;   
		pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("/UniBot/pose", 10);
		sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
			"/vel_cmd", 
			10, 
			std::bind(&Unicycle_robot::vel_cmd_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
			std::chrono::milliseconds(dt), 
			std::bind(&Unicycle_robot::timer_callback, this));
        MatrixX_t Q(2,2);       // covariance on the control input
        VectorX_t u_init(2);    // init control input
        Q.setZero();
        u_init.setZero();
        RobotModel = std::make_shared<UnicycleModel>(Q, data_t(dt) / MILLISECONDS, u_init);
    }
};

int main(int argc, char ** argv)
{
    Vector3_t starting_pose;
    starting_pose << 0,0,0;
	rclcpp::init(argc, argv);
	std::shared_ptr<Unicycle_robot> Sim(new Unicycle_robot(starting_pose));

	rclcpp::spin(Sim);

	rclcpp::shutdown();
	printf("Simulator node shutdown\n");

  return 0;
}
