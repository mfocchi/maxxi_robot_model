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

class UnicycleRobot : public rclcpp::Node
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

        RobotModel->integrate(pose, 3, &pose, euler); // update robot's position
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
    UnicycleRobot()
    : Node("UnicycleRobot"), count_(0)
    {
		declare_parameter("pub_dt_ms", 50);
        declare_parameter("x0_m", 0.0);
        declare_parameter("y0_m", 0.0);
        declare_parameter("theta0_rad", 0.0);
		int dt = this->get_parameter("pub_dt_ms").as_int();
		data_t x0     = this->get_parameter("x0_m").as_double();
		data_t y0     = this->get_parameter("y0_m").as_double();
		data_t theta0 = this->get_parameter("theta0_rad").as_double();
        Vector3_t pose0;
        pose0 << x0, y0, theta0;
        pose = pose0;   
		
        pub = this->create_publisher<geometry_msgs::msg::TransformStamped>("/tractor/ground_truth/pose", 10);
		sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
			"/vel_cmd", 
			10, 
			std::bind(&UnicycleRobot::vel_cmd_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
			std::chrono::milliseconds(dt), 
			std::bind(&UnicycleRobot::timer_callback, this));
        MatrixX_t Q(2,2);       // covariance on the control input
        VectorX_t u_init(2);    // init control input
        Q.setZero();
        u_init.setZero();
        std::map<std::string, data_t> params{{"dt", data_t(dt)}};
        RobotModel = std::make_shared<UnicycleModel>(Q, u_init, params);
    }
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<UnicycleRobot> Sim(new UnicycleRobot());

	rclcpp::spin(Sim);

	rclcpp::shutdown();
	printf("Simulator node shutdown\n");

  return 0;
}
