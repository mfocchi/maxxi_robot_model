#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


using namespace std::chrono_literals;
typedef double data_t;

class ModelConverter_unicycle_to_differential : public rclcpp::Node
{
public:
    ModelConverter_unicycle_to_differential()
    : Node("conv_uni_2_diff"), count_(0)
    {
		declare_parameter("wheels_distance_m", 0.7);
		declare_parameter("wheel_radius_m", 0.07);
		d = this->get_parameter("wheels_distance_m").as_double();
		r = this->get_parameter("wheel_radius_m").as_double();

		pub = this->create_publisher<sensor_msgs::msg::JointState>("/command", 10);
		sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
			"/vel_cmd", 
			10, 
			std::bind(&ModelConverter_unicycle_to_differential::vel_cmd_callback, this, std::placeholders::_1));
    }

private:
/* when a msg of twist type is published, it is converted into left and right wheel velocities and published back*/
    void vel_cmd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) 
    {
		data_t v = msg->twist.linear.x;
		data_t omega = msg->twist.angular.z;

		data_t wheel_vel_left  = v - omega * d * 0.5;
		data_t wheel_vel_right = v + omega * d * 0.5;
		
		sensor_msgs::msg::JointState msg_sprocket_speed;
		
		std::vector<data_t> sprockets_speed = {wheel_vel_left  / r, wheel_vel_right / r};
		msg_sprocket_speed.name = {"left_sprocket", "right_sprocket"};
		msg_sprocket_speed.velocity = sprockets_speed;

		pub->publish(msg_sprocket_speed);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub;
    size_t count_;
	data_t r;
	data_t d;
 
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	std::shared_ptr<ModelConverter_unicycle_to_differential> ModelConverter(new ModelConverter_unicycle_to_differential());

	rclcpp::spin(ModelConverter);

	rclcpp::shutdown();
	printf("Model converter (unicycle to differential) node shutdown\n");

  return 0;
}
