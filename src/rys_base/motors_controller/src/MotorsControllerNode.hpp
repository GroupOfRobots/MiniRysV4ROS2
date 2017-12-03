#ifndef _MOTORS_CONTROLLER_NODE
#define _MOTORS_CONTROLLER_NODE

#include <chrono>
#include <string>

#include <kdl/frames.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rys_interfaces/msg/imu_roll_rotation.hpp"
#include "rys_interfaces/msg/steering.hpp"
#include "rys_interfaces/srv/set_regulator_settings.hpp"
#include "rys_interfaces/srv/get_regulator_settings.hpp"

#include "MotorsController.hpp"

class MotorsControllerNode : public rclcpp::Node {
	private:
		const float wheelRadius;
		const float baseWidth;

		bool enabled;
		bool balancing;
		std::chrono::milliseconds enableTimeout;
		std::chrono::time_point<std::chrono::high_resolution_clock> enableTimerEnd;
		std::chrono::time_point<std::chrono::high_resolution_clock> previous;
		std::chrono::time_point<std::chrono::high_resolution_clock> now;

		float roll;
		float rollPrevious;
		float rotationX;

		float rotation;
		float throttle;
		unsigned char steeringPrecision;

		builtin_interfaces::msg::Time previousOdometryTime;
		KDL::Frame currentOdometryFrame;

		MotorsController * motorsController;

		rclcpp::TimerBase::SharedPtr loopTimer;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motorsEnableSubscriber;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr balancingEnableSubscriber;
		rclcpp::Subscription<rys_interfaces::msg::Steering>::SharedPtr steeringSubscriber;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscriber;

		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

		rclcpp::service::Service<rys_interfaces::srv::SetRegulatorSettings>::SharedPtr setRegulatorSettingsServer;
		rclcpp::service::Service<rys_interfaces::srv::GetRegulatorSettings>::SharedPtr getRegulatorSettingsServer;

		void motorsRunTimed(const float leftSpeed, const float rightSpeed, const int microstep, const int milliseconds);
		void standUp();

		void enableMessageCallback(const std_msgs::msg::Bool::SharedPtr message);
		void setBalancingMode(const std_msgs::msg::Bool::SharedPtr message);
		void imuMessageCallback(const sensor_msgs::msg::Imu::SharedPtr message);
		void setSteering(const rys_interfaces::msg::Steering::SharedPtr message);
		void setRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Response> response);
		void getRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Response> response);

		void runLoop();
	public:
		MotorsControllerNode(
			const std::string & robotName,
			const std::string & nodeName,
			std::chrono::milliseconds rate,
			float wheelRadius,
			float baseWidth
		);
		~MotorsControllerNode();
};

#endif
