#ifndef _MOTORS_CONTROLLER_NODE
#define _MOTORS_CONTROLLER_NODE

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rys_interfaces/msg/imu_roll_rotation.hpp"
#include "rys_interfaces/msg/steering.hpp"
#include "rys_interfaces/srv/set_regulator_settings.hpp"
#include "rys_interfaces/srv/get_regulator_settings.hpp"

#include "MotorsController.hpp"

class MotorsControllerNode : public rclcpp::Node {
	private:
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

		MotorsController * motorsController;

		rclcpp::TimerBase::SharedPtr loopTimer;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableSubscriber;
		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr balancingModeSubscriber;
		rclcpp::Subscription<rys_interfaces::msg::Steering>::SharedPtr steeringSubscriber;
		rclcpp::Subscription<rys_interfaces::msg::ImuRollRotation>::SharedPtr imuSubscriber;

		rclcpp::service::Service<rys_interfaces::srv::SetRegulatorSettings>::SharedPtr setRegulatorSettingsServer;
		rclcpp::service::Service<rys_interfaces::srv::GetRegulatorSettings>::SharedPtr getRegulatorSettingsServer;

		void motorsRunTimed(const float leftSpeed, const float rightSpeed, const int microstep, const int milliseconds);
		void standUp();

		void enableMessageCallback(const std_msgs::msg::Bool::SharedPtr message);
		void imuMessageCallback(const rys_interfaces::msg::ImuRollRotation::SharedPtr message);
		void setBalancingMode(const std_msgs::msg::Bool::SharedPtr message);
		void setSteering(const rys_interfaces::msg::Steering::SharedPtr message);
		void setRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::SetRegulatorSettings::Response> response);
		void getRegulatorSettingsCallback(const std::shared_ptr<rmw_request_id_t> requestHeader, const std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Request> request, std::shared_ptr<rys_interfaces::srv::GetRegulatorSettings::Response> response);

		void runLoop();
	public:
		MotorsControllerNode(const char * nodeName, std::chrono::milliseconds rate);
		~MotorsControllerNode();
};

#endif
