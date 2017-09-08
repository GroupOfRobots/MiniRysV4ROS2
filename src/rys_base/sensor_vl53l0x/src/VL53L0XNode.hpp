#ifndef _VL53L0X_NODE_HPP
#define _VL53L0X_NODE_HPP

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/ranges.hpp"

#include "VL53L0X.hpp"

class VL53L0XNode : public rclcpp::Node {
	private:
		VL53L0X * sensors[5];
		bool sensorInitialized[5];

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<rys_interfaces::msg::Ranges>::SharedPtr publisher;

		int readSensor(int sensorIndex);
		void sensorsReadAndPublishData();
	public:
		VL53L0XNode(
			const char * nodeName,
			const char * publishTopicName,
			const std::chrono::milliseconds loopDuration,
			const uint8_t pins[5],
			const uint8_t addresses[5]
		);
		~VL53L0XNode();
};

#endif
