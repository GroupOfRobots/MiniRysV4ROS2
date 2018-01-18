#ifndef _VL53L0X_NODE_HPP
#define _VL53L0X_NODE_HPP

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/ranges.hpp"

#include "VL53L0X.hpp"

class RangesNode : public rclcpp::Node {
	private:
		VL53L0X * sensors[5];
		bool sensorInitialized[5];

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<rys_interfaces::msg::Ranges>::SharedPtr publisher;

		int readSensor(int sensorIndex);
		void publishData();
	public:
		RangesNode(
			const std::string & robotName,
			const std::string & nodeName,
			const std::chrono::milliseconds loopDuration,
			const uint8_t pins[5],
			const uint8_t addresses[5]
		);
		~RangesNode();
};

#endif
