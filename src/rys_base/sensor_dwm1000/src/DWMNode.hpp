#ifndef _DWM_NODE_HPP_
#define _DWM_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rys_interfaces/msg/location_distances.hpp"

#include "DWM.hpp"

class DWMNode : public rclcpp::Node {
	private:
		DWM * dwm;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Publisher<rys_interfaces::msg::LocationDistances>::SharedPtr publisher;

		void publishData();
	public:
		DWMNode(
			const std::string & robotName,
			const std::string & nodeName,
			const bool useIPC,
			std::chrono::milliseconds rate
		);
		~DWMNode();
};

#endif
