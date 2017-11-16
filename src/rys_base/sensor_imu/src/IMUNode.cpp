#include "IMUNode.hpp"

#include <iostream>
#include <memory>

IMUNode::IMUNode(
	const std::string & robotName,
	const std::string & nodeName,
	const std::chrono::milliseconds loopDuration,
	const std::chrono::milliseconds calibrationDuration
) : rclcpp::Node(nodeName, robotName, true) {
	this->calibration = false;
	this->calibrationValuesSum = 0;
	this->calibrationIterations = 0;
	this->calibrationDuration = calibrationDuration;
	this->calibrationEndTime = std::chrono::high_resolution_clock::now();

	std::cout << "[IMU] Initializing IMU...\n";
	this->imu = new IMU();
	this->imu->initialize(1000/loopDuration.count());
	this->imu->resetFIFO();
	std::cout << "[IMU] IMU initialized\n";

	this->imuPublisher = this->create_publisher<rys_interfaces::msg::ImuRollRotation>("/" + robotName + "/sensor/imu", rmw_qos_profile_sensor_data);
	this->calibrationSubscription = this->create_subscription<std_msgs::msg::Empty>("/" + robotName + "/control/imu/calibrate", std::bind(&IMUNode::imuCalibrateCallback, this, std::placeholders::_1));
	this->timer = this->create_wall_timer(loopDuration, std::bind(&IMUNode::imuReadAndPublishData, this));
	std::cout << "[IMU] Node ready\n";
}

IMUNode::~IMUNode() {
	delete this->imu;
}

void IMUNode::imuCalibrateCallback(const std_msgs::msg::Empty::SharedPtr message) {
	// Prevent unused parameter warning
	(void)message;

	std::cout << "[IMU] Calibration: collecting data (" << this->calibrationDuration.count() << "ms)...\n";
	this->calibrationEndTime = std::chrono::high_resolution_clock::now() + this->calibrationDuration;
	this->calibrationValuesSum = 0;
	this->calibrationIterations = 0;
	this->calibration = true;
}

void IMUNode::imuReadAndPublishData() {
	auto message = std::make_shared<rys_interfaces::msg::ImuRollRotation>();

	message->header.stamp = rclcpp::Time::now();
	message->header.frame_id = "MPU6050";

	float roll = 0;
	try {
		roll = this->imu->getRoll();
		message->roll = roll;

		float rotationX, rotationY, rotationZ;
		this->imu->getGyro(&rotationX, &rotationY, &rotationZ);
		message->rotation_x = rotationX;
		message->rotation_y = rotationY;
		message->rotation_z = rotationZ;
	} catch (std::string & error) {
		std::cout << "[IMU] Error getting IMU reading: " << error << std::endl;
		return;
	}

	this->imuPublisher->publish(message);

	if (this->calibration) {
		this->calibrationValuesSum += roll;
		this->calibrationIterations++;
		if (std::chrono::high_resolution_clock::now() >= this->calibrationEndTime) {
			this->calibration = false;
			float averageRoll = this->calibrationValuesSum / this->calibrationIterations;
			this->imu->setOffsets(0, 0, averageRoll);
			std::cout << "[IMU] Calibration: data collected, average offset: " << averageRoll << std::endl;
		}
	}
}
