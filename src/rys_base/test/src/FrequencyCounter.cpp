#include "FrequencyCounter.hpp"
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

FrequencyCounter::FrequencyCounter() {

	this->previous = std::chrono::high_resolution_clock::now();
	this->timeNow = std::chrono::high_resolution_clock::now();

	this->numOfRuns = 0;
	this->frequency = 0.0;

	std::cout << "[Frequency Counter] Ready\n";
}

void FrequencyCounter::count() {
	this->numOfRuns++;
	// std::this_thread::sleep_for(2ms);
	// rclcpp::sleep_for(2ms);
	if (this->numOfRuns > 999) {
		this->previous = this->timeNow;
		this->timeNow = std::chrono::high_resolution_clock::now();
		auto loopTimeSpan = std::chrono::duration_cast<std::chrono::duration<float>>(this->timeNow - this->previous);
		float loopTime = loopTimeSpan.count();
		this->frequency = this->numOfRuns/loopTime;
		std::cout <<"[FrequencyCounter] Frequency " << this->frequency << "Hz after " << numOfRuns << " messages." << std::endl;
		this->numOfRuns = 0;
	}
}
