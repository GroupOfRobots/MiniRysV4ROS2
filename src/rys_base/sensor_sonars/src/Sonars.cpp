#include "Sonars.hpp"

#include <fstream>

Sonars::Sonars() {}

Sonars::~Sonars() {}

void Sonars::getDistances(unsigned int * front, unsigned int * back, unsigned int * top) {
	std::fstream file;

	file.open(DEVICE_FILE_NAME, std::fstream::out | std::fstream::binary);
	if (!file.is_open() || !file.good()) {
		file.close();
		std::string error("Failed opening file: ");
		error += std::string(DEVICE_FILE_NAME);
		throw(error);
	}
	file.write("h", 13);
	file.close();

	file.open(DEVICE_FILE_NAME, std::fstream::in | std::fstream::binary);
	if (!file.is_open() || !file.good()) {
		file.close();
		std::string error("Failed opening file: ");
		error += std::string(DEVICE_FILE_NAME);
		throw(error);
	}

	DistanceFrame frame;
	file.read((char *)(&frame.front), sizeof(frame.front));
	file.read((char *)(&frame.back), sizeof(frame.back));
	file.read((char *)(&frame.top), sizeof(frame.top));
	file.close();

	*front = frame.front;
	*back = frame.back;
	*top = frame.top;
}
