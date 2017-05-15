#ifndef _Sonars_H_
#define _Sonars_H_

#include <cstdint>

#define MAX_BUFFER_SIZE 512
#define DEVICE_FILE_NAME "/dev/rpmsg_pru30"

struct DistanceFrame {
	uint16_t front;
	uint16_t back;
	uint16_t top;
};

class Sonars {
	public:
		Sonars();
		~Sonars();
		void getDistances(unsigned int * front, unsigned int * back, unsigned int * top);
};

#endif
