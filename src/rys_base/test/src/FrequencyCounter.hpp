#include <chrono>
#include <string>

class FrequencyCounter {
	private:
		std::chrono::time_point<std::chrono::high_resolution_clock> previous;
		std::chrono::time_point<std::chrono::high_resolution_clock> timeNow;

		int numOfRuns;
		float frequency;

	public:
		void count();
		FrequencyCounter();
};
