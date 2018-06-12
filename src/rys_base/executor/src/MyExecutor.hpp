#include <chrono>
#include <mutex>

struct Exec
{
	std::mutex *m;
	bool *activate;
	std::chrono::milliseconds delay;
	std::chrono::time_point<std::chrono::high_resolution_clock> nextActivationTime;
	Exec *next;
	Exec();
};

class MyExecutor {
	private:
		Exec *firstExec;
		bool *destroy;
	public:
		MyExecutor(bool&);
		~MyExecutor();
		void addExec(std::mutex& mut, bool& activationBool, std::chrono::milliseconds threadDelay);
		void spin();
		void list();
};