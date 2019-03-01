#ifndef SRC_LOG_HPP_
#define SRC_LOG_HPP_

#include <mutex>
#include <map>
#include <iostream>

class Stats {
public:
	std::chrono::high_resolution_clock::time_point t1 {};
	std::chrono::high_resolution_clock::time_point t2 {};
	std::chrono::duration<double, std::milli> dt {};
	int counter { 0 };
	Stats() {}
	void setStart(std::chrono::high_resolution_clock::time_point &t) {
		t1 = t;
	}
	void setEnd(std::chrono::high_resolution_clock::time_point &t) {
		t2 = t;
		dt += t2 - t1;
		counter++;
	}
	double getAvgTime() {
	// returns the average time in milliseconds
		return dt.count() / counter;
	}

};

class Log {
private:
	std::mutex mtx { };
	std::map<std::string, Stats> logging{};

public:

	bool terminateGenPairs = false;
	bool terminateProcFeatures = false;
	bool terminateProcPose = false;

	void start (std::string st) {
		std::unique_lock<std::mutex> lck {mtx};
		std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
		logging[st].setStart(t);
	}

	void stop (std::string st) {
		std::unique_lock<std::mutex> lck {mtx};
		std::chrono::high_resolution_clock::time_point t = std::chrono::high_resolution_clock::now();
		if (logging.find(st)!=logging.end()) {
			logging[st].setEnd(t);
		}
	}

	void listRunningTimes() {
		for (auto x:logging) {
			std::cout << x.first << ": " << x.second.getAvgTime() << " ms" << std::endl;
		}
	}
};




#endif /* SRC_LOG_HPP_ */
