#include <unistd.h>

#include <iostream>
#include <string>

#include "timer/LoopTimer.h"

using namespace std;

int main(int argc, char** argv) {
	cout << endl
		 << "This example runs a timer with overtime monitoring." << endl
		 << endl;

	// create a loop timer
	SaiCommon::LoopTimer timer(1000.0, 1e6);
	timer.enableOvertimeMonitoring(0.4, 0.1, 9, true);

	// run for 2000 cycles
	while (timer.elapsedCycles() < 2000) {
		// force overtime on certain cycles
		if (timer.elapsedCycles() < 1000 && timer.elapsedCycles() > 900) {
			std::this_thread::sleep_for(std::chrono::microseconds(900));
		}

		if (timer.elapsedCycles() == 1900) {
			std::this_thread::sleep_for(std::chrono::microseconds(1400));
		}

		// wait the correct amount of time
		if (!timer.waitForNextLoop()) {
			std::cout << "Overtime detected at " << timer.elapsedCycles()
					  << " cycles." << std::endl;
			std::cout << std::endl;
		}
	}
	timer.stop();

	// print main timer info
	std::cout << "timer info:" << std::endl;
	timer.printInfoPostRun();

	return 0;
}