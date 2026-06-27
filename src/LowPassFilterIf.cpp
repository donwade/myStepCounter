#include <Arduino.h>
#include <iostream>
#include "LowPassFilter.hpp"
using namespace std;
#include <cmath> //Used for the sin() function.

// Create a low pass filter. DetltaTime for each cycle is unknown and will
// vary.
LowPassFilter lpf;

float run_LP(float value, float timeUs, float cutoffFreq)
{
#if 0
	// Cycles 500 times.
	//
	// As the lpf deltaTime set to 0.01 it will simulate 5 seconds of run time.
	for(int i = 0; i < 500; i++){
		// Simulate a slightly varying sampling time. for each cycle.
		float cycleTime = 0.01 + (0.002 * sin((float)i * 0.05));
		cout <<
			"cycleTime = " << cycleTime <<
			",\t Output = " <<
			// Update with 1.0 as input value, the current cycle time as
			// deltaTime and 2 Hz cutoff frequency.

			lpf.update((float) random(0, 100) / 100., cycleTime, 2) <<
			//lpf.update(1.0, cycleTime, 2) <<
			endl;
	}
#endif
    return lpf.update(value, timeUs, cutoffFreq);
}
