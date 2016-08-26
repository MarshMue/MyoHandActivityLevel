// MyoDisplacement.cpp : Defines the entry point for the console application.
//

#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <myo/myo.hpp>
#include <chrono>

using namespace std::chrono;

const double GRAVITY = 9.80665;
class DataCollector : public myo::DeviceListener {
public:
	DataCollector()
	{
		prevVel.x;
		prevVel.y;
		prevVel.z;

		currVel.x = 0;
		currVel.y = 0;
		currVel.z = 0;
		currVel.magUpdate();
		initTime = system_clock::now();
		prevTime = initTime;

		samples = 0;
		//count = 0;
		averagex = 0;
		averagey = 0;
		averagez = 0;
		workTime = initTime - initTime;
		inWork = false;
		inWorkPrev = false;
		totalRMS = 0;
		rmsSpeed = 0;
	}

	struct vec {
		double x;
		double y;
		double z;
		double mag;
		void magUpdate() {
			mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
		} 
	};

	// data from armband
	myo::Vector3<float> rawAccel = myo::Vector3<float>();
	myo::Vector3<float> Accel = myo::Vector3<float>();
	myo::Vector3<float> Accelms2 = myo::Vector3<float>();
	myo::Quaternion<float> orientation = myo::Quaternion<float>();

	//miscellaneous data
	vec prevVel, currVel, initVel;
	double HAL, rmsSpeed, dutyCycle, averagex, averagey, averagez, totalRMS;
	time_point<system_clock> initTime, prevTime;
	duration<double> dt, workTime;
	uint32_t samples; //count
	bool inWork, inWorkPrev;

	

	// onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
	void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
	{

		for (size_t i = 0; i < 8; i++) {

		}
	}

	// onOrientationData is called whenever new orientation data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onOrientationData(myo::Myo *myo, uint64_t timestamp, const myo::Quaternion< float > &rotation) {
		orientation = rotation;

		using std::atan2;
		using std::asin;
		using std::sqrt;
		using std::max;
		using std::min;

		// Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
		float roll = atan2(2.0f * (rotation.w() * rotation.x() + rotation.y() * rotation.z()),
			1.0f - 2.0f * (rotation.x() * rotation.x() + rotation.y() * rotation.y()));
		float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (rotation.w() * rotation.y() - rotation.z() * rotation.x()))));
		float yaw = atan2(2.0f * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
			1.0f - 2.0f * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));
	}

	// onAccelerometerData is called whenever new acceleromenter data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onAccelerometerData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &accel) {
		rawAccel = accel;
		// update prev vel
		prevVel.x = currVel.x;
		prevVel.y = currVel.y;
		prevVel.z = currVel.z;

		
		// update acceleration
		Accel = myo::rotate(orientation, rawAccel);
		Accel = myo::Vector3<float>(Accel.x(), Accel.y(), Accel.z() - 1.0);
		Accelms2 = myo::Vector3<float>(Accel.x() * GRAVITY, Accel.y() * GRAVITY, Accel.z() * GRAVITY - .28); //.28 taken from an average of the noise when myo was on table, ROUGH

		bool xfilt, yfilt, zfilt;
		xfilt = yfilt = zfilt = false;

		// filter noise?
		double cutoff = 0.05;
		if (Accelms2.x() < cutoff && Accelms2.x() > -cutoff) {
			Accelms2 = myo::Vector3<float>(0, Accelms2.y(), Accelms2.z());
			currVel.x = 0;
			xfilt = true;
		}
		if (Accelms2.y() < cutoff && Accelms2.y() > -cutoff) {
			Accelms2 = myo::Vector3<float>(Accelms2.x(), 0, Accelms2.z());
			yfilt = true;
		}
		if (Accelms2.z() < cutoff && Accelms2.z() > -cutoff) {
			Accelms2 = myo::Vector3<float>(Accelms2.x(), Accelms2.y(), 0);
			zfilt = true;
		}
		
		
		// update velocity
		dt = system_clock::now() - prevTime;
		prevTime = system_clock::now();
		if (!xfilt) {
			currVel.x = prevVel.x + dt.count() * Accelms2.x();
		}
		else {
			currVel.x = 0;
			xfilt = false;
		}
		if (!yfilt) {
			currVel.y = prevVel.y + dt.count() * Accelms2.y();
		}
		else {
			currVel.y = 0;
			yfilt = false;
		}
		if (!zfilt) {
			currVel.z = prevVel.z + dt.count() * Accelms2.z();
		}
		else {
			currVel.z = 0;
			zfilt = false;
		}
		
		currVel.magUpdate();
		
		float accelMag = sqrt(pow(Accelms2.x(), 2) + pow(Accelms2.y(), 2) + pow(Accelms2.z(), 2));
		
		
		
		if (currVel.mag > 0.1) {
			inWorkPrev = inWork;
			inWork = true;
			workTime += dt;
			initVel.x = prevVel.x;
			initVel.y = prevVel.y;
			initVel.z = prevVel.z;
			initVel.magUpdate();
		}
		else {
			inWorkPrev = inWork;
			inWork = false;

			initVel.x = prevVel.x;
			initVel.y = prevVel.y;
			initVel.z = prevVel.z;
			initVel.magUpdate();
		}

		updateHAL();

	}

	// onGyroscopeData is called whenever new gyroscope data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
		//printVector(gyroFile, timestamp, gyro);

	}

	void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
		//Reneable streaming
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
	}

	void updateHAL() {
		// get RMS speed in mm/s
		time_point<system_clock> currTime = system_clock::now();
		duration<double> totalTime = currTime - initTime;
		if (inWork) {
			totalRMS += totalRMS + currVel.mag;
			samples++;
			rmsSpeed = totalRMS * 1000 / samples;
		}
		
		// duty cycle = 100 * (work time / (work time + rest time) )
		dutyCycle = 100 * (workTime / (currTime - initTime));
		HAL = 10 * (exp(-15.87 + 0.02*dutyCycle + 2.25 * log(rmsSpeed)))
			/ (1 + exp(-15.87 + 0.02*dutyCycle + 2.25 * log(rmsSpeed)));
	}

	// Helper to print out accelerometer and gyroscope vectors
	void printVector(std::ofstream &file, uint64_t timestamp, const myo::Vector3< float > &vector) {
		// Clear the current line
		//std::cout << std::setw(7) << '\r' << abs(sqrt(pow(vector.x(), 2) + pow(vector.y(), 2) + pow(vector.z(), 2)) - 1) * 9.8 << std::flush;
	}
	
	// accelerometer data with gravity accounted for
	void gravityAdjustedAccel() {
		// get expected direction of gravity
		float x = 2 * (orientation.x() * orientation.z() - orientation.w() * orientation.y());
		float y = 2 * (orientation.w() * orientation.x() + orientation.y() * orientation.z());
		float z = pow(orientation.w(), 2) - pow(orientation.x(), 2) - pow(orientation.z(), 2) + pow(orientation.z(), 2);
		float accx = rawAccel.x() - x;
		float accy = rawAccel.y() - y;
		float accz = rawAccel.z() - z;
		myo::Vector3<float> gravityAdjusted = myo::Vector3<float>(accx, accy, accz);
		Accel = gravityAdjusted;

	}
	
	// print acceleration in orientation and accounting for gravity in g's
	void printAccel() {
		std::cout << '\r' << "x: " << Accel.x() << " y: " << Accel.y() << " z: " << Accel.z() << std::flush;
	}

	// print acceleration in orientation and accounting for gravity in m/s^2
	void printAccelms2() {
		/*
		averagex += Accelms2.x();
		averagey += Accelms2.y();
		averagez += Accelms2.z();
		<< " avex: " << std::setw(5) << (averagex / count)
			<< " avey: " << std::setw(5) << (averagey / count)
			<< " avez: " << std::setw(5) << (averagez / count)
		count++;
		*/
		std::cout
			<< " x: " << std::setw(4) << Accelms2.x() 
			<< " y: " << std::setw(4) << Accelms2.y() 
			<< " z: " << std::setw(4) << Accelms2.z() 
			
			<< std::flush;
	}

	void printAbsAccel() {
		std::cout << std::setw(7) << sqrt(pow(Accel.x(), 2) + pow(Accel.y(), 2) + pow(Accel.z(), 2)) 
			<< std::flush;
	}

	void printVel() {
		std::cout << '\r' 
			<< "x: " << std::setw(4) << currVel.x 
			<< " y: " << std::setw(4) << currVel.y 
			<< " z: " << std::setw(4) << currVel.z
			<< " mag: " <<std::setw(4) << currVel.mag;
		printAccelms2();
	}

	void printHAL() {
		std::cout << "\rHAL: " << std::setw(3) << HAL << " vel: " << currVel.mag << " duty cycle: " << dutyCycle << " rmsSpeed: " << rmsSpeed 
			<< " totalRMS " << totalRMS 
			<< " dt: " << dt.count() << std::endl;
	}


};

int main()
{
	// We catch any exceptions that might occur below -- see the catch statement for more details.
	try {
		// First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
		// publishing your application. The Hub provides access to one or more Myos.
		myo::Hub hub("com.example.hello-myo");
		std::cout << "Attempting to find a Myo..." << std::endl;
		// Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
		// immediately.
		// waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
		// if that fails, the function will return a null pointer.
		myo::Myo* myo = hub.waitForMyo(10000);
		// If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
		if (!myo) {
			throw std::runtime_error("Unable to find a Myo!");
		}
		// We've found a Myo.
		std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

		// Next we enable EMG streaming on the found Myo.
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);

		// Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
		DataCollector collector;
		// Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
		// Hub::run() to send events to all registered device listeners.
		hub.addListener(&collector);

		

		// Finally we enter our main loop.
		while (1) {
			// In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
			// In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
			hub.run(1000 / 20);
			// After processing events, we call the print() member function we defined above to print out the values we've
			// obtained from any events that have occurred.
			collector.printHAL();
			//std::cout << '/n';
			//collector.printAbsAccel();
		}
		// If a standard exception occurred, we print out its message and exit.
	}
	catch (const std::exception& e) {
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}

