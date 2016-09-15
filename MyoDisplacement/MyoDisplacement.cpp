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
#include <stdio.h>
#include <curl/curl.h>
#include <string>



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
		stationary = false;
		gyroMag = 0;
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
	myo::Vector3<float> gyroAvg;
	myo::Quaternion<float> orientation = myo::Quaternion<float>();

	//moving average data accel
	float movAvgX[4] = { 0, 0, 0, 0 };
	float movAvgY[4] = { 0, 0, 0, 0 };
	float movAvgZ[4] = { 0, 0, 0, 0 };

	//moving average data gyro
	float movAvgw0[4] = { 0, 0, 0, 0 };
	float movAvgw1[4] = { 0, 0, 0, 0 };
	float movAvgw2[4] = { 0, 0, 0, 0 };
	float gyroMag;

	//miscellaneous data
	vec prevVel, currVel, initVel;
	double HAL, rmsSpeed, dutyCycle, averagex, averagey, averagez, totalRMS;
	time_point<system_clock> initTime, prevTime;
	duration<double> dt, workTime;
	uint32_t samples; //count
	bool inWork, inWorkPrev, stationary;
	std::string callBackContent; //will hold the url's contents
	
	

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
		
		updateVelocity(accel);
		updateHAL();

	}

	// 
	void updateVelocity(const myo::Vector3< float > &accel) {
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

		updateMovAvg(Accelms2.x(), movAvgX, 4);
		updateMovAvg(Accelms2.y(), movAvgY, 4);
		updateMovAvg(Accelms2.z(), movAvgZ, 4);

		float xAvg = calcAvg(movAvgX, 4);
		float yAvg = calcAvg(movAvgY, 4);
		float zAvg = calcAvg(movAvgZ, 4);

		// replace Acceleration vector with moving average values
		Accelms2 = myo::Vector3<float>(xAvg, yAvg, zAvg);

		// filter noise?
		double cutoff = 0.05;
		if (Accelms2.x() < cutoff && Accelms2.x() > -cutoff) {
			Accelms2 = myo::Vector3<float>(0, Accelms2.y(), Accelms2.z());
		}
		if (Accelms2.y() < cutoff && Accelms2.y() > -cutoff) {
			Accelms2 = myo::Vector3<float>(Accelms2.x(), 0, Accelms2.z());
		}
		if (Accelms2.z() < cutoff && Accelms2.z() > -cutoff) {
			Accelms2 = myo::Vector3<float>(Accelms2.x(), Accelms2.y(), 0);
		}


		// update velocity
		dt = system_clock::now() - prevTime;
		prevTime = system_clock::now();

		// x
		if (!stationary) {
			currVel.x = prevVel.x + dt.count() * Accelms2.x();
		}
		else {
			currVel.x = 0;
		}

		// y
		if (!stationary) {
			currVel.y = prevVel.y + dt.count() * Accelms2.y();
		}
		else {
			currVel.y = 0;
		}

		// z
		if (!stationary) {
			currVel.z = prevVel.z + dt.count() * Accelms2.z();
		}
		else {
			currVel.z = 0;
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
	}

	// onGyroscopeData is called whenever new gyroscope data is provided
	// Be warned: This will not make any distiction between data from other Myo armbands
	void onGyroscopeData(myo::Myo *myo, uint64_t timestamp, const myo::Vector3< float > &gyro) {
		//printVector(gyroFile, timestamp, gyro);
		stationaryDetection(gyro);
		
	}

	// sets stationary value to true when the arm is stationary, false if moving
	void stationaryDetection(const myo::Vector3< float > &gyro) {
		updateMovAvg(gyro.x(), movAvgw0, 4);
		updateMovAvg(gyro.y(), movAvgw1, 4);
		updateMovAvg(gyro.z(), movAvgw2, 4);

		float w0Avg = calcAvg(movAvgw0, 4);
		float w1Avg = calcAvg(movAvgw1, 4);
		float w2Avg = calcAvg(movAvgw2, 4);

		// replace Acceleration vector with moving average values
		gyroAvg = myo::Vector3<float>(w0Avg, w1Avg, w2Avg);

		gyroMag = sqrt(pow(gyroAvg.x(), 2) + pow(gyroAvg.y(), 2) + pow(gyroAvg.z(), 2));
		float threshold = 2;
		if (gyroMag < threshold) {
			stationary = true;
		}
		else {
			stationary = false;
		}
	}

	void onConnect(myo::Myo *myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion) {
		//Reneable streaming
		myo->setStreamEmg(myo::Myo::streamEmgEnabled);
	}

	// updates the HAL
	void updateHAL() {
		// get RMS speed in mm/s
		time_point<system_clock> currTime = system_clock::now();
		duration<double> totalTime = currTime - initTime;
		if (inWork) {
			totalRMS += currVel.mag;
			samples++;
			rmsSpeed = totalRMS * 1000 / samples;
		}
		
		// duty cycle = 100 * (work time / (work time + rest time) )
		dutyCycle = 100 * (workTime / (currTime - initTime));
		// HAL rounded to nearest half
		HAL = std::round(2*(10 * (exp(-15.87 + 0.02*dutyCycle + 2.25 * log(rmsSpeed)))
			/ (1 + exp(-15.87 + 0.02*dutyCycle + 2.25 * log(rmsSpeed))))) / 2;
		//std::cout << HAL << std::endl;
		//***********************send HAL here***************************
		curl_global_init(CURL_GLOBAL_ALL);
		//char variable_string[] = "HAL";
		make_post(HAL);
		curl_global_cleanup();
	}

	static std::string readBuffer;

	static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
	{
		((std::string*)userp)->append((char*)contents, size * nmemb);
		return size * nmemb;
	}

	void make_post(double hal) {
		//curl represents a request
		CURL *curl;
		//represents a response
		CURLcode res;
		std::string readBuffer;
		curl = curl_easy_init();
		if (curl) {
			//set the url to post to
			char url[1024];
			std::string route = "http://192.168.1.163:5000/tracking/myo/HAL/" + std::to_string(hal);
			strcpy_s(url, route.c_str());

			curl_easy_setopt(curl, CURLOPT_URL, url);

			// send all data to this function
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);

			//set the postfield data
			//curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);

			res = curl_easy_perform(curl);
			// Check for errors
			if (res != CURLE_OK) {
				fprintf(stderr, "curl_easy_perform() failed: %s\n",
					curl_easy_strerror(res));
			}
			else {
				std::cout << readBuffer << std::endl;
			}
			
			// always cleanup
			curl_easy_cleanup(curl);

			// we're done with libcurl, so clean it up 
			curl_global_cleanup();
		}
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
		//std::cout << "\rHAL: " << std::setw(3) << HAL << " vel: " << currVel.mag << " duty cycle: " << dutyCycle << " rmsSpeed: " << rmsSpeed 
			//<< " totalRMS " << totalRMS 
			//<< " gyro: " << gyroMag << std::endl;
	}

	// update moving average filter
	void updateMovAvg(float updateVal, float* valArray, int len) {
		for (int i = len - 1; i > 0; i--) {
			valArray[i] = valArray[i - 1];
		}
		valArray[0] = updateVal;
	}

	float calcAvg(float* valArray, int len) {
		float total = 0;
		for (int i = len - 1; i >= 0; i--) {
			total += valArray[i];
		}
		
		return total / len;
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
			//collector.printHAL();
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

