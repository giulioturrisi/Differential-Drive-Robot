#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>

#include <pigpio.h>

#include "rotary_encoder.hpp"
#include <chrono>
#include <unistd.h>

#include <cmath>
/*

REQUIRES

A rotary encoder contacts A and B connected to separate gpios and
the common contact connected to Pi ground.

TO BUILD

g++ -o rot_enc_cpp test_rotary_encoder.cpp rotary_encoder.cpp -lpigpio -lrt

TO RUN

sudo ./rot_enc_cpp

*/
static int count = 0;

void callback(int way)
{
   
   count += way;

   


   //std::cout << "count=" << count << std::endl;
}


void forward() {

    gpioWrite(26,1);
    gpioWrite(20,0);
    //usleep(1000);
    //gpioWrite(26,0);
}


void backward() {
        gpioWrite(26,0);
        //gpioWrite(20,1);
        gpioWrite(20,1);
        //usleep(1000);
        //gpioWrite(20,0);
}

int main(int argc, char *argv[])
{
   

   if (gpioInitialise() < 0) return 1;
   gpioSetMode(20, PI_OUTPUT);
   gpioWrite(20,0);
   gpioSetMode(26, PI_OUTPUT);
   gpioWrite(26,0);
   //gpioPWM(20,50);
   //gpioPWM(26,50);

   gpioSetMode(13, PI_OUTPUT);
   gpioSetPWMfrequency(13,1000);
   gpioSetPWMrange(13,255);
   
   

   re_decoder dec(27, 17, callback);

   double duty_cycle = 0;
   int W = 10;
   double error_angle = 0;
   double error_angle_integral = 0;
   double error_vel = 0;
   double error_vel_integral = 0;
   double error_acc = 0;
   double angle_des = 3.06;
   double vel_des = 8;

   double speed = 0;

   double error_vel_last = 0;
   double speed_last = 0;
   double speed_last_last = 0;
   double speed_filtered = 0;
   double angle_rad = 0;
   double old_angle_rad = 0;

   	std::vector<std::vector< double > > log;
	log.resize(2000);
	for (int j = 0; j < log.size(); ++j)
		log[j].resize(2, 0.0);

   for(int i = 0; i < 2000; i++) {

       if(i == 1000) {
           vel_des = 3;
           error_vel_integral = 0;
       }

	    auto start = std::chrono::high_resolution_clock::now();
	


        old_angle_rad = angle_rad;
        angle_rad = (count*0.20)*3.14159/180;

        speed = (angle_rad - old_angle_rad)/0.001;
        
        error_angle = angle_des - angle_rad; 
        error_angle_integral += error_angle;
        //duty_cycle = W*(100*error_angle + 0.0*error_angle_integral - 2*speed_filtered);

        speed_filtered = (speed_last + speed + speed_last_last*0)/2;
        speed_last_last = speed_last;
        speed_last = speed;
        
        error_vel = vel_des - speed_filtered;
        error_acc = (speed_last + speed_filtered)/0.001;
        error_vel_last = error_vel;
        error_vel_integral += error_vel;

        //duty_cycle = W*(10*error_vel + 0.1*error_vel_integral - 0*error_acc);

        duty_cycle = 255;
        std::cout << count << std::endl;

        if(duty_cycle > 255) duty_cycle = 255;
        if(duty_cycle < -255) duty_cycle = -255;

    
        if(duty_cycle < 0) {
            gpioPWM(13,-duty_cycle);
            backward();
        }
        else {
            gpioPWM(13,duty_cycle);
            forward();
        }

        double Ts_fake = 0;
        while (true){
            auto finish = std::chrono::high_resolution_clock::now();
            auto time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
            Ts_fake = (double)time / 1e3;
            if(Ts_fake > 0.999)
                break;
        }

        log[i][0] = speed;
        log[i][1] = speed_filtered;
        log[i][2] = Ts_fake;
        //log[i][3] = angle_rad;
  
   }

    std::cout << "error_angle" << error_angle << std::endl;
    std::cout << "here"  << std::endl;

   

   //sleep(3000);
   gpioPWM(0,duty_cycle);
   gpioWrite(26,0);
   gpioWrite(20,0);
   dec.re_cancel();

   gpioTerminate();

   	std::ofstream dataLog("./data_log.txt");
	if (dataLog.is_open())
	{
		for (int k = 0; k < log.size()-1; k++)
		{
			dataLog << log[k][0] << " ";
			dataLog << log[k][1] << " ";
			dataLog << log[k][2] << " ";
        
            dataLog << std::endl;
        }

		dataLog.close();
	}
	else
	{
		std::cout << "Log file could not be opened" << std::endl;

		return 1;
	}
    return 0;
}

