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
static int count2 = 0;

void callback(int way)
{
   
   count += way;

   


   //std::cout << "count=" << count << std::endl;
}

void callback2(int way)
{
   
   count2 += way;

   


  // std::cout << "count2=" << count2 << std::endl;
}


void forward() {

    gpioWrite(26,1);
    gpioWrite(20,0);

    gpioWrite(5,1);
    gpioWrite(6,0);
    //usleep(1000);
    //gpioWrite(26,0);
}


void backward() {
        gpioWrite(26,0);
        //gpioWrite(20,1);
        gpioWrite(20,1);
        //usleep(1000);
        //gpioWrite(20,0);
        gpioWrite(5,0);
        //gpioWrite(20,1);
        gpioWrite(6,1);
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
   gpioSetPWMfrequency(13,50);
   gpioSetPWMrange(13,255);


   gpioSetMode(5, PI_OUTPUT);
   gpioWrite(5,0);
   gpioSetMode(6, PI_OUTPUT);
   gpioWrite(6,0);

   gpioSetMode(12, PI_OUTPUT);
   gpioSetPWMfrequency(12,50);
   gpioSetPWMrange(12,255);
   
   

   re_decoder dec(27, 17, callback);
   re_decoder dec2(22, 23, callback2);

   double duty_cycle = 0;
   int W = 10;
   double error_angle = 0;
   double error_angle_integral = 0;
   double error_vel = 0;
   double error_vel_integral = 0;
   double error_acc = 0;
   double angle_des = 0.01;
   double vel_des = 0.1;

   double speed = 0;
   double count2_last = 0;

   double error_vel_last = 0;
   double speed1 = 0;
   double speed2 = 0;
   double speed3 = 0;
   double speed4 = 0;
   double speed5 = 0;
   double speed6 = 0;
   double speed7 = 0;
   double speed8 = 0;
   double speed9 = 0;
   double speed10 = 0;
   double speed11 = 0;
   double speed12 = 0;
   double speed13 = 0;
   double speed14 = 0;
   double speed15 = 0;
   double speed16 = 0;
   double speed17 = 0;
   double speed18 = 0;
   double speed19 = 0;
   double speed_last = 0;
   //double speed_last_last = 0;
   double speed_filtered = 0;
   double angle_rad = 0;
   double old_angle_rad = 0;

   	std::vector<std::vector< double > > log;
	log.resize(1000*3);
	for (int j = 0; j < log.size(); ++j)
		log[j].resize(3, 0.0);

   for(int i = 0; i < 1000; i++) {

       if(i % 20 == 0) {
           //std::cout << "############ ang des "  << angle_des << std::endl;
           //std::cout << "############ ang "  << angle_rad << std::endl;

           vel_des = vel_des*1.1;
           angle_des = angle_des + 0.1;
           error_vel_integral = 0;
           //angle_des = -angle_des;
           error_angle_integral = 0;
           //std::cout << "############ vel des "  << vel_des << std::endl;
           //std::cout << "############ error_vel_integral "  << error_vel_integral << std::endl;
           std::cout << "############ speed_filtered "  << speed_filtered << std::endl;


       }

	    auto start = std::chrono::high_resolution_clock::now();
	


        old_angle_rad = angle_rad;
        angle_rad = (count2*0.11)*3.14159/180;
        
        //std::cout << "############"  << std::endl;
        //std::cout << "old_count" << count2_last << std::endl;
        //std::cout << "old_angle_rad" << old_angle_rad << std::endl;
        //std::cout << "count2 " << count2 << std::endl; 
        //std::cout << "count " << count << std::endl; 
        
        count2_last = count2;
        
        speed = (angle_rad - old_angle_rad)/0.001;
        
        //std::cout << "angle" << angle_rad << std::endl; 

        error_angle = angle_des - angle_rad; 
        error_angle_integral += error_angle;
        //duty_cycle = W*(100*error_angle + 0.0*error_angle_integral - 2*speed_filtered);
        //duty_cycle = (200*error_angle + 0.1*error_angle_integral - 2*speed_filtered);
        //duty_cycle = 0;

        //speed_filtered = (speed_last + speed + speed_last_last*0)/2;
        //speed_last_last = speed_last;
        //speed_last = speed;

        speed_filtered = (speed + speed1 + speed2 + speed3 + speed4 + speed5 + speed6 + speed7 + speed8 + speed9)/10.;
                        //+ speed10 + speed11 + speed12 + speed13 + speed14 + speed15 + speed16 + speed17 + speed18 + speed19)/20.;
        //speed_filtered = (speed + speed1 + speed2 + speed4)/4.;
        speed19 = speed18;
        speed18 = speed17;
        speed17 = speed16;
        speed16 = speed15;
        speed15 = speed14;
        speed14 = speed13;
        speed13 = speed12;
        speed12 = speed11;
        speed11 = speed10;
        speed10 = speed9;
        
        speed9 = speed8;
        speed8 = speed7;
        speed7 = speed6;
        speed6 = speed5;
        speed5 = speed4;
        speed4 = speed3;
        speed3 = speed2;
        speed2 = speed1;
        speed1 = speed;

        
        
        error_vel = vel_des - speed_filtered;
        error_acc = (speed_last + speed_filtered)/0.001;
        speed_last = speed_filtered;
        //error_acc = 0;
        error_vel_last = error_vel;
        error_vel_integral += error_vel;

        duty_cycle = (10000*error_angle + 0*error_angle_integral - 100*speed_filtered);
        if(duty_cycle > 0) duty_cycle += 80;
        else  duty_cycle -= 80;
        //duty_cycle = W*(10*error_vel + 0.1*error_vel_integral - 0*error_acc);
        //duty_cycle = 80 + (400*error_vel + 5*error_vel_integral - 0.01*error_acc);
        duty_cycle = 250;
        //duty_cycle = 90;
        //std::cout << count << std::endl;

        if(duty_cycle > 255) duty_cycle = 255;
        if(duty_cycle < -255) duty_cycle = -255;

    
        if(duty_cycle < 0) {
            gpioPWM(13,-duty_cycle);
            gpioPWM(12,-duty_cycle);
            backward();
        }
        else {
            gpioPWM(13,duty_cycle);
            gpioPWM(12,duty_cycle);
            forward();
        }

        double Ts_fake = 0;
        while (true){
            auto finish = std::chrono::high_resolution_clock::now();
            auto time = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
            Ts_fake = (double)time / 1e3;
            if(Ts_fake > 0.999){
                //std::cout << "time" << Ts_fake << std::endl;
                //std::cout << "ang "  << angle_rad << std::endl;
                break;
            }
        }

        /*log[i][0] = speed;
        log[i][1] = speed_filtered;
        log[i][2] = Ts_fake;
        log[i][3] = count2;*/
  
   }

    std::cout << "error_angle" << error_angle << std::endl;
    std::cout << "here"  << std::endl;

   

   //sleep(3000);
   //gpioPWM(0,duty_cycle);
   gpioWrite(26,0);
   gpioWrite(20,0);

   gpioWrite(5,0);
   gpioWrite(6,0);
   dec.re_cancel();

   gpioTerminate();

   	std::ofstream dataLog("./data_log.txt");
	if (dataLog.is_open())
	{
		for (int k = 0; k < log.size()-1; k++)
		{
			/*dataLog << log[k][0] << " ";
			dataLog << log[k][1] << " ";
			dataLog << log[k][2] << " ";
            dataLog << log[k][3] << " ";
        
            dataLog << std::endl;*/
        }

		//dataLog.close();
	}
	else
	{
		std::cout << "Log file could not be opened" << std::endl;

		return 1;
	}
    return 0;
}

