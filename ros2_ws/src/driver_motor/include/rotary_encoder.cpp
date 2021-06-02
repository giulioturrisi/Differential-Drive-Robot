#include <iostream>

#include <pigpio.h>

#include "rotary_encoder.hpp"

/*

             +---------+         +---------+      0
             |         |         |         |
   A         |         |         |         |
             |         |         |         |
   +---------+         +---------+         +----- 1

       +---------+         +---------+            0
       |         |         |         |
   B   |         |         |         |
       |         |         |         |
   ----+         +---------+         +---------+  1

*/

/*void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{

   std::cout << "######" << std::endl;
   std::cout << "chiamato da " << gpio << std::endl;
   std::cout << "con valore " << level << std::endl;
   std::cout << "l'ultimo era " << lastGpio << std::endl;
   //levA = gpioRead(27);
   if(level == 2) return;
   levA = level;
   levB = gpioRead(17);
   levB = !levA;
   std::cout << "B è " << levB << std::endl;

   if (levA != lastGpio) 
   {

         if (levA != levB) (mycallback)(1);

         if (levA == levB) (mycallback)(-1);

   }
   lastGpio = levA;
   std::cout << "lastGPIO" << lastGpio <<  std::endl;
}*/

/*void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{
   if (gpio == mygpioA) levA = level; else levB = level;

   if (gpio != lastGpio) 
   {
      lastGpio = gpio;

      if ((gpio == mygpioA) && (level == 1))
      {
         if (levB) (mycallback)(1);
      }
      else if ((gpio == mygpioB) && (level == 1))
      {
         if (levA) (mycallback)(-1);
      }
   }
}*/

void re_decoder::_pulse(int gpio, int level, uint32_t tick)
{
   bool A,B;
   if (gpio == mygpioA) A = level; else B = level;
   //bool A = gpioRead(22), B = gpioRead(23);

   if(levA == A && levB == B) return; 


   int encoded = (A << 1) | B;
   int sum = (lastGpio << 2) | encoded;

   if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) (mycallback)(-1);
   if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) (mycallback)(1);

   lastGpio = encoded;
   levA = A;
   levB = B;

}

/*void re_decoder::_pulse(int gpio, int level, uint32_t tick) {
   bool A = gpioRead(27), B = gpioRead(17);

   int counter = 0;

   if (B != prevB) counter += (B-prevB) * (A ? +1 : -1);
   else if (A != prevA) counter += (A-prevA) * (B ? -1 : +1);
   else return; //nothing changed: exit

   if(counter > 0)
     (mycallback)(1);
   else
      (mycallback)(-1);
   prevA = A;
   prevB = B;

}*/

void re_decoder::_pulseEx(int gpio, int level, uint32_t tick, void *user)
{
   /*
      Need a static callback to link with C.
   */

   re_decoder *mySelf = (re_decoder *) user;

   mySelf->_pulse(gpio, level, tick); /* Call the instance callback. */
}

re_decoder::re_decoder(int gpioA, int gpioB, re_decoderCB_t callback)
{
   mygpioA = gpioA;
   mygpioB = gpioB;

   mycallback = callback;

   levA=0;
   levB=0;

   lastGpio = -1;

   gpioSetMode(gpioA, PI_INPUT);
   gpioSetMode(gpioB, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   gpioSetPullUpDown(gpioA, PI_PUD_UP);
   gpioSetPullUpDown(gpioB, PI_PUD_UP);

   /* monitor encoder level changes */

   gpioSetAlertFuncEx(gpioA, _pulseEx, this);
   gpioSetAlertFuncEx(gpioB, _pulseEx, this);
}

void re_decoder::re_cancel(void)
{
   gpioSetAlertFuncEx(mygpioA, 0, this);
   //gpioSetAlertFuncEx(mygpioB, 0, this);
}

