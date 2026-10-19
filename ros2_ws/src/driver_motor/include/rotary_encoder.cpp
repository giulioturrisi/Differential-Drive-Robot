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
   
   if (gpio == mygpioA) levA = level; else levB = level;

   if (gpio != lastGpio) // debounce 
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

void re_decoder::_pulse(int gpio, int level, uint32_t /*tick*/)
{
   // only one channel changes: the other one keeps its last level
   int A = levA;
   int B = levB;
   if (gpio == mygpioA) A = level; else B = level;


   if(levA == A && levB == B) return;


   int encoded = (A << 1) | B;
   int sum = (lastEncoded << 2) | encoded;

   if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) (mycallback)(-1);
   if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) (mycallback)(1);

   lastEncoded = encoded;
   levA = A;
   levB = B;

}


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

   gpioSetMode(gpioA, PI_INPUT);
   gpioSetMode(gpioB, PI_INPUT);

   /* pull up is needed as encoder common is grounded */

   gpioSetPullUpDown(gpioA, PI_PUD_UP);
   gpioSetPullUpDown(gpioB, PI_PUD_UP);

   /* the decoding starts from the actual state of the encoder */

   levA = gpioRead(gpioA);
   levB = gpioRead(gpioB);
   lastEncoded = (levA << 1) | levB;

   /* monitor encoder level changes */

   gpioSetAlertFuncEx(gpioA, _pulseEx, this);
   gpioSetAlertFuncEx(gpioB, _pulseEx, this);
}

void re_decoder::re_cancel(void)
{
   gpioSetAlertFuncEx(mygpioA, 0, this);
   gpioSetAlertFuncEx(mygpioB, 0, this);
}