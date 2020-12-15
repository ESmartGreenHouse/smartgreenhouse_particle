/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/fabia/Downloads/particle_startup_isd/ISD/src/ISD.ino"
/*
 * Project ISD
 * Description:
 * Author:
 * Date:
 */
#include "main.h"

void setup();
void loop();
#line 9 "c:/Users/fabia/Downloads/particle_startup_isd/ISD/src/ISD.ino"
Main mainLoop;
const uint32_t UART_BAUD_RATE = 115200;
void setup() {  
mainLoop = Main();
mainLoop.initParticleVars();
}

void loop() {
  if(mainLoop.isConfigured())
  {
    mainLoop.loop();
  }
  else
  {

  }
  
}