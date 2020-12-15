/*
 * Project ISD
 * Description:
 * Author:
 * Date:
 */
#include "main.h"

Main mainLoop;
void setup() {  
mainLoop = Main();
mainLoop.initParticleVars();
}

void loop() {
  
  if(mainLoop.isConfigured())
  {
    mainLoop.loop();
  }
}