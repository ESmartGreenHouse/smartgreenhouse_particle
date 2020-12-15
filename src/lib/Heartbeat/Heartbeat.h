#ifndef __HEARTBEAT_H__
#define __HEARTBEAT_H__

#include <stdint.h>
#include "Particle.h"

class Heartbeat {
   public:
    /**
     * @brief Main constructor.
     */
      Heartbeat();

      bool toggleLeds();
      bool update();
      bool initParticleVars();
      bool beginn();
      bool isConfigured();

      uint32_t m_oldTime;

   private:
      bool m_on_off = false;
      uint8_t m_led = D7;

};
#endif // __HEARTBEAT_H__