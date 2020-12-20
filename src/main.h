#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdint.h>
#include <vector>
#include "lib/DHT22/DHT22.h"
#include "lib/Heartbeat/Heartbeat.h"
#include "lib/ParticleCloud/ParticleCloud.h"
#include "Particle.h"

class Main {
   public:
    /**
     * @brief Main constructor.
     */
    Main();

    /**
     * @brief Main loop.
     */
    void loop();
    bool isConfigured();
    void initParticleVars();
    Heartbeat m_Heartbeat;

 private:

     typedef enum State : uint8_t {
        IDLE = 0,
        RUNNING = 1,
        ERROR = 2
    } State;
    ParticleCloud m_particleCloud;
    DHT22 m_DHT22;
    std::vector<Sensor *> m_SensorVector;
    uint8_t m_mainState = 0;
};
#endif // __MAIN_H__