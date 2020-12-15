#ifndef __DHT22_H__
#define __DHT22_H__

#include <stdint.h>
#include "../sensor/sensor.h"
#include "../ParticleCloud/ParticleCloud.h"

class DHT22 : private Sensor {
   public:
    /**
     * @brief Main constructor.
     */
    DHT22();

    /**
     * @brief Main loop.
     */
    int32_t getTemperature();
    bool isConfigured();
    bool update();
    int32_t m_temp=0;
    uint32_t m_humidity=0;
    int32_t m_heatValue=0;


 private:

    uint8_t m_mainState = 0;
};
#endif // __DHT22_H__