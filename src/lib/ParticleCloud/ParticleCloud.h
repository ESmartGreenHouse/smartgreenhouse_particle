#ifndef __PARTICLECLOUD_H__
#define __PARTICLECLOUD_H__


#include <stdint.h>
#include <WString.h>
#include "ArduinoJson.h"
#undef min
#undef max

#include <vector>
class ParticleCloud {
   public:
    /**
     * @brief Pure Virtual Update Function implemented from sensors 
     * @return Success True else False
     */
    ParticleCloud();
    bool update(String sensorName, std::vector<float> data);
    bool connectToFirestore();
    bool init();
    const char *PUBLISH_EVENT_NAME = "test1data";
    String generateHashFromMac();
    JsonArray packValueType(JsonObject &obj, String ValueType, std::vector<float> data);
    String get_JsonStructure(String sensorName, std::vector<float> data);
    String m_deviceHash="non";
    uint32_t m_oldTime;
};


#endif // __PARTICLECLOUD_H__