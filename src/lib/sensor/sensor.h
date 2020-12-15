#ifndef __SENSOR_H__
#define __SENSOR_H__

 #include <stdint.h>

class Sensor {
   public:
    /**
     * @brief Pure Virtual Update Function implemented from sensors 
     * @return Success True else False
     */
    virtual bool update();
};

#endif // __SENSOR_H__