#ifndef __PARTICLECLOUD_H__
#define __PARTICLECLOUD_H__

 #include <stdint.h>

class ParticleCloud {
   public:
    /**
     * @brief Pure Virtual Update Function implemented from sensors 
     * @return Success True else False
     */
    bool updateVariable();
};


#endif // __PARTICLECLOUD_H__