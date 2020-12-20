#include "main.h"
#include "CryptoSuite.h"

#undef min
#undef max

#include <vector>

using namespace std;

/**
 * @brief Section for constants definitions
 **/
// Serial port
const uint32_t UART_BAUD_RATE = 115200;


/**
 * @brief Section for class function implementations
 **/
Main::Main(): m_particleCloud(), m_DHT22(), m_SensorVector(),m_Heartbeat()
{
    Serial.begin(9600);
    Sha256.init();
}


void Main::loop() 
{
m_Heartbeat.update();


/////// DEBUG RANDOM VEC
vector<float> myVector;

srand((unsigned)millis());

int a = rand() % 20 + 1; //1 to 20    
for (int i =0; i < a; i++){
        int b = rand() % 20 + 1;
        myVector.push_back(b);
    }
/////// DEBUG RANDOM VEC END

//Serial.printf(m_particleCloud.get_JsonStructure("AwesomeName", myVector)+"\n");
m_particleCloud.update("RndSensor", myVector);

 switch (m_mainState) {
 case State::IDLE: {

 }; break;
 case State::RUNNING: {

 }; break;
 case State::ERROR: {

 }; break;
 };

}

bool Main::isConfigured() {
    return true;
}

void Main::initParticleVars() 
{
    m_Heartbeat.initParticleVars();
}
