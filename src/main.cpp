#include "main.h"

/**
 * @brief Section for constants definitions
 **/
// Serial port
const uint32_t UART_BAUD_RATE = 115200;


/**
 * @brief Section for class function implementations
 **/
Main::Main(): m_DHT22(), m_SensorVector(),m_Heartbeat()
{
       
}

void Main::loop() 
{
 m_Heartbeat.update();

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
