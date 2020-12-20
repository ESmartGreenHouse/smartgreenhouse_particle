#include "Heartbeat.h" 

Heartbeat::Heartbeat() : m_oldTime(0)
{
    pinMode(m_led, OUTPUT);
}

bool Heartbeat::toggleLeds() 
{
    if (m_on_off)
    {
        digitalWrite(m_led, HIGH);
        m_on_off = !m_on_off;
    }
    else
    {
        digitalWrite(m_led, LOW);
        m_on_off = !m_on_off;
    }
    return true;
}

bool Heartbeat::update() 
{
    uint32_t now = millis();
    if (now >= (m_oldTime + 1000))
    {
        toggleLeds();
        m_oldTime = millis();
    }
     return true;
}

bool Heartbeat::initParticleVars() 
{
    //Particle.variable("UpTime", m_oldTime);
}

bool Heartbeat::isConfigured() 
{
     return true;
}
