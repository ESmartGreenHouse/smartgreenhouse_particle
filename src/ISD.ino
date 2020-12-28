/*
 * Project ISD
 * Description:
 * Author:
 * Date:
 */
#include "Particle.h"
#include "CryptoSuite.h"
#include <stdint.h>
#include <WString.h>
#include "ArduinoJson.h"

// https://firestore.googleapis.com/v1/projects/awesomeprojekt-89fc9/databases/(default)/documents/testdata/
typedef enum State : uint8_t {
        STATE_IDLE = 0,
        STATE_RUNNING = 1,
        STATE_ERROR = 2
    } State;
    
#define maxVecSize 10

struct Data
{
  float vec[maxVecSize]= {};
  uint32_t size=0;
  char SensorName[60] ="";
};

#define CLOUD_RATE 10000
#define SENSOR_RATE 1000
#define HEARTBEAT_RATE 1000

uint32_t g_oldHeartbeatTimer=0;
uint32_t g_oldSensorTimer=0;
uint32_t g_oldCloudPushTime=0;

uint8_t g_STATE;
bool g_on_off = true;
uint32_t g_led = D7;

String g_deviceHash="non";
const char *PUBLISH_EVENT_NAME = "test1data";
Data g_rndSensorData;

bool toggleLeds() 
{
    if (g_on_off)
    {
        digitalWrite(g_led, HIGH);
        g_on_off = !g_on_off;
    }
    else
    {
        digitalWrite(g_led, LOW);
        g_on_off = !g_on_off;
    }
    return true;
}

bool heartbeatUpdate() 
{
    uint32_t now = millis();
    if (now >= (g_oldHeartbeatTimer + HEARTBEAT_RATE))
    {
        toggleLeds();
        g_oldHeartbeatTimer = millis();
    }
     return true;
}

String generateHashFromMac() 
{
  byte mac[6];
  WiFi.macAddress(mac);
  char macStr[18];
  char hashStr[65];

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Sha256.init();
  Sha256.print(macStr);

  uint8_t* hash= Sha256.result();

  for (int i=0; i<32; i++) {
  hashStr[i*2] = "0123456789abcdef"[hash[i]>>4];
  hashStr[i*2+1] = "0123456789abcdef"[hash[i]&0xf];
  }
  hashStr[64] ='\0';

  return String(hashStr);
}

JsonArray packValueType(JsonObject &obj, String ValueType, float* data, uint32_t size)
{
  int it =0 ;
  JsonArray fields_values_arrayValue_values = obj["values"]["arrayValue"].createNestedArray("values");
  DynamicJsonDocument abc(200);
  JsonObject nested = abc.createNestedObject();

  for (int i=0; i < size; ++i) {
    it++;
    nested[ValueType]= data[i];
    fields_values_arrayValue_values.add(nested);
  }  

  return fields_values_arrayValue_values;
} 

String get_JsonStructure(String sensorName, float* data,uint32_t size) 
{
    String ret;

    DynamicJsonDocument doc(4096);
    JsonObject fields = doc.createNestedObject("fields");
    fields["time"]["stringValue"] = "timestamp";
    fields["min_timestamp"]["timestampValue"] = Time.format(Time.now(), "%Y-%m-%dT%H:%M:00Z");
    fields["particle_id"]["stringValue"] = g_deviceHash;
    fields["sensor"]["stringValue"] = sensorName;
    packValueType(fields,"integerValue",data,size);
    serializeJson(doc, ret);
    return ret;
}

bool updateParticleCloud(Data Datafield) 
{
    uint32_t now = millis();
    if (now >= (g_oldCloudPushTime + CLOUD_RATE))
    {
      String jsonString = get_JsonStructure(String(Datafield.SensorName),Datafield.vec,Datafield.size);
      Particle.publish(PUBLISH_EVENT_NAME, jsonString, PRIVATE);
      Serial.printf(jsonString+"\n");
      g_oldCloudPushTime = millis();
      Datafield.size =0;
    }
     return true;
}

void rndDataCollector()
{
  if (g_rndSensorData.size < maxVecSize)
  {
    g_rndSensorData.vec[g_rndSensorData.size] = rand() % 20 + 1;
    ++g_rndSensorData.size;
  }
}

void DataCollectionTrigger()
{
    uint32_t now = millis();
    if (now >= (g_oldSensorTimer + SENSOR_RATE))
    {
      rndDataCollector();




      g_oldSensorTimer = millis();
      updateParticleCloud(g_rndSensorData);
    }
}

void setup() {  
  Serial.begin(9600);
  Sha256.init();
  pinMode(g_led, OUTPUT);
  g_deviceHash = generateHashFromMac();
}

void loop() {
  switch (g_STATE) {
  case State::STATE_IDLE: {
    heartbeatUpdate();
    DataCollectionTrigger();

  }; break;
  case State::STATE_RUNNING: {
  }; break;
  case State::STATE_ERROR: {
  }; break;
  };
}
