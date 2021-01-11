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
#include "DHT.h"

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
  uint32_t DataCount=0;
  char SensorName[60] ="";
};

#define CLOUD_RATE 10000
#define DATA_PER_MINUTE maxVecSize
#define SENSOR_RATE CLOUD_RATE/DATA_PER_MINUTE
#define HEARTBEAT_RATE 1000

uint32_t g_oldHeartbeatTimer=0;
uint32_t g_oldSensorTimer=0;
uint32_t g_oldCloudPushTime=0;

uint8_t g_STATE;
bool g_on_off = true;
uint32_t g_led = D7;
double RndSensorVar= 0.0;

String g_deviceID= System.deviceID();
const char *PUBLISH_EVENT_NAME = "test1data";
Data g_rndSensorData;

//---------Sensors dec-----------------
#define LIGHTPIN A0

Data g_lightSensorData;
//----------Sensors end-------------


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
  char macStr[19];
  char hashStr[65];

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  macStr[18]='\0';

  Serial.printf("The Mac of this device is: %s\n",macStr);
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

  for (uint32_t i=0; i < size; ++i) {
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
    fields["particle_id"]["stringValue"] = g_deviceID;
    fields["sensor"]["stringValue"] = sensorName;
    packValueType(fields,"doubleValue",data,size);
    serializeJson(doc, ret);
    return ret;
}


void push_jsonstring_to_cloud(Data* Datafield){
  String jsonString = get_JsonStructure(String(Datafield->SensorName),Datafield->vec,Datafield->DataCount);
  Particle.publish(PUBLISH_EVENT_NAME, jsonString, PRIVATE);
  Serial.printf(jsonString+"\n");
  Datafield->DataCount =0;
}

bool updateParticleCloud() 
{
    uint32_t now = millis();
    if (now >= (g_oldCloudPushTime + CLOUD_RATE))
    {
      
      push_jsonstring_to_cloud(&g_rndSensorData);
      push_jsonstring_to_cloud(&g_lightSensorData);
      g_oldCloudPushTime = millis();
    }
     return true;
}

void rndDataCollector()
{
  if (strlen(g_rndSensorData.SensorName)==0)
  {
  strcpy( g_rndSensorData.SensorName, String("RndSensor").c_str() );
  }
  if (g_rndSensorData.DataCount < maxVecSize)
  {
    RndSensorVar= static_cast<float>(random(2000)) / 100.0;
    g_rndSensorData.vec[g_rndSensorData.DataCount] =RndSensorVar;
    Serial.printf("%f\n",g_rndSensorData.vec[g_rndSensorData.DataCount]);
    ++g_rndSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i < %i\n",g_rndSensorData.DataCount, maxVecSize);
    }
  }
  
}
//sensor-----------------------------------------------------------


void lightDataCollector(){
  
  if (strlen(g_lightSensorData.SensorName)==0)
  {
  strcpy( g_lightSensorData.SensorName, String("LightSensor").c_str() );
  }
  if (g_lightSensorData.DataCount < maxVecSize)
  {
    g_lightSensorData.vec[g_lightSensorData.DataCount] = static_cast<float>(map(analogRead(LIGHTPIN), 0, 800, 0, 10));
    Serial.printf("%f\n",g_lightSensorData.vec[g_lightSensorData.DataCount]);
    ++g_lightSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i < %i\n",g_lightSensorData.DataCount, maxVecSize);
    }
  }
}


//sensor-----------------------------------------------------------
void DataCollectionTrigger()
{
    uint32_t now = millis();
    if (now >= (g_oldSensorTimer + SENSOR_RATE))
    {
      rndDataCollector();
      lightDataCollector();
      g_oldSensorTimer = millis();
      updateParticleCloud();
    }
}

void setup() {  
  Serial.begin(9600);
  Sha256.init();
  randomSeed(analogRead(0));
  pinMode(g_led, OUTPUT);
  Particle.variable("sensor_RndSensor", RndSensorVar);
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
