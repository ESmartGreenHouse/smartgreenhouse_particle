/*
 * Project ISD
 * Description:
 * Author:
 * Date:
 */
#include "Particle.h"
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

#define CLOUD_RATE 60000
#define DATA_PER_MINUTE maxVecSize
#define SENSOR_RATE CLOUD_RATE/DATA_PER_MINUTE
#define HEARTBEAT_RATE 1000
#define TIME_BETWEEN_RULE_ACTIONS 5000

uint32_t g_oldHeartbeatTimer=0;
uint32_t g_oldSensorTimer=0;
uint32_t g_oldCloudPushTime=0;

uint8_t g_STATE;
bool g_on_off = true;
uint32_t g_led = D7;

String g_deviceID= System.deviceID();
const char *PUBLISH_EVENT_NAME = "test1data";

/////------Rules Section--------------

//---------var State-------------------

bool g_WindowIsClosedByRuleState = false;
bool g_WindowState = false;
bool g_IrrigationState = false;
bool g_LightState = false;
bool g_RainingState = false;
bool g_HighWindState = false;

//-------- Threshhold var--------------

double thresh_LowMoisture = 0.0;
double thresh_IndoorTemp = 0.0;
double thresh_IndoorHum = 0.0;
double thresh_Raining = 0.0;
double thresh_HighWind = 0.0;
double thresh_DayLight = 0.0;

//-------- last actions timestamps------

unsigned long last_LightAction =0;
unsigned long last_WindowAction =0;
unsigned long last_IrrigationAction =0;

/////------Rules Section END-----------

//---------Sensors dec-----------------

#define LIGHTPIN A0

Data g_lightSensorData;
Data g_rndSensorData;
Data g_WindSensorData;
Data g_MoistureSensorData;
Data g_RainingSensorData;
Data g_HumIndoorSensorData;
Data g_HumOutdoorSensorData;
Data g_TempIndoorSensorData;
Data g_TempOutdoorSensorData;

double RndSensorVar= 0.0;

double LightSensorVar= 0.0;

double WindSensorVar= 0.0;

double MoistureSensorVar= 0.0;

double RainingSensorVar = 0.0;

double HumIndoorSensorVar= 0.0;
double TempIndoorSensorVar= 0.0;

double HumOutdoorSensorVar= 0.0;
double TempOutdoorSensorVar= 0.0;

//----------Sensors end----------------

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

//sensor-----------------------------------------------------------

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
    ++g_rndSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_rndSensorData.DataCount, maxVecSize);
    }
  }
  
}

void lightDataCollector(){
  
  if (strlen(g_lightSensorData.SensorName)==0)
  {
  strcpy( g_lightSensorData.SensorName, String("LightSensor").c_str() );
  }
  if (g_lightSensorData.DataCount < maxVecSize)
  {
    LightSensorVar =  static_cast<float>(map(analogRead(LIGHTPIN), 0, 800, 0, 10));
    g_lightSensorData.vec[g_lightSensorData.DataCount] = LightSensorVar;
    ++g_lightSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_lightSensorData.DataCount, maxVecSize);
    }
  }
}

//sensor-----------------------------------------------------------

//rules -----------------------------------------------------------------

void rule_light(){
  unsigned long time = millis();
  if (time - last_LightAction > TIME_BETWEEN_RULE_ACTIONS){
    // an action can be triggered only every 5000 ms
    if (LightSensorVar > thresh_DayLight)
    g_LightState = false;
    else g_LightState = true;
    last_LightAction = time;
  }
}

void rule_closeWindow(){
  unsigned long time = millis();
  if (time - last_WindowAction > TIME_BETWEEN_RULE_ACTIONS) {
    // an action can be triggered only every 5000 ms
    if (WindSensorVar > thresh_HighWind || RainingSensorVar > thresh_Raining)
    g_WindowIsClosedByRuleState = true;
    else g_WindowIsClosedByRuleState = false;
    last_WindowAction = time;
  }
}

void rule_openWindow(){
  unsigned long time = millis();
  if (time - last_WindowAction > TIME_BETWEEN_RULE_ACTIONS) {
    // an action can be triggered only every 5000 ms
    if (!g_WindowIsClosedByRuleState && (TempIndoorSensorVar < TempOutdoorSensorVar) && (HumIndoorSensorVar < HumOutdoorSensorVar))
    g_WindowState = true;
    else g_WindowState = false;
    last_WindowAction = time;
  }
}

void rule_irrigation(){
  unsigned long time = millis();
  if (time - last_IrrigationAction > TIME_BETWEEN_RULE_ACTIONS){
    // an action can be triggered only every 5000 ms
    if (MoistureSensorVar < thresh_LowMoisture)
    g_IrrigationState = true;
    else g_IrrigationState = false;
    last_IrrigationAction = time;
  }
}

//rules -----------------------------------------------------------------


//setter functions ------------------------------------------------------

int set_thresh_LowMoisture(String val)
{
  thresh_LowMoisture = atof(val.c_str());
  return 0;
}
int set_thresh_IndoorTemp(String val)
{
  thresh_IndoorTemp = atof(val.c_str());
  return 0;
}
int set_thresh_IndoorHum(String val)
{
  thresh_IndoorHum = atof(val.c_str());
  return 0;
}
int set_thresh_Raining(String val)
{
  thresh_Raining = atof(val.c_str());
  return 0;
}
int set_thresh_HighWind(String val)
{
  thresh_HighWind = atof(val.c_str());
  return 0;
}
int set_thresh_DayLight(String val)
{
  thresh_DayLight = atof(val.c_str());
  return 0;
}

//setter functions ------------------------------------------------------


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
  randomSeed(analogRead(0));
  pinMode(g_led, OUTPUT);

  //---------Sensors dec-----------------
  
  Particle.variable("sensor_RndSensor", RndSensorVar);
  Particle.variable("sensor_LightSensor", LightSensorVar);
  Particle.variable("sensor_RndSensor", WindSensorVar);
  Particle.variable("sensor_LightSensor", RainingSensorVar);
  Particle.variable("sensor_RndSensor", MoistureSensorVar);

  Particle.variable("sensor_LightSensor", HumIndoorSensorVar);
  Particle.variable("sensor_LightSensor", HumOutdoorSensorVar);

  Particle.variable("sensor_LightSensor", TempIndoorSensorVar);
  Particle.variable("sensor_LightSensor", TempOutdoorSensorVar);
  //---------Sensors end-----------------

  Particle.function("set_thresh_LowMoisture", set_thresh_LowMoisture);
  Particle.function("set_thresh_IndoorTemp", set_thresh_IndoorTemp);
  Particle.function("set_thresh_IndoorHum", set_thresh_IndoorHum);
  Particle.function("set_thresh_Raining", set_thresh_Raining);
  Particle.function("set_thresh_HighWind", set_thresh_HighWind);
  Particle.function("set_thresh_DayLight", set_thresh_DayLight);

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
