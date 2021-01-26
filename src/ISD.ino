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
#include "DHT.h"  //Please install "Adafruit_Sensor" library as well (INclusion in "DHT.h" lib file)
#include "Arduino.h" //Necessary for DHT Library to work on Particle

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
#define RAININGPIN D4
#define MOISTPIN A2

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

#define DHTTYPE DHT22
#define DHTPIN_OUTDOOR D2
#define DHTPIN_INDOOR D6
DHT dht_indoor(DHTPIN_INDOOR, DHTTYPE);
DHT dht_outdoor(DHTPIN_OUTDOOR, DHTTYPE);

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
      push_jsonstring_to_cloud(&g_TempIndoorSensorData);
      push_jsonstring_to_cloud(&g_MoistureSensorData);
      push_jsonstring_to_cloud(&g_RainingSensorData);
      push_jsonstring_to_cloud(&g_TempOutdoorSensorData);
      push_jsonstring_to_cloud(&g_HumIndoorSensorData);
      push_jsonstring_to_cloud(&g_HumOutdoorSensorData);

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

void indoorTempDataCollector()
{
  if (strlen(g_TempIndoorSensorData.SensorName)==0)
  {
  strcpy( g_TempIndoorSensorData.SensorName, String("TempIndoorSensor").c_str() );
  }
  if (g_TempIndoorSensorData.DataCount < maxVecSize)
  {
    TempIndoorSensorVar= static_cast<float>(dht_indoor.readTemperature());
    g_TempIndoorSensorData.vec[g_TempIndoorSensorData.DataCount] =TempIndoorSensorVar;
    ++g_TempIndoorSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_TempIndoorSensorData.DataCount, maxVecSize);
    }
  }
  
}

void outdoorTempDataCollector()
{
  if (strlen(g_TempOutdoorSensorData.SensorName)==0)
  {
  strcpy( g_TempOutdoorSensorData.SensorName, String("TempOutdoorSensor").c_str() );
  }
  if (g_TempOutdoorSensorData.DataCount < maxVecSize)
  {
    TempOutdoorSensorVar = static_cast<float>(dht_outdoor.readTemperature());
    g_TempOutdoorSensorData.vec[g_TempOutdoorSensorData.DataCount] =TempOutdoorSensorVar;
    ++g_TempOutdoorSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_TempOutdoorSensorData.DataCount, maxVecSize);
    }
  }
  
}

void outdoorHumDataCollector()
{
  if (strlen(g_HumOutdoorSensorData.SensorName)==0)
  {
  strcpy( g_HumOutdoorSensorData.SensorName, String("HumOutdoorSensor").c_str() );
  }
  if (g_HumOutdoorSensorData.DataCount < maxVecSize)
  {
    HumOutdoorSensorVar= static_cast<float>(dht_outdoor.readHumidity());
    g_HumOutdoorSensorData.vec[g_HumOutdoorSensorData.DataCount] =HumOutdoorSensorVar;
    ++g_HumOutdoorSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_HumOutdoorSensorData.DataCount, maxVecSize);
    }
  }
  
}

void indoorHumDataCollector()
{
  if (strlen(g_HumIndoorSensorData.SensorName)==0)
  {
  strcpy( g_HumIndoorSensorData.SensorName, String("HumIndoorSensor").c_str() );
  }
  if (g_HumIndoorSensorData.DataCount < maxVecSize)
  {
    HumIndoorSensorVar= static_cast<float>(dht_indoor.readHumidity());
    g_HumIndoorSensorData.vec[g_HumIndoorSensorData.DataCount] =HumIndoorSensorVar;
    ++g_HumIndoorSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_HumIndoorSensorData.DataCount, maxVecSize);
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

void RainingDataCollector(){
  
  if (strlen(g_RainingSensorData.SensorName)==0)
  {
  strcpy( g_RainingSensorData.SensorName, String("RainingSensor").c_str() );
  }
  if (g_RainingSensorData.DataCount < maxVecSize)
  {
    RainingSensorVar =  static_cast<float>(!analogRead(RAININGPIN));
    g_RainingSensorData.vec[g_RainingSensorData.DataCount] = RainingSensorVar;
    ++g_RainingSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_RainingSensorData.DataCount, maxVecSize);
    }
  }
}

void MoistureDataCollector(){
  
  if (strlen(g_MoistureSensorData.SensorName)==0)
  {
  strcpy( g_MoistureSensorData.SensorName, String("MoistureSensor").c_str() );
  }
  if (g_MoistureSensorData.DataCount < maxVecSize)
  {
    MoistureSensorVar =  static_cast<float>(map(3300*analogRead(MOISTPIN)/4096, 0, 1200, 0, 100));
    g_MoistureSensorData.vec[g_MoistureSensorData.DataCount] = MoistureSensorVar;
    ++g_MoistureSensorData.DataCount;
  }
  else
  {
    {
      Serial.printf("Size %i > %i\n",g_MoistureSensorData.DataCount, maxVecSize);
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

      indoorHumDataCollector();
      outdoorHumDataCollector();
      indoorTempDataCollector();
      outdoorTempDataCollector();
      RainingDataCollector();
      MoistureDataCollector();

      g_oldSensorTimer = millis();
      updateParticleCloud();
    }
}

void setup() {  
  Serial.begin(9600);
  randomSeed(analogRead(0));
  pinMode(g_led, OUTPUT);

  //---------Sensors --------------------
  dht_indoor.begin();
  dht_indoor.begin();
  Particle.variable("sensor_RndSensor", RndSensorVar);
  Particle.variable("sensor_LightSensor", LightSensorVar);
  Particle.variable("sensor_WindSensor", WindSensorVar);
  Particle.variable("sensor_RainingSensor", RainingSensorVar);
  Particle.variable("sensor_MoistureSensor", MoistureSensorVar);

  Particle.variable("sensor_HumIndoorSensor", HumIndoorSensorVar);
  Particle.variable("sensor_HumOutdoorSensor", HumOutdoorSensorVar);

  Particle.variable("sensor_TempIndoorSensor", TempIndoorSensorVar);
  Particle.variable("sensor_TempOutdoorSensor", TempOutdoorSensorVar);
  //---------Sensors end-----------------

  Particle.variable("thresh_DayLight", thresh_DayLight);
  Particle.variable("thresh_HighWind", thresh_HighWind);
  Particle.variable("thresh_IndoorHum", thresh_IndoorHum);
  Particle.variable("thresh_IndoorTemp", thresh_IndoorTemp);
  Particle.variable("thresh_LowMoisture", thresh_LowMoisture);
  Particle.variable("thresh_Raining", thresh_Raining);

  Particle.variable("state_WindowIsClosedByRuleState",g_WindowIsClosedByRuleState);
  Particle.variable("state_WindowState",g_WindowState);
  Particle.variable("state_IrrigationState",g_IrrigationState);
  Particle.variable("state_LightState",g_LightState);
  Particle.variable("state_RainingState",g_RainingState);
  Particle.variable("state_HighWindState",g_HighWindState);

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
