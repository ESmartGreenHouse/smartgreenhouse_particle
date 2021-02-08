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
#include "multi_channel_relay.h"

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
#define DATA_DEBOUNCE 1000

uint32_t g_oldHeartbeatTimer=0;
uint32_t g_oldSensorTimer=0;
uint32_t g_oldCloudPushTime=0;
uint32_t g_oldDataSendTimer=0;

uint8_t g_STATE;
uint8_t g_DataSendIterator=0;
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

bool g_lastWindowState = false;
bool g_lastLightState = false;
bool g_lastIrrigationState = false;

//-------- Threshhold var--------------

double thresh_LowMoisture = 1.0;
double thresh_IndoorTemp = 0.0;
double thresh_IndoorHum = 0.0;
double thresh_Raining = 1.0;
double thresh_HighWind = 10.0;
double thresh_DayLight = 20.0;

//-------- last actions timestamps------

unsigned long last_LightAction =0;
unsigned long last_WindowcloseAction =0;
unsigned long last_WindowopenAction =0;
unsigned long last_IrrigationAction =0;

/////------Rules Section END-----------

//---------Relays---------------------------
Multi_Channel_Relay Relays;
const uint8_t ChannelWindow = 1;
const uint8_t ChannelLight = 4;
const uint8_t ChannelIrrigation = 3;
//---------Relays END----------------------

//---------Sensors dec-----------------

#define LIGHTPIN A0
#define RAININGPIN D4
#define MOISTPIN A4
#define WINDPIN D5 //interrupt pin

Data g_lightSensorData;
Data g_WindSensorData;
Data g_MoistureSensorData;
Data g_RainingSensorData;
Data g_HumIndoorSensorData;
Data g_HumOutdoorSensorData;
Data g_TempIndoorSensorData;
Data g_TempOutdoorSensorData;


double LightSensorVar= 0.0;

double MoistureSensorVar= 0.0;

double RainingSensorVar = 0.0;

double HumIndoorSensorVar= 0.0;
double TempIndoorSensorVar= 0.0;

double HumOutdoorSensorVar= 0.0;
double TempOutdoorSensorVar= 0.0;

int WindCounter = 0;
double WindSensorVar = 0.0;
unsigned long WindTime = 0;

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
  Particle.publish(Datafield->SensorName, jsonString, PRIVATE);
  Serial.printf(jsonString+"\n");
  Datafield->DataCount =0;
}

bool updateParticleCloud() 
{
    uint32_t now = millis();
    if (now >= (g_oldCloudPushTime + CLOUD_RATE))
    {
      if (now >= (g_oldDataSendTimer + DATA_DEBOUNCE))
      {
        g_DataSendIterator++;
        g_oldDataSendTimer = millis();

        // Serial.printf("Data Iterator: %i, Time: %d\n", g_DataSendIterator, int(g_oldDataSendTimer));
        
        //the switch statement should be inside the if statement to avoid pushing to cloud more than once for a new g_DataSendITerator
        switch (g_DataSendIterator) {
        case 1: {
        push_jsonstring_to_cloud(&g_TempIndoorSensorData);
        }; break;
        case 2: {
        push_jsonstring_to_cloud(&g_TempOutdoorSensorData);
        }; break;
        case 3: {
        push_jsonstring_to_cloud(&g_HumIndoorSensorData);
        }; break;
        case 4: {
        push_jsonstring_to_cloud(&g_HumOutdoorSensorData);
        }; break;
        case 5: {
        push_jsonstring_to_cloud(&g_RainingSensorData);
        }; break;
        case 6: {
        push_jsonstring_to_cloud(&g_lightSensorData);
        }; break;
        case 7: {
        push_jsonstring_to_cloud(&g_WindSensorData);
        }; break;
        case 8: {
        push_jsonstring_to_cloud(&g_MoistureSensorData);
        }; break;
        case 9: {
        g_oldCloudPushTime = millis();
        g_DataSendIterator=0;
        }; break;
        };    
      }
    }
  return true;
}

//sensor-----------------------------------------------------------

void send_size_error(Data sensor)
{
  //Each time this message is sent, a data point is lost (This is caused by timing errors due to the DATA_DEBOUNCE rate)
  //However, if the CLOUD_RATE is rather large (~60 s), not a lot of data points will be lost (tested)
  Serial.printf("%s: Data Point Lost: Size %i > %i\n",sensor.SensorName, int(sensor.DataCount), maxVecSize);
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
      // Serial.printf("Size %i > %i\n",g_TempIndoorSensorData.DataCount, maxVecSize);
      send_size_error(g_TempIndoorSensorData);
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
      // Serial.printf("Size %i > %i\n",g_TempOutdoorSensorData.DataCount, maxVecSize);
      send_size_error(g_TempOutdoorSensorData);
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
      // Serial.printf("Size %i > %i\n",g_HumOutdoorSensorData.DataCount, maxVecSize);
      send_size_error(g_HumOutdoorSensorData);
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
      // Serial.printf("Size %i > %i\n",g_HumIndoorSensorData.DataCount, maxVecSize);
      send_size_error(g_HumIndoorSensorData);
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
      // Serial.printf("Size %i > %i\n",g_lightSensorData.DataCount, maxVecSize);
      send_size_error(g_lightSensorData);
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
    RainingSensorVar =  static_cast<float>(!digitalRead(RAININGPIN));
    g_RainingSensorData.vec[g_RainingSensorData.DataCount] = RainingSensorVar;
    ++g_RainingSensorData.DataCount;
  }
  else
  {
    {
      // Serial.printf("Size %i > %i\n",g_RainingSensorData.DataCount, maxVecSize);
      send_size_error(g_RainingSensorData);
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
      // Serial.printf("Size %i > %i\n",g_MoistureSensorData.DataCount, maxVecSize);
      send_size_error(g_MoistureSensorData);
    }
  }
}

void WindCount(){
  WindCounter++;
}

void WindDataCollector(){
  
  if (strlen(g_WindSensorData.SensorName)==0)
  {
  strcpy( g_WindSensorData.SensorName, String("WindSensor").c_str() );
  }
  if (g_WindSensorData.DataCount < maxVecSize)
  {
    unsigned long now = millis();
    if ((now - WindTime) >= 1 * 1000){
      // make a working copy of the counter while disabling interrupts
      cli();
      uint32_t cnt = WindCounter;
      WindCounter = 0;
      sei();
      WindSensorVar = static_cast<float>(float(cnt)/(now-WindTime)*1000);
      WindTime = now;
      // Serial.print("Windsensor: ");
      // Serial.println(WindSensorVar);
    }
    g_WindSensorData.vec[g_WindSensorData.DataCount] = WindSensorVar;
    ++g_WindSensorData.DataCount;
  }
  else
  {
    {
      // Serial.printf("Size %i > %i\n",g_WindSensorData.DataCount, maxVecSize);
      send_size_error(g_WindSensorData);
    }
  }
}

//sensor-----------------------------------------------------------

//rules -----------------------------------------------------------------

void CheckRules() {
  rule_light();
  rule_closeWindow();
  rule_openWindow();
  rule_irrigation();

}

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
  if (time - last_WindowcloseAction > TIME_BETWEEN_RULE_ACTIONS) {
    // an action can be triggered only every 5000 ms
    if (WindSensorVar > thresh_HighWind || RainingSensorVar == thresh_Raining)
    g_WindowIsClosedByRuleState = true;
    else g_WindowIsClosedByRuleState = false;
    last_WindowcloseAction = time;
  }
}

void rule_openWindow(){
  unsigned long time = millis();
  if (time - last_WindowopenAction > TIME_BETWEEN_RULE_ACTIONS) {
    // an action can be triggered only every 5000 ms
    if (!g_WindowIsClosedByRuleState)// && (TempIndoorSensorVar < TempOutdoorSensorVar) && (HumIndoorSensorVar < HumOutdoorSensorVar))
    g_WindowState = true;
    else g_WindowState = false;
    last_WindowopenAction = time;
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
void Switch_one_relay(uint8_t r_no, bool state) {
  if (state) Relays.turn_on_channel(r_no);
  else Relays.turn_off_channel(r_no);
}

void SwitchRelays() {
  if (g_WindowState != g_lastWindowState){
    Switch_one_relay(ChannelWindow, g_WindowState);
    g_lastWindowState = g_WindowState;
  }
  if (g_IrrigationState != g_lastIrrigationState){
    Switch_one_relay(ChannelIrrigation, g_IrrigationState);
    g_lastIrrigationState = g_IrrigationState;
  }
  if (g_LightState != g_lastLightState){
    Switch_one_relay(ChannelLight, g_LightState);
    g_lastLightState = g_LightState;
  }
  // Switch_one_relay(ChannelIrrigation, g_IrrigationState);
  // Switch_one_relay(ChannelLight, g_LightState);
}

void DataCollectionTrigger()
{
    uint32_t now = millis();
    if (now >= (g_oldSensorTimer + SENSOR_RATE))
    {
      lightDataCollector();

      indoorHumDataCollector();
      outdoorHumDataCollector();
      indoorTempDataCollector();
      outdoorTempDataCollector();
      RainingDataCollector();
      MoistureDataCollector();
      WindDataCollector();

      g_oldSensorTimer = millis();
    }
    //should be outside the loop because SENSOR_RATE>DATA_DEBOUNCE
    updateParticleCloud();
}

void setup() {  
  Serial.begin(9600);
  randomSeed(analogRead(0));
  pinMode(g_led, OUTPUT);

  Relays.begin();
  //---------Sensors --------------------
  dht_indoor.begin();
  dht_indoor.begin();
  attachInterrupt(WINDPIN, WindCount, CHANGE);

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
    CheckRules();
    SwitchRelays();
  }; break;
  case State::STATE_RUNNING: {
  }; break;
  case State::STATE_ERROR: {
  }; break;
  };
}
