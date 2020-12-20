#include "ParticleCloud.h"
#include <HTTPClient.h>
#include <CryptoSuite.h>
#undef min
#undef max

ParticleCloud::ParticleCloud() 
{
      
}

bool ParticleCloud::init()
{
    m_deviceHash = generateHashFromMac();
    return true;
}

bool ParticleCloud::connectToFirestore() 
{
    
}

String ParticleCloud::generateHashFromMac() 
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

JsonArray ParticleCloud::packValueType(JsonObject &obj, String ValueType, std::vector<float> data)
{
  int it =0 ;
  JsonArray fields_values_arrayValue_values = obj["values"]["arrayValue"].createNestedArray("values");
  DynamicJsonDocument abc(200);
  JsonObject nested = abc.createNestedObject();

  for (auto i : data) {
    it++;
    nested[ValueType]= i;
    fields_values_arrayValue_values.add(nested);
  }  

  return fields_values_arrayValue_values;
} 

String ParticleCloud::get_JsonStructure(String sensorName, std::vector<float> data) 
{
    String ret;

    DynamicJsonDocument doc(4096);
    JsonObject fields = doc.createNestedObject("fields");
    fields["time"]["stringValue"] = "timestamp";
    fields["min_timestamp"]["timestampValue"] = Time.format(Time.now(), "%Y-%m-%dT%H:%M:00Z");
    fields["particle_id"]["stringValue"] = m_deviceHash;
    fields["sensor"]["stringValue"] = sensorName;
    packValueType(fields,"integerValue",data);
    serializeJson(doc, ret);
    return ret;
}

bool ParticleCloud::update(String sensorName, std::vector<float> data) 
{
    uint32_t now = millis();
    if (now >= (m_oldTime + 60000))
    {
      String jsonString = get_JsonStructure(sensorName,data);
      Particle.publish(PUBLISH_EVENT_NAME, jsonString, PRIVATE);
      m_oldTime = millis();
    }
     return true;
}
