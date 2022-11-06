#include "secrets.h"
#include <PubSubClient.h>
#include <ArduinoJson.h> //parse received topics
#include "WiFi.h"
#include "esp_camera.h"
#include "base64.h"
#include <libb64/cencode.h>
#include "mbedtls/base64.h"
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
// use for no hardreset at the fist loop
bool newstart = 0;

// ==================== ==================== CAMERA PINOUT
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==================== ==================== TEMPERATURE
#include "OneWire.h" 
#include "DallasTemperature.h"  
#define ONE_WIRE_BUS 13         
#define __DS18B20 

// ==================== ==================== MQTT CONFIG
#define MQTT_IOT_PUBLISH_TOPIC   "esp32/sensors"
#define MQTT_IOT_TEMP1 "esp32/sensor/temp_a1"
#define MQTT_IOT_TEMP2 "esp32/sensor/temp_a2"
#define MQTT_IOT_TEMP3 "esp32/sensor/temp_a3"
#define MQTT_IOT_TEMP4 "esp32/sensor/temp_a4"
#define MQTT_IOT_RSSI "esp32/sensor/rssi_a1"
#define MQTT_IOT_SUBSCRIBE_TOPIC "esp32/sub"
#define ESP32CAM_PUBLISH_TOPIC   "esp32/cam_0"
#define DEBUG                  1
#define CLIENT_NAME "ESP32CAM-"

bool flash;
bool useMQTT = true;
const char* mqttServer = "192.168.10.1";
const char* HostName = "ESP32-cam";
const char* mqttUser = "";
const char* mqttPassword = "";
const char* topic_IMG = "esp32/IMG";
const char* topic_PUBLISH = "esp32/capture";
const int MAX_PAYLOAD = 250000;
const int bufferSize = 1024 * 43; // 23552 bytes

int total_ds18b20_devices;
DeviceAddress sensor_address;
OneWire oneWire_DS18B20(ONE_WIRE_BUS);
DallasTemperature _ds18b20_(&oneWire_DS18B20);

WiFiClient net;
PubSubClient client(net);

void reconnect() {
  Serial.println("mqtt reconnect start");
  if (!client.connected()) {
    if ( WiFi.status() != WL_CONNECTED )
        {
          WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
          Serial.println(".....");
          delay(5000); // give time to open up the serial monitor
          WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
          WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
          WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
        }
    if ( newstart == 0 )
        {
          hard_restart();
        }  
    Serial.println("Attempting MQTT connection...");
    client.setServer(mqttServer, 1883);
    client.setBufferSize (MAX_PAYLOAD); //This is the maximum payload length
    client.setCallback(callback);
    client.setKeepAlive(300);
    client.setSocketTimeout(180);
    if (client.connect(CLIENT_NAME)) {
      Serial.println("connected");
      client.publish("esp32/sensor", "hello world");
      client.subscribe(MQTT_IOT_SUBSCRIBE_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" wait 0.5 seconds");
      delay(500);
    }
  }

  Serial.println("mqtt reconnect ende");
}

void hard_restart() {
  esp_task_wdt_init(1,true);
  esp_task_wdt_add(NULL);
  while(true);
}

void callback(String topic, byte* message, unsigned int length) {
  String messageTemp;
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  if (topic == topic_IMG) {
    grabImage();
  }
}

// ==================== ==================== READING SENSORS ==================== ==================== 
float read_ds18b20(uint8_t index){
  _ds18b20_.requestTemperatures();
  float temp = _ds18b20_.getTempCByIndex(index);
  return temp;
}

// ==================== ==================== PUBLISH SENSORS  ==================== ====================
void publishSensors()
{
  StaticJsonDocument<200> doc;
  // ====================================== TEMP 1
  float ds18b20_temp_a1 = read_ds18b20(0);
  delay(1000);
  Serial.print("Temp  1: ");
  Serial.print(String(ds18b20_temp_a1));
  Serial.print("C");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_temp_a1 =  client.publish(MQTT_IOT_TEMP1, String(ds18b20_temp_a1, 2).c_str());
  Serial.println(result_temp_a1);

  // ====================================== TEMP 2
  float ds18b20_temp_a2 = read_ds18b20(1);
  delay(1000);
  Serial.print("Temp  2: ");
  Serial.print(String(ds18b20_temp_a2));
  Serial.print("C");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_temp_a2 =  client.publish(MQTT_IOT_TEMP2, String(ds18b20_temp_a2, 2).c_str());
  Serial.println(result_temp_a2);

  // ====================================== TEMP 3
  float ds18b20_temp_a3 = read_ds18b20(2);
  delay(1000);
  Serial.print("Temp  3: ");
  Serial.print(String(ds18b20_temp_a3));
  Serial.print("C");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_temp_a3 =  client.publish(MQTT_IOT_TEMP3, String(ds18b20_temp_a3, 2).c_str());
  Serial.println(result_temp_a3);

  // ====================================== TEMP 3
  float ds18b20_temp_a4 = read_ds18b20(3);
  delay(1000);
  Serial.print("Temp  4: ");
  Serial.print(String(ds18b20_temp_a4));
  Serial.print("C");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_temp_a4 =  client.publish(MQTT_IOT_TEMP4, String(ds18b20_temp_a4, 2).c_str());
  Serial.println(result_temp_a4);

  // ====================================== WIFI RSSI
  float esp32cam_rssi = WiFi.RSSI();
  delay(1000);
  Serial.print("Connection RSSI  : ");
  Serial.print(String(esp32cam_rssi));
  Serial.print("dB");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_rssi =  client.publish(MQTT_IOT_RSSI, String(esp32cam_rssi, 2).c_str());
  Serial.println(result_rssi);

  // ====================================== WRITE TO DOCUMENT
  doc["temp_a1"] = String(ds18b20_temp_a1);
  doc["temp_a2"] = String(ds18b20_temp_a2);
  doc["temp_a3"] = String(ds18b20_temp_a3);
  doc["temp_a4"] = String(ds18b20_temp_a4);
  doc["rssi"] = String(esp32cam_rssi);
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  bool result =  client.publish(MQTT_IOT_PUBLISH_TOPIC, jsonBuffer);
  delay(1000);
}

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}


// ==================== ==================== CAMERA INIT
void cameraInit(){
  Serial.println("\nInitializing Camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; // 640x480
  config.jpeg_quality = 5;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf(", failed 0x%x", err);
    ESP.restart();
    return;
  }
  Serial.println("Camera initialization complete.");
  
}
void grabImage() {
  camera_fb_t * fb = NULL;
  Serial.println("Taking picture");
  fb = esp_camera_fb_get(); // used to get a single picture.
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  Serial.println("Picture taken");
  sendMQTT(fb->buf, fb->len);
  esp_camera_fb_return(fb); // must be used to free the memory allocated by esp_camera_fb_get().
  
}

void sendMQTT(const uint8_t * buf, uint32_t len){
  Serial.println("Sending picture...");
  if(len>MAX_PAYLOAD){
    Serial.println("LEN:");
    Serial.println(String(len));
    Serial.println("Picture too large, increase the MAX_PAYLOAD value");
  }else{
    Serial.print("LEN:\t");
    Serial.println(String(len));
    Serial.print("Picture sent ? :\t");
    Serial.println(client.publish(ESP32CAM_PUBLISH_TOPIC, buf, len, false));
    delay(1000);
  }
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print("WiFi connected, IP: ");
  Serial.print(WiFi.localIP());
  Serial.println(" ");
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.print("Disconnected from WiFi access point, reason:");
  Serial.println(info.disconnected.reason);
  Serial.print("Connecting to: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(2000);
  reconnect();
}

// ==================== ==================== SETUP
void setup() {
  newstart = 1;
  Serial.begin(115200);
  Serial.print("Initialize sensors");
  _ds18b20_.begin();
  // initialize camera
  cameraInit();
  Serial.println("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println(".....");
  delay(5000); // give time to open up the serial monitor
  WiFi.onEvent(WiFiStationConnected, SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  
  
}

void loop() {
  if(!client.connected()) reconnect();
  if(client.connected()) grabImage();
  if(client.connected()) publishSensors();
  newstart = 0;
  client.loop();
  Serial.println("==== 60s Loop, No Sleep =====================\n\n");
  delay(60000);
}