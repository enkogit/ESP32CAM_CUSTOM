#include "secrets.h"
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h> //parse received topics
#include "WiFi.h"
#include "esp_camera.h"


// camera pinout
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

// deep sleep settings
#define uS_TO_S_FACTOR 1000000
#define TIME_TO_SLEEP 60
RTC_DATA_ATTR int bootCount = 0;


// ds18b20 temperature sensor
#include "OneWire.h" 
#include "DallasTemperature.h"  
#define ONE_WIRE_BUS 12         
#define __DS18B20 

// DHT11 humidity sensor
#include "DHT.h"
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
float _h;
float h;



// The MQTT topics that this device should publish/subscribe
#define MQTT_IOT_PUBLISH_TOPIC   "esp32/sensor/all"
#define MQTT_IOT_TEMP1 "esp32/sensor/temp_a1"
#define MQTT_IOT_TEMP2 "esp32/sensor/temp_a2"
#define MQTT_IOT_HUM1 "esp32/sensor/hum_a1"
#define MQTT_IOT_SUBSCRIBE_TOPIC "esp32/sub"
#define ESP32CAM_PUBLISH_TOPIC   "esp32/cam_0"
#define DEBUG                  1
#define CLIENT_NAME "ESP32CAM"

const int bufferSize = 1024 * 23; // 23552 bytes
int total_ds18b20_devices;
DeviceAddress sensor_address;
OneWire oneWire_DS18B20(ONE_WIRE_BUS);
DallasTemperature _ds18b20_(&oneWire_DS18B20);


WiFiClient net;
MQTTClient client;

// ====================== CONNECT TO MQTT Broker =========================

void connectMQTT()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED){
    delay(100);
    Serial.print(".");
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println("\nWiFi Connected, IP: ");
    Serial.println(WiFi.localIP());
    delay(2000);
  }

  // Connect to the MQTT broker endpoint
  client.begin("192.168.10.1", net);

  // Create a message handler
  client.onMessage(messageHandler);
  Serial.println("\nConnecting to MQTT BROKER");
  

  while (!client.connect(CLIENT_NAME)) {
    Serial.print(".");
    delay(600);
  }

  if(!client.connected()){
    Serial.println("MQTT BROKER Timeout!");
    Serial.println("=========================\n");
    return;
  }

  // Subscribe to a topic
  client.subscribe(MQTT_IOT_SUBSCRIBE_TOPIC);

  Serial.println("MQTT BROKER Connected!");
  Serial.println("=========================\n");
  delay(100);
}

// --------------------------------- READ SENSORS ---------------------------
float read_ds18b20(uint8_t index){
  _ds18b20_.requestTemperatures();
  float temp = _ds18b20_.getTempCByIndex(index);
  return temp;
}

float read_DHT(){
  dht.begin();
  delay(2000);
  float h = dht.readHumidity();
  if (!isnan(h)){
    _h = h;
    return h;
  }else{
    return _h;
  }
  
}

 // --------------------------------- PUBLISH MESSAGE --------------------------
void publishSensors()
{
  StaticJsonDocument<200> doc;
  float ds18b20_temp_a1 = read_ds18b20(0);
  delay(1000);
  Serial.print("Temp  1: ");
  Serial.print(String(ds18b20_temp_a1));
  Serial.print("C");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_temp_a1 =  client.publish(MQTT_IOT_TEMP1, String(ds18b20_temp_a1));
  Serial.println(result_temp_a1);
  
  float ds18b20_temp_a2 = read_ds18b20(1);
  delay(1000);
  Serial.print("Temp  2: ");
  Serial.print(String(ds18b20_temp_a2));
  Serial.print("C");
  Serial.print("\t Publishing Sensor\tresult:\t");
  bool result_temp_a2 =  client.publish(MQTT_IOT_TEMP2, String(ds18b20_temp_a2));
  Serial.println(result_temp_a2);

  float DHT11_hum_a1 = read_DHT();
  delay(1000);
  Serial.print("Hum   1:  ");
  Serial.print(String(DHT11_hum_a1));
  Serial.print("%");
  Serial.print("\t\t Publishing Sensor\tresult:\t");
  bool result_hum_a1 =  client.publish(MQTT_IOT_HUM1, String(DHT11_hum_a1));
  Serial.println(result_hum_a1);
  
  doc["temp_a2"] = String(ds18b20_temp_a1);
  doc["temp_a2"] = String(ds18b20_temp_a2);
  doc["hum_a1"] = String(DHT11_hum_a1);
  
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client
  bool result =  client.publish(MQTT_IOT_PUBLISH_TOPIC, jsonBuffer);
  delay(1000);
}

void messageHandler(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);

//  StaticJsonDocument<200> doc;
//  deserializeJson(doc, payload);
//  const char* message = doc["message"];
}


// ------------------------------ CAMERA INIT --------------------------
void cameraInit(){
  Serial.print("\nInitializing Camera...");
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
  config.jpeg_quality = 10;
  config.fb_count = 2;
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf(", failed 0x%x", err);
    ESP.restart();
    return;
  }
  Serial.println(", done...");
  
}

void grabImage(){
  
  camera_fb_t * fb = esp_camera_fb_get();
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize){
    Serial.print("Taking Image. Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publishing Image\tresult:\t");
    delay(1000);
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char*)fb->buf, fb->len, false);
//    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char*)fb->buf, fb->len);
    delay(3000);
    Serial.println(result);

    if(!result){
      ESP.restart();
    }
  }
  esp_camera_fb_return(fb);
  delay(1);
}


// ------------------------------ SETUP 

void setup() {
  Serial.begin(115200);
  _ds18b20_.begin();
  delay(2000); // give time to open up the serial monitor
  
  Serial.println("\n\n=========================");
  Serial.println("waking up, been sleeping for: " + String(TIME_TO_SLEEP) + "s");
  total_ds18b20_devices = _ds18b20_.getDeviceCount();
  
  Serial.print("Devices found: ");
  Serial.print(total_ds18b20_devices, DEC);
  Serial.print(" ");
  
  // set wakeup interval
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  // increment the boot number and print it every boot
  ++bootCount;
  Serial.println("Boot count number: " + String(bootCount));

  // wakeup reason
  print_wakeup_reason();
  
  //connect to wifi and broker
  connectMQTT();
  publishSensors();
  delay(1000);
  
  // initialize camera and publish message
  cameraInit();
  delay(1000);
  if(client.connected()) grabImage();
  
  
  client.loop();
  
  Serial.println("\nfinished cycle, entering deep sleep for: "  + String(TIME_TO_SLEEP) + "s");
  Serial.println("=========================\n\n");

  //hush little baby
  esp_deep_sleep_start();
}

void loop() {

}

void print_wakeup_reason(){
  // function prints wakeup reason for the esp32
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by a timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was NOT caused by deep sleep"); break;
  }
  
}