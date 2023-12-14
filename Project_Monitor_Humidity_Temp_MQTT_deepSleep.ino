#include <WiFi.h>
#include <WiFiMulti.h>
#include <ArduinoJson.h>  //ArduinoJson by Benoit Blanchon Version 6.13.0
#include <MQTT.h>         //MQTT by Joel Gaehwiler Version 2.4.7
#include "DHTesp.h"       // Click here to get the library: http://librarymanager/All#DHTesp
#include <Ticker.h>
#include <ESP32Time.h>

//ESPTimer Params
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  180        /* Time ESP32 will go to sleep (in seconds) */

//Create Object
WiFiMulti wifiMulti;
WiFiClient client;
MQTTClient mqtt(1024);
DHTesp dht;

//MQTT setting
const char* broker = "broker.hivemq.com"; //Mosquitto
const char* unique_id = "d0922dff-76cc-418d-833b-2b0f42ad3c14";
String topic_subscribe = "home/pi/control/2";
String topic_publish = "esp32/DHT22/data/calibrate";

// LED Pin
const byte ledPin = 2;
const byte led1Pin22 = 22;

//Global Variable
unsigned long ledTick = 0;

//DHT parameter
void tempTask(void *pvParameters);
bool getTemperature();
void triggerGetTemp();

/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;
/** Ticker for temperature reading */
Ticker tempTicker;
/** Comfort profile */
ComfortState cf;
/** Flag if task should run */
bool tasksEnabled = false;
/** Pin number for DHT11 data pin */
int dhtPin = 25;

/**
   initTemp
   Setup DHT library
   Setup task and timer for repeated measurement
   @return bool
      true if task and timer are started
      false if task or timer couldn't be started
*/
bool initTemp() {
  byte resultValue = 0;
  // Initialize temperature sensor
  dht.setup(dhtPin, DHTesp::DHT22);
  Serial.println("DHT initiated");

  // Start task to get temperature
  xTaskCreatePinnedToCore(
    tempTask,                       /* Function to implement the task */
    "tempTask ",                    /* Name of the task */
    4000,                           /* Stack size in words */
    NULL,                           /* Task input parameter */
    5,                              /* Priority of the task */
    &tempTaskHandle,                /* Task handle. */
    1);                             /* Core where the task should run */

  if (tempTaskHandle == NULL) {
    Serial.println("Failed to start task for temperature update");
    return false;
  } else {
    // Start update of environment data every 20 seconds
    tempTicker.attach(20, triggerGetTemp);
  }
  return true;
}

/**
   triggerGetTemp
   Sets flag dhtUpdated to true for handling in loop()
   called by Ticker getTempTimer
*/
void triggerGetTemp() {
  if (tempTaskHandle != NULL) {
    xTaskResumeFromISR(tempTaskHandle);
  }
}

/**
   Task to reads temperature from DHT11 sensor
   @param pvParameters
      pointer to task parameters
*/
void tempTask(void *pvParameters) {
  Serial.println("tempTask loop started");
  while (1) // tempTask loop
  {
    if (tasksEnabled) {
      // Get temperature values
      getTemperature();
    }
    // Got sleep again
    vTaskSuspend(NULL);
  }
}

/**
   getTemperature
   Reads temperature from DHT11 sensor
   @return bool
      true if temperature could be aquired
      false if aquisition failed
*/
bool getTemperature() {
  // Reading temperature for humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity newValues = dht.getTempAndHumidity();
  // Check if any reads failed and exit early (to try again).
  if (dht.getStatus() != 0) {
    Serial.println("DHT11 error status: " + String(dht.getStatusString()));
    return false;
  }

  float heatIndex = dht.computeHeatIndex(newValues.temperature, newValues.humidity);
  float dewPoint = dht.computeDewPoint(newValues.temperature, newValues.humidity);
  float cr = dht.getComfortRatio(cf, newValues.temperature, newValues.humidity);

  String comfortStatus;
  switch (cf) {
    case Comfort_OK:
      comfortStatus = "Comfort_OK";
      break;
    case Comfort_TooHot:
      comfortStatus = "Comfort_TooHot";
      break;
    case Comfort_TooCold:
      comfortStatus = "Comfort_TooCold";
      break;
    case Comfort_TooDry:
      comfortStatus = "Comfort_TooDry";
      break;
    case Comfort_TooHumid:
      comfortStatus = "Comfort_TooHumid";
      break;
    case Comfort_HotAndHumid:
      comfortStatus = "Comfort_HotAndHumid";
      break;
    case Comfort_HotAndDry:
      comfortStatus = "Comfort_HotAndDry";
      break;
    case Comfort_ColdAndHumid:
      comfortStatus = "Comfort_ColdAndHumid";
      break;
    case Comfort_ColdAndDry:
      comfortStatus = "Comfort_ColdAndDry";
      break;
    default:
      comfortStatus = "Unknown:";
      break;
  };

  StaticJsonDocument<200>data;
  data["temperature"] = newValues.temperature;
  data["humidity"] = newValues.humidity;
  data["heatIndex"] = heatIndex;
  data["dewPoint"] = dewPoint;
  data["comfortStatus"] = comfortStatus;

  //====Debuging Message=====
  serializeJson(data, Serial);
  Serial.println();
  serializeJsonPretty(data, Serial);
  Serial.println();
  //=========================

  String payload;
  serializeJson(data, payload);
  Serial.println(payload);
  mqtt.publish(topic_publish, payload);

  //Serial.println(" T:" + String(newValues.temperature) + " H:" + String(newValues.humidity) + " I:" + String(heatIndex) + " D:" + String(dewPoint) + " " + comfortStatus);
  //Goto sleep mode after publish payload to MQTT
  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
  return true;
}

void wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default :
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}


//User Function - setup_wifi()
//------------------------------------------------
void setup_wifi() {
  delay(10);
  wifiMulti.addAP("tomato", "king@535382+1");
  wifiMulti.addAP("HONOR X9a 5G", "12345678");
  //wifiMulti.addAP("tp-link-deco", "wW7287cC");

  Serial.println("Connecting Wifi...");
  if (wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

//User Function - setup_mqtt()
//------------------------------------------------
void setup_mqtt() {
  while (!mqtt.connect(unique_id)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("MQTT connected.");
  mqtt.subscribe(topic_subscribe);
}

//CALL THIS FUNCTION WHEN RECEIVED MESSAGE FROM MQTT
//-------------------------------------------------------------
void messageReveived(String &topic_subscribe, String &payload) {
  Serial.print("Incoming Topic:");
  Serial.print(topic_subscribe);
  Serial.print(", Payload");
  Serial.println(payload);

  //Decode DATA from JSON format
  //e.g.: {"value":"on"}
  DynamicJsonDocument doc(1024);
  deserializeJson (doc, payload);
  String value = doc["value"];

  Serial.println(value);

  //Application according to JSON Data
  if (value == "on") {
    digitalWrite(led1Pin22, HIGH);
    Serial.println("LED on");
  }
  else if (value == "off") {
    digitalWrite(led1Pin22, LOW);
    Serial.println("LED off");
  }
}
//------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);

  wakeup_reason();

  pinMode(ledPin, OUTPUT);
  pinMode(led1Pin22, OUTPUT);

  mqtt.begin(broker, 1883, client);
  mqtt.onMessage(messageReveived);
  setup_wifi();
  setup_mqtt();

  initTemp();
  tasksEnabled = true;
}

void loop()
{
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    delay(1000);
  }

  mqtt.loop();
  delay(10);
  if (!mqtt.connected()) {
    setup_mqtt();
  }

  if (!tasksEnabled) {
    // Wait 2 seconds to let system settle down
    delay(2000);
    // Enable task that will read values from the DHT sensor
    tasksEnabled = true;
    if (tempTaskHandle != NULL) {
      vTaskResume(tempTaskHandle);
    }
  }
  yield();


  if (millis() > ledTick) {
    ledTick = millis() + 200;
    digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
  }
}
