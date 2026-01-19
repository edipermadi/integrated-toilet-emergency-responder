// COMP8041041 Internet of Things (IOT) - Assignment 2
// Integrated Toilet Emergency Responder
// Edi Permadi
// 2702795374

#include <WiFi.h>
#include <DHTesp.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <time.h>

typedef enum triColorLedCombination {
  TRICOLOR_LED_COLOR_NONE,
  TRICOLOR_LED_COLOR_BLUE,
  TRICOLOR_LED_COLOR_GREEN,
  TRICOLOR_LED_COLOR_CYAN,
  TRICOLOR_LED_COLOR_RED,
  TRICOLOR_LED_COLOR_MAGENTA,
  TRICOLOR_LED_COLOR_YELLOW,
  TRICOLOR_LED_COLOR_WHITE,
} triColorLedCombination_t;


typedef enum emergencyResponderIndication {
  EMERGENCY_RESPONDER_INDICATE_NONE,
  EMERGENCY_RESPONDER_INDICATE_STANDBY,
  EMERGENCY_RESPONDER_INDICATE_CRITICAL_RAISED,
  EMERGENCY_RESPONDER_INDICATE_CRITICAL_ACKNOWLEDGED,
  EMERGENCY_RESPONDER_INDICATE_WARNING_RAISED,
  EMERGENCY_RESPONDER_INDICATE_WARNING_ACKNOWLEDGED,
  EMERGENCY_RESPONDER_INDICATE_CANCELLATION,
} emergencyResponderIndication_t;

// tissue dispensing subsystem
static void tissueDispenserMainTask(void *arg);
static void tissueDispenserPIRSensorISR(void);
static void tissueDispenserReloadButtonISR(void);
static void tissueDispenserAdjustButtonISR(void);
static unsigned int resetTissueDispenseCounter(void);
static unsigned int adjustTissueDispenseCountLimit(void);
static triColorLedCombination_t setTissueDispenserLedIndicator(triColorLedCombination_t color);
static TaskHandle_t tissueDispenserMainTaskHandle;
static SemaphoreHandle_t tissueDispenserPIRSensorSemaphore;
static SemaphoreHandle_t tissueDispenserReloadButtonSemaphore;
static SemaphoreHandle_t tissueDispenserAdjustButtonSemaphore;
static QueueSetHandle_t tissueDispenserEventSetQueue;

// emergency responder subsystem
static void emergencyResponderMainTask(void *arg);
static void emergencyResponderIndicatorTask(void *arg);
static void emergencyResponderPanicButtonISR();
static void emergencyResponderAssistanceButtonISR();
static void emergencyResponderResolveButtonISR();
static triColorLedCombination_t setEmergencyResponderLedIndicator(triColorLedCombination_t color);
static TaskHandle_t emergencyResponderMainTaskHandle;
static TaskHandle_t emergencyResponderIndicatorTaskHandle;
static SemaphoreHandle_t emergencyResponderPanicButtonSemaphore;
static SemaphoreHandle_t emergencyResponderAssistanceButtonSemaphore;
static SemaphoreHandle_t emergencyResponderResolveButtonSemaphore;
static SemaphoreHandle_t emergencyResponderAcknowledgeSignalSemaphore;  // acknowledge signal sent from nurse station via MQTT
static QueueSetHandle_t emergencyResponderEventSetQueue;
static QueueHandle_t emergencyResponderIndicationQueue;

// environment monitoring subsystem
static void environmentMonitoringTask(void *arg);
static TaskHandle_t environmentMonitoringTaskHandle;
float convertPollutionToPPM(uint16_t raw);

// MQTT
static void mqttCallback(char *topic, byte *message, unsigned int length);
static void mqttKeepAliveTask(void *arg);
static TaskHandle_t mqttKeepAliveTaskHandle;

// hardware configurations
#define TISSUE_DISPENSER_PIR_SENSOR_PIN 13           // PIR sensor
#define TISSUE_DISPENSER_RELOAD_BUTTON_PIN 12        // tissue reload button
#define TISSUE_DISPENSER_ADJUST_BUTTON_PIN 14        // dispense limit adjust button, last dispense count will be stored as threshold
#define TISSUE_DISPENSER_LED_RED_INDICATOR_PIN 27    // LED indicator
#define TISSUE_DISPENSER_LED_GREEN_INDICATOR_PIN 26  // LED indicator
#define TISSUE_DISPENSER_LED_BLUE_INDICATOR_PIN 25   // LED indicator
#define TISSUE_DISPENSER_MOTOR_RELAY_PIN 33          // relay to activate dispensing motor
#define EMERGENCY_RESPONDER_PANIC_BUTTON_PIN 32
#define EMERGENCY_RESPONDER_ASSISTANCE_BUTTON_PIN 35
#define EMERGENCY_RESPONDER_RESOLVE_BUTTON_PIN 34
#define EMERGENCY_RESPONDER_LED_RED_INDICATOR_PIN 23
#define EMERGENCY_RESPONDER_LED_GREEN_INDICATOR_PIN 22
#define EMERGENCY_RESPONDER_LED_BLUE_INDICATOR_PIN 21
#define DHT_SENSOR_PIN 19
#define MQ2_SENSOR_DIGITAL_OUT_PIN 36
#define MQ2_SENSOR_ANALOG_OUT_PIN 39
#define EXHAUST_FAN_PIN 18

// wifi configuration default values
#define DEFAULT_WIFI_SSID "Wokwi-GUEST"
#define DEFAULT_WIFI_PASSWORD ""

// mqtt configuration default values
#define DEFAULT_MQTT_CLIENT_ID "387ayS5guKgiYAK1clWUzfRl2MD"
#define DEFAULT_MQTT_SERVER_HOST "broker.hivemq.com"
#define DEFAULT_MQTT_SERVER_PORT 1883
#define DEFAULT_MQTT_HOSPITAL_ID "hospital-1"
#define DEFAULT_MQTT_ROOM_ID "room-1"

#define DEFAULT_TISSUE_DISPENSER_PIR_WARM_UP_TIME_MS 30000    // wait 30 seconds until PIR is ready
#define DEFAULT_TISSUE_DISPENSER_DISPENSING_DURATION_MS 5000  // motor will run for 5 seconds to dispend tissue
#define DEFAULT_TISSUE_DISPENSER_COOLDOWN_DURATION_MS 10000   // motor will cool down for 10 seconds after dispensing tissue
#define DEFAULT_TISSUE_DISPENSER_DISPENSE_LIMIT 100           // by default 100 sheets of tissue can be dispensed

#define DEFAULT_POLLUTION_HIGH_THRESHOLD_VALUE 1200     // 1200 ppm to consider harmful air quality
#define DEFAULT_POLLUTION_LOW_THRESHOLD_VALUE 800       // 800 ppm to consider safe air quality
#define DEFAULT_POLLUTION_HIGH_THRESHOLD_COUNT_LIMIT 5  // activate exhaust fan after 5 second
#define DEFAULT_POLLUTION_LOW_THRESHOLD_COUNT_LIMIT 5   // deactivate exhaust fan after 5 seconds

#define DEFAULT_TEMPERATURE_HIGH_THRESHOLD_VALUE 28.0           // 28 C to as upper comfort limit
#define DEFAULT_TEMPERATURE_LOW_THRESHOLD_VALUE 20.0            // 20 C to as lower comfort limit
#define DEFAULT_TEMPERATURE_DISCOMFORT_THRESHOLD_COUNT_LIMIT 5  // activate exhaust fan after 5 seconds
#define DEFAULT_TEMPERATURE_COMFORT_THRESHOLD_COUNT_LIMIT 5     // deactivate exhaust fan after 5 seconds

#define DEFAULT_HUMIDITY_HIGH_THRESHOLD_VALUE 60.0           // 60% humidity to as upper comfort limit
#define DEFAULT_HUMIDITY_LOW_THRESHOLD_VALUE 40.0            // 40% humidity to as lower comfort limit
#define DEFAULT_HUMIDITY_DISCOMFORT_THRESHOLD_COUNT_LIMIT 5  // activate exhaust fan after 5 seconds
#define DEFAULT_HUMIDITY_COMFORT_THRESHOLD_COUNT_LIMIT 5     // deactivate exhaust fan after 5 seconds

typedef struct ledDutyCycle {
  int timeOnCount;
  int timeOffCount;
  int timeOnCountInitValue;
  int timeOffCountInitValue;
} ledDutyCycle_t;

class Hysteresis {
private:
  unsigned int lowCount;
  unsigned int lowCountLimit;
  unsigned int highCount;
  unsigned int highCountLimit;
  bool _state;

public:
  Hysteresis(unsigned int lowCountLimitInput, unsigned int highCountLimitInput) {
    lowCountLimit = lowCountLimitInput;
    highCountLimit = highCountLimitInput;
    reset();
  }

  void update(bool value) {
    if (value == true) {
      if (highCount < highCountLimit) {
        highCount++;
      } else {
        lowCount = 0;
        _state = true;
      }
    } else if (lowCount < lowCountLimit) {
      lowCount++;
    } else {
      highCount = 0;
      _state = false;
    }
  }

  void reset() {
    lowCount = 0;
    highCount = 0;
    _state = false;
  }

  bool state() {
    return _state;
  }
};

// shared instances
static Preferences preferences;
static DHTesp dhtSensor;
static WiFiClient wifiClient;
static PubSubClient mqttClient(wifiClient);
static bool overrideExhaustFanStateToOn;
static bool overrideExhaustFanStateToOff;

// subscription topics
#define MQTT_SUBSCRIPTION_TOPIC_TEMPLATES_COUNT 2
#define MQTT_SUBSCRIPTION_TOPIC_TEMPLATE_EMERGENCY_ACKNOWNEDGE "/events/%s/%s/emergency-acknowledge"
#define MQTT_SUBSCRIPTION_TOPIC_TEMPLATE_EXHAUST_FAN_OVERIDE "/events/%s/%s/override-exhaust-fan"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_EXHAUST_FAN_STATE "/states/%s/%s/exhaust-fan"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_POLLUTION_LEVEL_MEASUREMENT "/measurements/%s/%s/pollution"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_TEMPERATURE_MEASUREMENT "/measurements/%s/%s/temperature"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_HUMIDITY_MEASUREMENT "/measurements/%s/%s/humidity"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_TISSUE_DISPENSER_STATE "/states/%s/%s/tissue-dispenser"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_DISPENSED_TISSUE_MEASUREMENT "/measurements/%s/%s/dispensed-tissue"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_DISPENSED_TISSUE_LIMIT_MEASUREMENT "/measurements/%s/%s/dispensed-tissue-limit"
#define MQTT_PUBLISH_TOPIC_TEMPLATE_EMERGENCY_LEVEL_MEASUREMENT "/measurements/%s/%s/emergency-level"

const char *mqttSubscriptionTopicTemplates[MQTT_SUBSCRIPTION_TOPIC_TEMPLATES_COUNT] = {
  MQTT_SUBSCRIPTION_TOPIC_TEMPLATE_EMERGENCY_ACKNOWNEDGE,
  MQTT_SUBSCRIPTION_TOPIC_TEMPLATE_EXHAUST_FAN_OVERIDE,
};

void setup() {
  Serial.begin(115200);
  dhtSensor.setup(DHT_SENSOR_PIN, DHTesp::DHT22);
  preferences.begin("settings", false);

  // environment air monitoring
  {
    // start environment monitoring task
    environmentMonitoringTaskHandle = NULL;
    xTaskCreate(environmentMonitoringTask, "environment_monitoring", 8192, NULL, tskIDLE_PRIORITY + 5, &environmentMonitoringTaskHandle);
    configASSERT(environmentMonitoringTaskHandle);
  }

  // initialize tissue dispensing
  {
    // start tissue dispensing task
    tissueDispenserMainTaskHandle = NULL;
    xTaskCreate(tissueDispenserMainTask, "tissue_dispenser_main", 8192, NULL, tskIDLE_PRIORITY + 4, &tissueDispenserMainTaskHandle);
    configASSERT(tissueDispenserMainTaskHandle);
  }

  // initialize emergency responder
  {
    // create emergency responder inidication queue
    emergencyResponderIndicationQueue = xQueueCreate(8, sizeof(emergencyResponderIndication_t));
    configASSERT(emergencyResponderIndicationQueue);

    // start emergency responder main task
    emergencyResponderMainTaskHandle = NULL;
    xTaskCreate(emergencyResponderMainTask, "emergency_responder", 8192, NULL, tskIDLE_PRIORITY + 3, &emergencyResponderMainTaskHandle);
    configASSERT(emergencyResponderMainTaskHandle);

    // start emergency responder indicator task
    emergencyResponderIndicatorTaskHandle = NULL;
    xTaskCreate(emergencyResponderIndicatorTask, "emergency_indicator", 8192, NULL, tskIDLE_PRIORITY + 2, &emergencyResponderIndicatorTaskHandle);
    configASSERT(emergencyResponderIndicatorTaskHandle);
  }

  // connect to wifi
  {
    String wifiSsid = preferences.getString("wifi_ssid", DEFAULT_WIFI_SSID);
    String wifiPassword = preferences.getString("wifi_password", DEFAULT_WIFI_PASSWORD);

    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid, wifiPassword, 6);

    Serial.println("connecting to Wifi");

    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    Serial.println("connected to Wifi");

    // use UTC time
    configTime(0, 0, "pool.ntp.org");
  }

  {
    // start environment monitoring task
    mqttKeepAliveTaskHandle = NULL;
    xTaskCreate(mqttKeepAliveTask, "mqtt_keep_alive", 8192, NULL, tskIDLE_PRIORITY + 1, &mqttKeepAliveTaskHandle);
    configASSERT(mqttKeepAliveTaskHandle);
  }
}

void loop() {
  unsigned long start = millis();

  // check and reconnect to Wifi
  if (!WiFi.isConnected()) {
    Serial.println("reconnecting to Wifi");

    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Wifi connected");
  }

  // sleep 20s when needed
  unsigned long elapsed = millis() - start;
  if (elapsed < 20000) {
    vTaskDelay(pdMS_TO_TICKS(20000 - elapsed));
  }
}

static void tissueDispenserMainTask(void *arg) {
  typedef enum tissueDispenserState {
    TISSUE_DISPENSER_STATE_INIT,
    TISSUE_DISPENSER_STATE_WARMING_UP,
    TISSUE_DISPENSER_STATE_READY,
    TISSUE_DISPENSER_STATE_WAITING_FOR_EVENT,
    TISSUE_DISPENSER_STATE_MOTION_DETECTED,
    TISSUE_DISPENSER_STATE_DISPENSING,
    TISSUE_DISPENSER_STATE_COOLING_DOWN,
    TISSUE_DISPENSER_STATE_EXHAUSTION_DETECTED,
    TISSUE_DISPENSER_STATE_EXHAUSTED,
  } tissueDispenserState_t;

  unsigned long pirWarmUpStartedAt = 0;
  unsigned long dispensingMotorStartedAt = 0;
  unsigned long coolDownStartedAt = 0;
  triColorLedCombination_t ledColor = TRICOLOR_LED_COLOR_NONE;
  unsigned int dispenseCount;
  unsigned int dispenseCountLimit;

  char tissueDispenserStatePublishTopic[64];
  char dispensedTissueCountPublishTopic[64];
  char dispensedTissueCountLimitPublishTopic[64];

  tissueDispenserState_t state = TISSUE_DISPENSER_STATE_INIT;
  for (;;) {
    switch (state) {
      case TISSUE_DISPENSER_STATE_INIT:
        {
          // configure directions
          pinMode(TISSUE_DISPENSER_PIR_SENSOR_PIN, INPUT);
          pinMode(TISSUE_DISPENSER_RELOAD_BUTTON_PIN, INPUT);
          pinMode(TISSUE_DISPENSER_ADJUST_BUTTON_PIN, INPUT);
          pinMode(TISSUE_DISPENSER_LED_RED_INDICATOR_PIN, OUTPUT);
          pinMode(TISSUE_DISPENSER_LED_GREEN_INDICATOR_PIN, OUTPUT);
          pinMode(TISSUE_DISPENSER_LED_BLUE_INDICATOR_PIN, OUTPUT);
          pinMode(TISSUE_DISPENSER_MOTOR_RELAY_PIN, OUTPUT);

          // force reset outputs
          digitalWrite(TISSUE_DISPENSER_LED_RED_INDICATOR_PIN, LOW);
          digitalWrite(TISSUE_DISPENSER_LED_GREEN_INDICATOR_PIN, LOW);
          digitalWrite(TISSUE_DISPENSER_LED_BLUE_INDICATOR_PIN, LOW);
          digitalWrite(TISSUE_DISPENSER_MOTOR_RELAY_PIN, LOW);

          // initialize button and sensor semaphores
          tissueDispenserPIRSensorSemaphore = xSemaphoreCreateBinary();
          configASSERT(tissueDispenserPIRSensorSemaphore);
          tissueDispenserReloadButtonSemaphore = xSemaphoreCreateBinary();
          configASSERT(tissueDispenserReloadButtonSemaphore);
          tissueDispenserAdjustButtonSemaphore = xSemaphoreCreateBinary();
          configASSERT(tissueDispenserAdjustButtonSemaphore);

          // create semaphore watch list, then add those semaphores above
          tissueDispenserEventSetQueue = xQueueCreateSet(3);
          configASSERT(tissueDispenserEventSetQueue);
          xQueueAddToSet(tissueDispenserPIRSensorSemaphore, tissueDispenserEventSetQueue);
          xQueueAddToSet(tissueDispenserReloadButtonSemaphore, tissueDispenserEventSetQueue);
          xQueueAddToSet(tissueDispenserAdjustButtonSemaphore, tissueDispenserEventSetQueue);

          // set pin interrupt for buttons
          attachInterrupt(digitalPinToInterrupt(TISSUE_DISPENSER_PIR_SENSOR_PIN), tissueDispenserPIRSensorISR, RISING);         // PIR is active high, watch for voltage rise
          attachInterrupt(digitalPinToInterrupt(TISSUE_DISPENSER_RELOAD_BUTTON_PIN), tissueDispenserReloadButtonISR, FALLING);  // reload button is active low, watch for voltage fall
          attachInterrupt(digitalPinToInterrupt(TISSUE_DISPENSER_ADJUST_BUTTON_PIN), tissueDispenserReloadButtonISR, FALLING);  // adjust button is active low, watch for voltage fall

          // clear topic buffers
          memset(tissueDispenserStatePublishTopic, 0, sizeof(tissueDispenserStatePublishTopic));
          memset(dispensedTissueCountPublishTopic, 0, sizeof(dispensedTissueCountPublishTopic));
          memset(dispensedTissueCountLimitPublishTopic, 0, sizeof(dispensedTissueCountLimitPublishTopic));

          // derive topics
          String hospitalId = preferences.getString("mqtt_hospital_id", DEFAULT_MQTT_HOSPITAL_ID);
          String roomId = preferences.getString("mqtt_room_id", DEFAULT_MQTT_ROOM_ID);
          snprintf(tissueDispenserStatePublishTopic, sizeof(tissueDispenserStatePublishTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_TISSUE_DISPENSER_STATE, hospitalId, roomId);
          snprintf(dispensedTissueCountPublishTopic, sizeof(dispensedTissueCountPublishTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_DISPENSED_TISSUE_MEASUREMENT, hospitalId, roomId);
          snprintf(dispensedTissueCountLimitPublishTopic, sizeof(dispensedTissueCountLimitPublishTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_DISPENSED_TISSUE_LIMIT_MEASUREMENT, hospitalId, roomId);

          // read stored values
          dispenseCount = preferences.getUInt("dispense_count", 0);
          dispenseCountLimit = preferences.getUInt("dispense_count_limit", DEFAULT_TISSUE_DISPENSER_DISPENSE_LIMIT);

          pirWarmUpStartedAt = millis();
          state = TISSUE_DISPENSER_STATE_WARMING_UP;
        }
        break;

      case TISSUE_DISPENSER_STATE_WARMING_UP:
        {
          unsigned long pirSensorWarmUpTime = millis() - pirWarmUpStartedAt;
          unsigned long pirSensorWarmUpDuration = preferences.getULong("pir_sensor_warmup_time_ms", DEFAULT_TISSUE_DISPENSER_PIR_WARM_UP_TIME_MS);
          if (pirSensorWarmUpTime < pirSensorWarmUpDuration) {
            unsigned long startTime = millis();

            // while waiting for PIR to warm-up, ignore events from PIR sensor
            QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(tissueDispenserEventSetQueue, pdMS_TO_TICKS(100));
            if (activatedSemaphore != NULL) {
              // dispose events
              xSemaphoreTake(activatedSemaphore, 0);
            }

            // keep sleep time around 200 ms since its used to blink the led
            unsigned long elapsed = millis() - startTime;
            if (elapsed < 200) {
              vTaskDelay(pdMS_TO_TICKS(200 - elapsed));
            }

            // blink green to indicate PIR is warming up
            if (ledColor != TRICOLOR_LED_COLOR_NONE) {
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_NONE);
            } else {
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_GREEN);
            }
          } else if (dispenseCount < dispenseCountLimit) {
            state = TISSUE_DISPENSER_STATE_READY;
          } else {
            state = TISSUE_DISPENSER_STATE_EXHAUSTION_DETECTED;
          }
        }
        break;

      case TISSUE_DISPENSER_STATE_READY:
        {
          // switch on green led to indicate it's ready
          ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_GREEN);

          // broadcast tissue dispenser ready state
          mqttClient.publish(tissueDispenserStatePublishTopic, "ready");

          state = TISSUE_DISPENSER_STATE_WAITING_FOR_EVENT;
        }
        break;

      case TISSUE_DISPENSER_STATE_WAITING_FOR_EVENT:
        {
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(tissueDispenserEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore == tissueDispenserPIRSensorSemaphore) {
            // motion detected, activate motor to dispense tissue
            if (xSemaphoreTake(tissueDispenserPIRSensorSemaphore, 0)) {
              state = TISSUE_DISPENSER_STATE_MOTION_DETECTED;
            }
          } else if (activatedSemaphore == tissueDispenserReloadButtonSemaphore) {
            // reload button pressed, reset dispense count
            if (xSemaphoreTake(tissueDispenserReloadButtonSemaphore, 0) == pdTRUE) {
              dispenseCount = resetTissueDispenseCounter();
              mqttClient.publish(dispensedTissueCountPublishTopic, String(dispenseCount).c_str());
            }
          } else if (activatedSemaphore == tissueDispenserAdjustButtonSemaphore) {
            // adjust button pressed, store current dispense count as limit with minimum of 10
            if (xSemaphoreTake(tissueDispenserAdjustButtonSemaphore, 0) == pdTRUE) {
              dispenseCountLimit = adjustTissueDispenseCountLimit();
              mqttClient.publish(dispensedTissueCountLimitPublishTopic, String(dispenseCountLimit).c_str());
            }
          } else {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        break;

      case TISSUE_DISPENSER_STATE_MOTION_DETECTED:
        {
          // check if tissue is still available, threshold is updatable
          if (dispenseCount < dispenseCountLimit) {
            // dispense tissue, switch on blue led to indicate dispensing, then increase dispensed count
            ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_BLUE);
            digitalWrite(TISSUE_DISPENSER_MOTOR_RELAY_PIN, HIGH);

            // update counter
            dispenseCount++;
            preferences.putUInt("dispense_count", dispenseCount);

            // broadcast mqtt events
            mqttClient.publish(tissueDispenserStatePublishTopic, "dispensing");
            mqttClient.publish(dispensedTissueCountPublishTopic, String(dispenseCount).c_str());
            mqttClient.publish(dispensedTissueCountLimitPublishTopic, String(dispenseCountLimit).c_str());

            dispensingMotorStartedAt = millis();
            state = TISSUE_DISPENSER_STATE_DISPENSING;
          } else {
            state = TISSUE_DISPENSER_STATE_EXHAUSTION_DETECTED;
          }
        }
        break;

      case TISSUE_DISPENSER_STATE_DISPENSING:
        {
          unsigned long dispensingMotorUpTime = millis() - dispensingMotorStartedAt;
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(tissueDispenserEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore != NULL) {
            // dispose events
            xSemaphoreTake(activatedSemaphore, 0);
          }

          if (dispensingMotorUpTime < DEFAULT_TISSUE_DISPENSER_DISPENSING_DURATION_MS) {
            vTaskDelay(pdMS_TO_TICKS(100));
          } else {
            digitalWrite(TISSUE_DISPENSER_MOTOR_RELAY_PIN, LOW);
            dispensingMotorStartedAt = 0;

            // switch on yellow led for cooling down blink effect
            ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_YELLOW);

            // broadcast tissue dispenser cooling down state
            mqttClient.publish(tissueDispenserStatePublishTopic, "cooling-down");

            coolDownStartedAt = millis();
            state = TISSUE_DISPENSER_STATE_COOLING_DOWN;
          }
        }
        break;

      case TISSUE_DISPENSER_STATE_COOLING_DOWN:
        {
          unsigned long coolDownTime = millis() - coolDownStartedAt;
          unsigned long coolDownDurationMs = preferences.getULong("dispense_cooldown_period_ms", DEFAULT_TISSUE_DISPENSER_COOLDOWN_DURATION_MS);
          // while waiting for cool down, ignore PIR event
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(tissueDispenserEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore != NULL) {
            // dispose events
            xSemaphoreTake(activatedSemaphore, 0);
          }

          if (coolDownTime < coolDownDurationMs) {
            unsigned long startTime = millis();

            // keep delay at 200 ms for cooldown blinking effect
            unsigned long elapsed = millis() - startTime;
            if (elapsed < 200) {
              vTaskDelay(pdMS_TO_TICKS(200 - elapsed));
            }

            if (ledColor != TRICOLOR_LED_COLOR_NONE) {
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_NONE);
            } else {
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_YELLOW);
            }
          } else if (dispenseCount < dispenseCountLimit) {
            state = TISSUE_DISPENSER_STATE_READY;
          } else {
            state = TISSUE_DISPENSER_STATE_EXHAUSTION_DETECTED;
          }
        }
        break;

      case TISSUE_DISPENSER_STATE_EXHAUSTION_DETECTED:
        {
          // switch on red led to indicate tissue exhaustion
          ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_RED);

          // broadcast tissue dispenser dispensing state
          mqttClient.publish(tissueDispenserStatePublishTopic, "exhausted");

          state = TISSUE_DISPENSER_STATE_EXHAUSTED;
        }
        break;

      case TISSUE_DISPENSER_STATE_EXHAUSTED:
        {
          unsigned long startTime = millis();

          // while waiting for cool down, ignore PIR event
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(tissueDispenserEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore == tissueDispenserPIRSensorSemaphore) {
            if (xSemaphoreTake(tissueDispenserPIRSensorSemaphore, 0) == pdTRUE) {
              // motion event detected when tissue is exhaused, do nothing
            }
          } else if (activatedSemaphore == tissueDispenserReloadButtonSemaphore) {
            if (xSemaphoreTake(tissueDispenserReloadButtonSemaphore, 0) == pdTRUE) {
              // tissue reloaded, reset counter and mark as ready
              dispenseCount = resetTissueDispenseCounter();
              mqttClient.publish(dispensedTissueCountPublishTopic, String(dispenseCount).c_str());

              // switch on green led
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_GREEN);

              // broadcast tissue dispenser ready state
              mqttClient.publish(tissueDispenserStatePublishTopic, "ready");

              state = TISSUE_DISPENSER_STATE_READY;
            }
          } else if (activatedSemaphore == tissueDispenserAdjustButtonSemaphore) {
            if (xSemaphoreTake(tissueDispenserAdjustButtonSemaphore, 0) == pdTRUE) {
              // adjust button pressed, store current dispense count as limit with minimum of 10
              dispenseCountLimit = adjustTissueDispenseCountLimit();
              mqttClient.publish(dispensedTissueCountLimitPublishTopic, String(dispenseCountLimit).c_str());
            }
          }

          if (state == TISSUE_DISPENSER_STATE_EXHAUSTED) {
            // keep delay at 200 ms for exhaustion blinking effect
            unsigned long elapsed = millis() - startTime;
            if (elapsed < 200) {
              vTaskDelay(pdMS_TO_TICKS(200 - elapsed));
            }

            if (ledColor != TRICOLOR_LED_COLOR_NONE) {
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_NONE);
            } else {
              ledColor = setTissueDispenserLedIndicator(TRICOLOR_LED_COLOR_RED);
            }
          }
        }
        break;
    }
  }
}

// tissueDispenserPIRSensorISR will be called when PIR reacted to hand wave
static void tissueDispenserPIRSensorISR(void) {
  if (tissueDispenserPIRSensorSemaphore != NULL) {
    xSemaphoreGiveFromISR(tissueDispenserPIRSensorSemaphore, NULL);
  }
}

// tissueDispenserReloadButtonISR will be called when reload button is pressed
static void tissueDispenserReloadButtonISR(void) {
  if (tissueDispenserReloadButtonSemaphore != NULL) {
    xSemaphoreGiveFromISR(tissueDispenserReloadButtonSemaphore, NULL);
  }
}

// tissueDispenserAdjustButtonISR will be called when adjust button is pressed
static void tissueDispenserAdjustButtonISR(void) {
  if (tissueDispenserAdjustButtonSemaphore != NULL) {
    xSemaphoreGiveFromISR(tissueDispenserAdjustButtonSemaphore, NULL);
  }
}

static unsigned int resetTissueDispenseCounter(void) {
  unsigned int dispenseCount = preferences.getUInt("dispense_count", 0);
  if (dispenseCount > 0) {
    preferences.putUInt("dispense_count", 0);
  }

  return dispenseCount;
}

static unsigned int adjustTissueDispenseCountLimit(void) {
  unsigned int dispenseCount = preferences.getUInt("dispense_count", 0);
  if (dispenseCount > 10) {
    preferences.putUInt("dispense_count_limit", dispenseCount);
    return dispenseCount;
  }


  unsigned int dispenseCountLimit = preferences.getUInt("dispense_count_limit", DEFAULT_TISSUE_DISPENSER_DISPENSE_LIMIT);
  return dispenseCountLimit;
}

static triColorLedCombination_t setTissueDispenserLedIndicator(triColorLedCombination_t color) {
  bool red, green, blue;
  switch (color) {
    case TRICOLOR_LED_COLOR_NONE:
      {
        red = false;
        green = false;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_BLUE:
      {
        red = false;
        green = false;
        blue = true;
      }
      break;

    case TRICOLOR_LED_COLOR_GREEN:
      {
        red = false;
        green = true;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_CYAN:
      {
        red = false;
        green = true;
        blue = true;
      }
      break;

    case TRICOLOR_LED_COLOR_RED:
      {
        red = true;
        green = false;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_MAGENTA:
      {
        red = true;
        green = false;
        blue = true;
      }
      break;

    case TRICOLOR_LED_COLOR_YELLOW:
      {
        red = true;
        green = true;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_WHITE:
      {
        red = true;
        green = true;
        blue = true;
      }
      break;
  }

  digitalWrite(TISSUE_DISPENSER_LED_RED_INDICATOR_PIN, red);
  digitalWrite(TISSUE_DISPENSER_LED_GREEN_INDICATOR_PIN, green);
  digitalWrite(TISSUE_DISPENSER_LED_BLUE_INDICATOR_PIN, blue);
  return color;
}


static void emergencyResponderMainTask(void *arg) {
  typedef enum emergencyResponderState {
    EMERGENCY_RESPONDER_STATE_INIT,
    EMERGENCY_RESPONDER_STATE_STANDBY,
    EMERGENCY_RESPONDER_STATE_URGENCY_RAISED,
    EMERGENCY_RESPONDER_STATE_URGENCY_CANCELED,
    EMERGENCY_RESPONDER_STATE_URGENCY_ACKNOWLEDGED,
    EMERGENCY_RESPONDER_STATE_URGENCY_RESOLVED,
  } emergencyResponderState_t;


  typedef enum emergencyResponderUrgencyLevel {
    EMERGENCY_RESPONDER_URGENCY_LEVEL_NONE,
    EMERGENCY_RESPONDER_URGENCY_LEVEL_WARNING,
    EMERGENCY_RESPONDER_URGENCY_LEVEL_CRITICAL,
  } emergencyResponderUrgencyLevel_t;

  unsigned long standbyStartedAt = 0;
  unsigned long urgencyStartedAt = 0;
  unsigned long urgencyAcknowledgedAt = 0;
  unsigned long cancellationStartedAt = 0;
  emergencyResponderUrgencyLevel_t urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_NONE;
  char emergencyLevelMeasurementPublishTopic[64];

  emergencyResponderState_t state = EMERGENCY_RESPONDER_STATE_INIT;
  for (;;) {
    switch (state) {
      case EMERGENCY_RESPONDER_STATE_INIT:
        {
          // configure directions
          pinMode(EMERGENCY_RESPONDER_PANIC_BUTTON_PIN, INPUT);
          pinMode(EMERGENCY_RESPONDER_ASSISTANCE_BUTTON_PIN, INPUT);
          pinMode(EMERGENCY_RESPONDER_RESOLVE_BUTTON_PIN, INPUT);
          pinMode(EMERGENCY_RESPONDER_LED_RED_INDICATOR_PIN, OUTPUT);
          pinMode(EMERGENCY_RESPONDER_LED_GREEN_INDICATOR_PIN, OUTPUT);
          pinMode(EMERGENCY_RESPONDER_LED_BLUE_INDICATOR_PIN, OUTPUT);

          // force reset outputs
          digitalWrite(EMERGENCY_RESPONDER_LED_RED_INDICATOR_PIN, LOW);
          digitalWrite(EMERGENCY_RESPONDER_LED_GREEN_INDICATOR_PIN, LOW);
          digitalWrite(EMERGENCY_RESPONDER_LED_BLUE_INDICATOR_PIN, LOW);

          // initialize button and sensor semaphores
          emergencyResponderPanicButtonSemaphore = xSemaphoreCreateBinary();
          configASSERT(emergencyResponderPanicButtonSemaphore);
          emergencyResponderAssistanceButtonSemaphore = xSemaphoreCreateBinary();
          configASSERT(emergencyResponderAssistanceButtonSemaphore);
          emergencyResponderAssistanceButtonSemaphore = xSemaphoreCreateBinary();
          configASSERT(emergencyResponderAssistanceButtonSemaphore);
          emergencyResponderResolveButtonSemaphore = xSemaphoreCreateBinary();
          configASSERT(emergencyResponderResolveButtonSemaphore);
          emergencyResponderAcknowledgeSignalSemaphore = xSemaphoreCreateBinary();
          configASSERT(emergencyResponderAcknowledgeSignalSemaphore);

          // create semaphore watch list, then add those semaphores above
          emergencyResponderEventSetQueue = xQueueCreateSet(4);
          configASSERT(emergencyResponderEventSetQueue);
          xQueueAddToSet(emergencyResponderPanicButtonSemaphore, emergencyResponderEventSetQueue);
          xQueueAddToSet(emergencyResponderAssistanceButtonSemaphore, emergencyResponderEventSetQueue);
          xQueueAddToSet(emergencyResponderResolveButtonSemaphore, emergencyResponderEventSetQueue);
          xQueueAddToSet(emergencyResponderAcknowledgeSignalSemaphore, emergencyResponderEventSetQueue);

          // set pin interrupt for buttons
          attachInterrupt(digitalPinToInterrupt(EMERGENCY_RESPONDER_PANIC_BUTTON_PIN), emergencyResponderPanicButtonISR, FALLING);            // panic button is active low, watch for voltage fall
          attachInterrupt(digitalPinToInterrupt(EMERGENCY_RESPONDER_ASSISTANCE_BUTTON_PIN), emergencyResponderAssistanceButtonISR, FALLING);  // assistance button is active low, watch for voltage fall
          attachInterrupt(digitalPinToInterrupt(EMERGENCY_RESPONDER_RESOLVE_BUTTON_PIN), emergencyResponderResolveButtonISR, FALLING);        // resolve button is active low, watch for voltage fall

          // reset urgency level and switch off indicator
          urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_NONE;

          // derive publish topic
          String hospitalId = preferences.getString("mqtt_hospital_id", DEFAULT_MQTT_HOSPITAL_ID);
          String roomId = preferences.getString("mqtt_room_id", DEFAULT_MQTT_ROOM_ID);
          memset(emergencyLevelMeasurementPublishTopic, 0, sizeof(emergencyLevelMeasurementPublishTopic));
          snprintf(emergencyLevelMeasurementPublishTopic, sizeof(emergencyLevelMeasurementPublishTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_EMERGENCY_LEVEL_MEASUREMENT, hospitalId.c_str(), roomId.c_str());

          // indicate standby
          emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_STANDBY;
          xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

          // mark as no emergency
          mqttClient.publish(emergencyLevelMeasurementPublishTopic, "1");

          standbyStartedAt = millis();
          state = EMERGENCY_RESPONDER_STATE_STANDBY;
        }
        break;

      case EMERGENCY_RESPONDER_STATE_STANDBY:
        {
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(emergencyResponderEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore == emergencyResponderPanicButtonSemaphore) {
            if (xSemaphoreTake(emergencyResponderPanicButtonSemaphore, 0) == pdTRUE) {
              // panic button pressed
              urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_CRITICAL;

              // indicate critical (assistance required)
              emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_CRITICAL_RAISED;
              xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

              // mark as critical
              mqttClient.publish(emergencyLevelMeasurementPublishTopic, "3");

              urgencyStartedAt = millis();
              state = EMERGENCY_RESPONDER_STATE_URGENCY_RAISED;
            }
          } else if (activatedSemaphore == emergencyResponderAssistanceButtonSemaphore) {
            if (xSemaphoreTake(emergencyResponderAssistanceButtonSemaphore, 0) == pdTRUE) {
              // assistance button pressed
              urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_WARNING;

              // indicate warning (assistance needed)
              emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_WARNING_RAISED;
              xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

              // mark as warning
              mqttClient.publish(emergencyLevelMeasurementPublishTopic, "2");

              urgencyStartedAt = millis();
              state = EMERGENCY_RESPONDER_STATE_URGENCY_RAISED;
            }
          } else if (activatedSemaphore != NULL) {
            if (xSemaphoreTake(activatedSemaphore, 0) == pdTRUE) {
              // accept and dispose other events
            }
          } else {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        break;

      case EMERGENCY_RESPONDER_STATE_URGENCY_RAISED:
        {
          unsigned long elapsed = millis() - urgencyStartedAt;
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(emergencyResponderEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore == emergencyResponderPanicButtonSemaphore) {
            if (xSemaphoreTake(emergencyResponderPanicButtonSemaphore, 0) == pdTRUE) {
              if ((urgencyLevel < EMERGENCY_RESPONDER_URGENCY_LEVEL_CRITICAL) && (elapsed > 1000)) {
                // escalate urgency to critical
                urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_CRITICAL;

                // indicate critical, assistance required
                emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_CRITICAL_RAISED;
                xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                // escalated from warning to critical
                mqttClient.publish(emergencyLevelMeasurementPublishTopic, "3");
              }
            }
          } else if (activatedSemaphore == emergencyResponderAssistanceButtonSemaphore) {
            if (xSemaphoreTake(emergencyResponderAssistanceButtonSemaphore, 0) == pdTRUE) {
              if ((urgencyLevel < EMERGENCY_RESPONDER_URGENCY_LEVEL_WARNING) && (elapsed > 1000)) {
                // escalate urgency to warning
                urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_WARNING;

                // indicate warning, assistance needed
                emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_WARNING_RAISED;
                xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                // escalated to warning
                mqttClient.publish(emergencyLevelMeasurementPublishTopic, "2");
              }
            }
          } else if (activatedSemaphore == emergencyResponderResolveButtonSemaphore) {
            if (xSemaphoreTake(emergencyResponderResolveButtonSemaphore, 0) == pdTRUE) {
              if (elapsed > 1000) {
                // resolve button pressed, cancel urgency request
                urgencyLevel = EMERGENCY_RESPONDER_URGENCY_LEVEL_NONE;

                // indicate urgency cancellation
                emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_CANCELLATION;
                xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                cancellationStartedAt = millis();
                state = EMERGENCY_RESPONDER_STATE_URGENCY_CANCELED;
              }
            }
          } else if (activatedSemaphore == emergencyResponderAcknowledgeSignalSemaphore) {
            if (xSemaphoreTake(emergencyResponderAcknowledgeSignalSemaphore, 0) == pdTRUE) {
              if (elapsed > 1000) {
                switch (urgencyLevel) {
                  case EMERGENCY_RESPONDER_URGENCY_LEVEL_CRITICAL:
                    {
                      // indicate critical urgency acknowledged
                      emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_CRITICAL_ACKNOWLEDGED;
                      xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                      urgencyAcknowledgedAt = millis();
                      state = EMERGENCY_RESPONDER_STATE_URGENCY_ACKNOWLEDGED;
                    }
                    break;

                  case EMERGENCY_RESPONDER_URGENCY_LEVEL_WARNING:
                    {
                      // indicate warning urgency acknowledged
                      emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_WARNING_ACKNOWLEDGED;
                      xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                      urgencyAcknowledgedAt = millis();
                      state = EMERGENCY_RESPONDER_STATE_URGENCY_ACKNOWLEDGED;
                    }
                    break;
                }
              }
            }
          } else {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        break;

      case EMERGENCY_RESPONDER_STATE_URGENCY_ACKNOWLEDGED:
        {
          unsigned long elapsed = millis() - urgencyAcknowledgedAt;
          QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(emergencyResponderEventSetQueue, pdMS_TO_TICKS(100));
          if (activatedSemaphore == emergencyResponderResolveButtonSemaphore) {
            if (xSemaphoreTake(emergencyResponderResolveButtonSemaphore, 0) == pdTRUE) {
              if (elapsed > 1000) {
                // resolve button pressed while cancelling, force back to standby
                emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_STANDBY;
                xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                // de-escalate to stable
                mqttClient.publish(emergencyLevelMeasurementPublishTopic, "1");

                standbyStartedAt = millis();
                state = EMERGENCY_RESPONDER_STATE_STANDBY;
              }
            }
          } else if (activatedSemaphore != NULL) {
            // accept and dispose other events
            xSemaphoreTake(activatedSemaphore, 0);
          } else {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
        break;

      case EMERGENCY_RESPONDER_STATE_URGENCY_CANCELED:
        {
          // wait 1 minute before going back to standby
          unsigned long elapsed = millis() - cancellationStartedAt;
          if (elapsed < 60000) {
            QueueSetMemberHandle_t activatedSemaphore = xQueueSelectFromSet(emergencyResponderEventSetQueue, pdMS_TO_TICKS(100));
            if (activatedSemaphore == emergencyResponderResolveButtonSemaphore) {
              if (xSemaphoreTake(emergencyResponderResolveButtonSemaphore, 0) == pdTRUE) {
                if (elapsed > 1000) {
                  // resolve button pressed while cancelling, force back to standby
                  emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_STANDBY;
                  xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));

                  // de-escalate to stable
                  mqttClient.publish(emergencyLevelMeasurementPublishTopic, "1");

                  standbyStartedAt = millis();
                  state = EMERGENCY_RESPONDER_STATE_STANDBY;
                }
              }
            } else if (activatedSemaphore != NULL) {
              // accept and dispose other events
              xSemaphoreTake(activatedSemaphore, 0);
            } else {
              vTaskDelay(pdMS_TO_TICKS(100));
            }
          } else {
            // cancellation cooldown period expired, back top standby
            emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_STANDBY;
            xQueueSend(emergencyResponderIndicationQueue, &indication, pdMS_TO_TICKS(100));


            // de-escalate to stable
            mqttClient.publish(emergencyLevelMeasurementPublishTopic, "1");

            standbyStartedAt = millis();
            state = EMERGENCY_RESPONDER_STATE_STANDBY;
          }
        }
        break;

      default:
        {
          // do nothing
        }
        break;
    }
  }
}

static void emergencyResponderIndicatorTask(void *arg) {
  emergencyResponderIndication_t indication = EMERGENCY_RESPONDER_INDICATE_NONE;
  triColorLedCombination_t ledColor = TRICOLOR_LED_COLOR_NONE;
  ledDutyCycle_t dutyCycle = { .timeOnCount = 1, .timeOffCount = 19, .timeOnCountInitValue = 1, .timeOffCountInitValue = 19 };

  for (;;) {
    unsigned long startTime = millis();
    emergencyResponderIndication_t newIndication = EMERGENCY_RESPONDER_INDICATE_NONE;
    if (xQueueReceive(emergencyResponderIndicationQueue, &newIndication, pdMS_TO_TICKS(100)) == pdTRUE) {
      switch (newIndication) {
        case EMERGENCY_RESPONDER_INDICATE_STANDBY:
          {
            // switch on green led
            ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_GREEN);
            dutyCycle = { .timeOnCount = 1, .timeOffCount = 19, .timeOnCountInitValue = 1, .timeOffCountInitValue = 19 };
          }
          break;

        case EMERGENCY_RESPONDER_INDICATE_CRITICAL_RAISED:
          {
            // switch on red led
            ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_RED);
            dutyCycle = { .timeOnCount = 1, .timeOffCount = 9, .timeOnCountInitValue = 1, .timeOffCountInitValue = 9 };
          }
          break;

        case EMERGENCY_RESPONDER_INDICATE_CRITICAL_ACKNOWLEDGED:
          {
            // switch on red led
            ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_RED);
            dutyCycle = { .timeOnCount = 19, .timeOffCount = 1, .timeOnCountInitValue = 19, .timeOffCountInitValue = 1 };
          }
          break;

        case EMERGENCY_RESPONDER_INDICATE_WARNING_RAISED:
          {
            // switch on blue led
            ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_BLUE);
            dutyCycle = { .timeOnCount = 1, .timeOffCount = 9, .timeOnCountInitValue = 1, .timeOffCountInitValue = 9 };
          }
          break;

        case EMERGENCY_RESPONDER_INDICATE_WARNING_ACKNOWLEDGED:
          {
            // switch on blue led
            ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_BLUE);
            dutyCycle = { .timeOnCount = 19, .timeOffCount = 1, .timeOnCountInitValue = 19, .timeOffCountInitValue = 1 };
          }
          break;

        case EMERGENCY_RESPONDER_INDICATE_CANCELLATION:
          {
            // switch on red and green led
            ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_GREEN);
            dutyCycle = { .timeOnCount = 1, .timeOffCount = 9, .timeOnCountInitValue = 1, .timeOffCountInitValue = 9 };
          }
          break;
      }
      indication = newIndication;
    }

    // keep it at 100 ms for blink simulation
    unsigned long elapsed = millis() - startTime;
    if (elapsed < 100) {
      vTaskDelay(pdMS_TO_TICKS(100 - elapsed));
    }

    if (indication == EMERGENCY_RESPONDER_INDICATE_NONE) {
      ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_NONE);
    } else if (ledColor != EMERGENCY_RESPONDER_INDICATE_NONE) {
      if (dutyCycle.timeOnCount > 0) {
        dutyCycle.timeOnCount--;
        if (dutyCycle.timeOnCount == 0) {
          dutyCycle.timeOnCount = dutyCycle.timeOnCountInitValue;
          ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_NONE);
        }
      }
    } else if (dutyCycle.timeOffCount > 0) {
      dutyCycle.timeOffCount--;
      if (dutyCycle.timeOffCount == 0) {
        dutyCycle.timeOffCount = dutyCycle.timeOffCountInitValue;
        switch (indication) {
          case EMERGENCY_RESPONDER_INDICATE_STANDBY:
            {
              // switch on green led for standby
              ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_GREEN);
            }
            break;

          case EMERGENCY_RESPONDER_INDICATE_CRITICAL_RAISED:
          case EMERGENCY_RESPONDER_INDICATE_CRITICAL_ACKNOWLEDGED:
            {
              // switch on red led for warning
              ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_RED);
            }
            break;

          case EMERGENCY_RESPONDER_INDICATE_WARNING_RAISED:
          case EMERGENCY_RESPONDER_INDICATE_WARNING_ACKNOWLEDGED:
            {
              // switch on blue led for warning
              ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_BLUE);
            }
            break;

          case EMERGENCY_RESPONDER_INDICATE_CANCELLATION:
            {
              // switch on red and green led for cancellation
              ledColor = setEmergencyResponderLedIndicator(TRICOLOR_LED_COLOR_YELLOW);
            }
            break;
        }
      }
    }
  }
}

static void emergencyResponderPanicButtonISR(void) {
  if (emergencyResponderPanicButtonSemaphore != NULL) {
    xSemaphoreGiveFromISR(emergencyResponderPanicButtonSemaphore, NULL);
  }
}

static void emergencyResponderAssistanceButtonISR(void) {
  if (emergencyResponderAssistanceButtonSemaphore != NULL) {
    xSemaphoreGiveFromISR(emergencyResponderAssistanceButtonSemaphore, NULL);
  }
}

static void emergencyResponderResolveButtonISR(void) {
  if (emergencyResponderResolveButtonSemaphore != NULL) {
    xSemaphoreGiveFromISR(emergencyResponderResolveButtonSemaphore, NULL);
  }
}

static triColorLedCombination_t setEmergencyResponderLedIndicator(triColorLedCombination_t color) {
  bool red, green, blue;
  switch (color) {
    case TRICOLOR_LED_COLOR_NONE:
      {
        red = false;
        green = false;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_BLUE:
      {
        red = false;
        green = false;
        blue = true;
      }
      break;

    case TRICOLOR_LED_COLOR_GREEN:
      {
        red = false;
        green = true;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_CYAN:
      {
        red = false;
        green = true;
        blue = true;
      }
      break;

    case TRICOLOR_LED_COLOR_RED:
      {
        red = true;
        green = false;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_MAGENTA:
      {
        red = true;
        green = false;
        blue = true;
      }
      break;

    case TRICOLOR_LED_COLOR_YELLOW:
      {
        red = true;
        green = true;
        blue = false;
      }
      break;

    case TRICOLOR_LED_COLOR_WHITE:
      {
        red = true;
        green = true;
        blue = true;
      }
      break;
  }

  digitalWrite(EMERGENCY_RESPONDER_LED_RED_INDICATOR_PIN, red);
  digitalWrite(EMERGENCY_RESPONDER_LED_GREEN_INDICATOR_PIN, green);
  digitalWrite(EMERGENCY_RESPONDER_LED_BLUE_INDICATOR_PIN, blue);
  return color;
}

static void environmentMonitoringTask(void *arg) {
  typedef enum environmentMonitoringState {
    ENVIRONMENT_MONITORING_STATE_INIT,
    ENVIRONMENT_MONITORING_STATE_WARMING_UP,
    ENVIRONMENT_MONITORING_STATE_READY,
  } environmentMonitoringState_t;

  Hysteresis pollutionHysteresis(
    preferences.getUInt("pollution_low_threshold_count_limit", DEFAULT_POLLUTION_LOW_THRESHOLD_COUNT_LIMIT),
    preferences.getUInt("pollution_high_threshold_count_limit", DEFAULT_POLLUTION_HIGH_THRESHOLD_COUNT_LIMIT));
  Hysteresis temperatureHysteresis(
    preferences.getUInt("temperature_comfort_count_limit", DEFAULT_TEMPERATURE_COMFORT_THRESHOLD_COUNT_LIMIT),
    preferences.getUInt("temperature_discomfort_count_limit", DEFAULT_TEMPERATURE_DISCOMFORT_THRESHOLD_COUNT_LIMIT));
  Hysteresis humidityHysteresis(
    preferences.getUInt("humidity_low_trigger_count", DEFAULT_HUMIDITY_COMFORT_THRESHOLD_COUNT_LIMIT),
    preferences.getUInt("humidity_discomfort_count_limit", DEFAULT_HUMIDITY_DISCOMFORT_THRESHOLD_COUNT_LIMIT));

  char exhaustFanStateTopic[64];
  char pollutionMeasurementTopic[64];
  char temperatureMeasurementTopic[64];
  char humidityMeasurementTopic[64];

  bool exhaustFanState = false;
  environmentMonitoringState_t state = ENVIRONMENT_MONITORING_STATE_INIT;
  for (;;) {
    switch (state) {
      case ENVIRONMENT_MONITORING_STATE_INIT:
        {
          // configure direction
          pinMode(MQ2_SENSOR_ANALOG_OUT_PIN, ANALOG);
          pinMode(EXHAUST_FAN_PIN, OUTPUT);

          // reset output
          digitalWrite(EXHAUST_FAN_PIN, LOW);
          exhaustFanState = false;
          mqttClient.publish(exhaustFanStateTopic, "off");

          // clear override flag
          overrideExhaustFanStateToOn = false;
          overrideExhaustFanStateToOff = false;

          // clear topic buffer
          memset(exhaustFanStateTopic, 0, sizeof(exhaustFanStateTopic));
          memset(pollutionMeasurementTopic, 0, sizeof(pollutionMeasurementTopic));
          memset(temperatureMeasurementTopic, 0, sizeof(temperatureMeasurementTopic));
          memset(humidityMeasurementTopic, 0, sizeof(humidityMeasurementTopic));

          // derive topics
          String hospitalId = preferences.getString("mqtt_hospital_id", DEFAULT_MQTT_HOSPITAL_ID);
          String roomId = preferences.getString("mqtt_room_id", DEFAULT_MQTT_ROOM_ID);
          snprintf(exhaustFanStateTopic, sizeof(exhaustFanStateTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_EXHAUST_FAN_STATE, hospitalId.c_str(), roomId.c_str());
          snprintf(pollutionMeasurementTopic, sizeof(pollutionMeasurementTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_POLLUTION_LEVEL_MEASUREMENT, hospitalId.c_str(), roomId.c_str());
          snprintf(temperatureMeasurementTopic, sizeof(temperatureMeasurementTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_TEMPERATURE_MEASUREMENT, hospitalId.c_str(), roomId.c_str());
          snprintf(humidityMeasurementTopic, sizeof(humidityMeasurementTopic) - 1, MQTT_PUBLISH_TOPIC_TEMPLATE_HUMIDITY_MEASUREMENT, hospitalId.c_str(), roomId.c_str());

          // initialized, wait until DHT data is available
          state = ENVIRONMENT_MONITORING_STATE_WARMING_UP;
        }
        break;

      case ENVIRONMENT_MONITORING_STATE_WARMING_UP:
        {
          // poll DHT sensor until it's ready
          TempAndHumidity data = dhtSensor.getTempAndHumidity();
          if (!isnan(data.temperature) && !isnan(data.humidity)) {
            state = ENVIRONMENT_MONITORING_STATE_READY;
          } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
          }
        }
        break;

      case ENVIRONMENT_MONITORING_STATE_READY:
        {
          float pollutionLowThresholdValue = preferences.getFloat("pollution_low_threshold_value", DEFAULT_POLLUTION_LOW_THRESHOLD_VALUE);     // 20ppm as low limit
          float pollutionHighThresholdValue = preferences.getFloat("pollution_high_threshold_value", DEFAULT_POLLUTION_HIGH_THRESHOLD_VALUE);  // 120ppm as high limit

          // read air quality
          uint16_t pollutionRaw = analogRead(MQ2_SENSOR_ANALOG_OUT_PIN);
          float pollutionPPM = convertPollutionToPPM(pollutionRaw);
          if (pollutionPPM < pollutionLowThresholdValue) {
            pollutionHysteresis.update(false);
          } else if (pollutionPPM > pollutionHighThresholdValue) {
            pollutionHysteresis.update(true);
          }

          // send pollution level measurements
          mqttClient.publish(pollutionMeasurementTopic, String(pollutionPPM, 1).c_str());

          TempAndHumidity data = dhtSensor.getTempAndHumidity();
          if (!isnan(data.temperature) && !isnan(data.humidity)) {
            // get temperature preference
            float temperatureLowThresholdValue = preferences.getFloat("temperature_low_threshold_value", DEFAULT_TEMPERATURE_LOW_THRESHOLD_VALUE);     // 22 C as low limit
            float temperatureHighThresholdValue = preferences.getFloat("temperature_high_threshold_value", DEFAULT_TEMPERATURE_HIGH_THRESHOLD_VALUE);  // 28 C as high limit
            float humidityLowThresholdValue = preferences.getFloat("humidity_low_threshold_value", DEFAULT_HUMIDITY_LOW_THRESHOLD_VALUE);              // 40% humidity as low limit
            float humidityHighThresholdValue = preferences.getFloat("humidity_high_threshold_value", DEFAULT_HUMIDITY_HIGH_THRESHOLD_VALUE);           // 60% humidity as high limit

            // make sure temperature is in range
            if (data.temperature > temperatureHighThresholdValue || data.temperature < temperatureLowThresholdValue) {
              temperatureHysteresis.update(true);
            } else {
              temperatureHysteresis.update(false);
            }

            // make sure humidity is in range
            if (data.humidity > humidityHighThresholdValue || data.humidity < humidityLowThresholdValue) {
              humidityHysteresis.update(true);
            } else {
              humidityHysteresis.update(false);
            }

            // send temperature and humidity measure
            mqttClient.publish(temperatureMeasurementTopic, String(data.temperature, 2).c_str());
            mqttClient.publish(humidityMeasurementTopic, String(data.humidity, 1).c_str());
          }

          // activate exhaust fan when pollution is too high or temperature is out of range or humidity is out of range
          bool newExhaustFanState = pollutionHysteresis.state() || temperatureHysteresis.state() || humidityHysteresis.state();

          // consider overriding value
          if (overrideExhaustFanStateToOn) {
            newExhaustFanState = true;
          } else if (overrideExhaustFanStateToOff) {
            newExhaustFanState = false;
          }

          // apply exhaust fan state
          if (exhaustFanState != newExhaustFanState) {
            exhaustFanState = newExhaustFanState;
            digitalWrite(EXHAUST_FAN_PIN, newExhaustFanState ? HIGH : LOW);
            mqttClient.publish(exhaustFanStateTopic, newExhaustFanState ? "on" : "off");
          }

          vTaskDelay(pdMS_TO_TICKS(1000));
        }
        break;
    }
  }
}

static void mqttKeepAliveTask(void *arg) {
  typedef enum mqttKeepAliveTaskState {
    MQTT_KEEP_ALIVE_STATE_INIT,
    MQTT_KEEP_ALIVE_STATE_CONNECTING,
    MQTT_KEEP_ALIVE_STATE_SUBSCRIBING,
    MQTT_KEEP_ALIVE_STATE_READY,
  } mqttKeepAliveTaskState_t;
  mqttKeepAliveTaskState_t state = MQTT_KEEP_ALIVE_STATE_INIT;

  // get MQTT preferences
  String mqttServerHost = preferences.getString("mqtt_server_host", DEFAULT_MQTT_SERVER_HOST);
  uint16_t mqttServerPort = preferences.getUShort("mqtt_server_port", DEFAULT_MQTT_SERVER_PORT);
  String mqttClientId = preferences.getString("mqtt_client_id", DEFAULT_MQTT_CLIENT_ID);

  for (;;) {
    switch (state) {
      case MQTT_KEEP_ALIVE_STATE_INIT:
        {
          mqttClient.setCallback(mqttCallback);
          state = MQTT_KEEP_ALIVE_STATE_CONNECTING;
        }
        break;

      case MQTT_KEEP_ALIVE_STATE_CONNECTING:
        {
          Serial.printf("connecting to MQTT server %s:%d as %s\r\n", mqttServerHost.c_str(), mqttServerPort, mqttClientId.c_str());
          wifiClient.connect(mqttServerHost.c_str(), mqttServerPort);
          if (mqttClient.connect(mqttClientId.c_str())) {
            Serial.println("connected to MQTT server");
            state = MQTT_KEEP_ALIVE_STATE_SUBSCRIBING;
          } else {
            // can't connect, retry again after 5 seconds
            vTaskDelay(pdMS_TO_TICKS(5000));
          }
        }
        break;

      case MQTT_KEEP_ALIVE_STATE_SUBSCRIBING:
        {
          int mqttSubscriptionFailureCount = 0;
          for (int i = 0; i < MQTT_SUBSCRIPTION_TOPIC_TEMPLATES_COUNT && mqttSubscriptionFailureCount == 0; i++) {
            // get hospital and room identifier
            String hospitalId = preferences.getString("mqtt_hospital_id", DEFAULT_MQTT_HOSPITAL_ID);
            String roomId = preferences.getString("mqtt_room_id", DEFAULT_MQTT_ROOM_ID);

            const char *subscriptionTopicTemplate = mqttSubscriptionTopicTemplates[i];
            char subscriptionTopic[64];
            memset(subscriptionTopic, 0, sizeof(subscriptionTopic));
            snprintf(subscriptionTopic, sizeof(subscriptionTopic) - 1, subscriptionTopicTemplate, hospitalId.c_str(), roomId.c_str());

            if (mqttClient.subscribe(subscriptionTopic)) {
              Serial.printf("successfully subscribed to MQTT topic %s\r\n", subscriptionTopic);
            } else {
              Serial.printf("failed to subscribe to MQTT topic %s\r\n", subscriptionTopic);
              mqttSubscriptionFailureCount++;
            }
          }

          if (mqttSubscriptionFailureCount > 0) {
            Serial.println("failed to subscribe to several MQTT topics, reconnecting...");
            state = MQTT_KEEP_ALIVE_STATE_CONNECTING;
          } else {
            Serial.println("successfully subscribed to all MQTT topics");
            state = MQTT_KEEP_ALIVE_STATE_READY;
          }
        }
        break;

      case MQTT_KEEP_ALIVE_STATE_READY:
        {
          switch (mqttClient.state()) {
            case MQTT_CONNECTED:
              {
                if (mqttClient.loop()) {
                  vTaskDelay(pdMS_TO_TICKS(2000));
                } else {
                  Serial.println("MQTT loop failed");
                  state = MQTT_KEEP_ALIVE_STATE_CONNECTING;
                }
              }
              break;

            case MQTT_DISCONNECTED:
              {
                Serial.println("MQTT disconnected");
                state = MQTT_KEEP_ALIVE_STATE_CONNECTING;
              }
              break;

            case MQTT_CONNECTION_TIMEOUT:
              {
                Serial.println("MQTT timed out");
                state = MQTT_KEEP_ALIVE_STATE_CONNECTING;
              }
              break;
          }
        }
        break;
    }
  }
}

static void mqttCallback(char *topic, byte *message, unsigned int length) {
  // get hospital and room identifier
  String hospitalId = preferences.getString("mqtt_hospital_id", DEFAULT_MQTT_HOSPITAL_ID);
  String roomId = preferences.getString("mqtt_room_id", DEFAULT_MQTT_ROOM_ID);
  char subscriptionTopic[64];
  char bufferedMessage[64];

  bool matchingTopicFound = false;
  for (int i = 0; i < MQTT_SUBSCRIPTION_TOPIC_TEMPLATES_COUNT && !matchingTopicFound; i++) {
    const char *subscriptionTopicTemplate = mqttSubscriptionTopicTemplates[i];
    memset(subscriptionTopic, 0, sizeof(subscriptionTopic));
    snprintf(subscriptionTopic, sizeof(subscriptionTopic) - 1, subscriptionTopicTemplate, hospitalId.c_str(), roomId.c_str());

    // check for known topics
    if (strncmp(subscriptionTopic, topic, sizeof(subscriptionTopic) - 1) == 0) {
      // copy message
      memset(bufferedMessage, 0, sizeof(bufferedMessage));
      size_t limit = min(sizeof(bufferedMessage) - 1, length);
      memcpy(bufferedMessage, message, limit);
      Serial.printf("Message arrived on known topic: %s: %s\r\n", topic, bufferedMessage);

      matchingTopicFound = true;
      if (strcmp(subscriptionTopicTemplate, MQTT_SUBSCRIPTION_TOPIC_TEMPLATE_EMERGENCY_ACKNOWNEDGE) == 0) {
        // it's emeregency acknowledge event
        if (emergencyResponderAcknowledgeSignalSemaphore != NULL) {
          xSemaphoreGive(emergencyResponderAcknowledgeSignalSemaphore);
        }
      } else if (strcmp(subscriptionTopicTemplate, MQTT_SUBSCRIPTION_TOPIC_TEMPLATE_EXHAUST_FAN_OVERIDE) == 0) {
        if (strncmp((char *)bufferedMessage, "1", 1) == 0) {
          // override on
          overrideExhaustFanStateToOn = true;
          overrideExhaustFanStateToOff = false;
        } else if (strncmp((char *)bufferedMessage, "0", 1) == 0) {
          // override off
          overrideExhaustFanStateToOn = false;
          overrideExhaustFanStateToOff = true;
        } else {
          // clear override
          overrideExhaustFanStateToOn = false;
          overrideExhaustFanStateToOff = false;
        }
      }
    }
  }
}

static uint16_t rawValues[165] = {
  843, 962, 1124, 1251, 1334, 1421, 1474, 1527, 1582, 1656, 1807, 2018, 2173, 2269, 2364, 2420, 2476, 2531, 2567, 2621,
  2657, 2674, 2709, 2743, 2760, 2777, 2811, 2827, 2844, 2860, 2876, 2893, 2909, 2924, 2940, 2956, 2971, 2986, 3002, 3017,
  3032, 3046, 3061, 3076, 3090, 3104, 3118, 3132, 3146, 3160, 3173, 3187, 3200, 3213, 3226, 3239, 3252, 3264, 3277, 3289,
  3301, 3313, 3325, 3337, 3348, 3360, 3371, 3382, 3393, 3404, 3415, 3426, 3436, 3447, 3457, 3467, 3477, 3487, 3497, 3506,
  3516, 3525, 3534, 3543, 3552, 3561, 3570, 3578, 3587, 3595, 3603, 3611, 3619, 3627, 3635, 3643, 3650, 3658, 3665, 3672,
  3679, 3686, 3693, 3700, 3707, 3713, 3720, 3726, 3732, 3739, 3745, 3751, 3757, 3762, 3768, 3774, 3779, 3785, 3790, 3795,
  3801, 3806, 3811, 3816, 3821, 3825, 3830, 3835, 3839, 3844, 3848, 3853, 3859, 3861, 3865, 3869, 3873, 3877, 3881, 3885,
  3889, 3892, 3896, 3899, 3903, 3906, 3910, 3913, 3916, 3919, 3923, 3926, 3929, 3932, 3935, 3938, 3940, 3943, 3946, 3949,
  3951, 3954, 3956, 3959
};

static float ppmValues[165] = {
  0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
  22, 23, 24, 25, 26, 28, 29, 30, 32, 33, 35, 36, 38, 40, 42, 44, 46, 48, 50, 52, 55, 58, 60, 63, 66, 69, 72, 74, 79, 83,
  87, 91, 95, 100, 105, 110, 115, 120, 126, 132, 138, 145, 151, 158, 166, 174, 182, 191, 200, 209, 219, 229, 240, 251, 263,
  275, 288, 302, 316, 331, 347, 363, 380, 398, 417, 437, 457, 479, 501, 525, 550, 575, 603, 631, 661, 692, 724, 759, 794,
  832, 871, 912, 955, 1000, 1047, 1096, 1148, 1202, 1259, 1318, 1380, 1445, 1514, 1585, 1660, 1738, 1820, 1905, 1995, 2089,
  2188, 2291, 2399, 2512, 2630, 2754, 2884, 3020, 3162, 3311, 3467, 3631, 3802, 3981, 4169, 4365, 4571, 4786, 5012, 5248,
  5495, 5754, 6026, 6310, 6607, 6918, 7244, 7586, 7943, 8318, 8710, 9120, 9550, 10000
};

float convertPollutionToPPM(uint16_t raw) {
  if (raw < 843) {
    return 0.1;
  }

  if (raw > 3959) {
    return 10000;
  }

  uint16_t low = 0;
  uint16_t high = 164;
  uint16_t mid = 0;

  while (low <= high) {
    mid = (low + high) / 2;
    if (rawValues[mid] == raw) {
      return ppmValues[mid];
    } else if (rawValues[mid] < raw) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }
  if (high < 0) {
    return ppmValues[0];
  }

  if (low >= 165) {
    return ppmValues[164];
  }

  if (abs(rawValues[high] - raw) < abs(rawValues[low] - raw)) {
    return ppmValues[high];
  }

  return ppmValues[low];
}
