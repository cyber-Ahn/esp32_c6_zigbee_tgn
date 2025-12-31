#include "Zigbee.h"
#include <DHT.h>

//-------------------------Zigbee-Endpoints-----------------------
#define ZIGBEE_OUTLET_ENDPOINT 1
#define ZIGBEE_ILLUMINANCE_SENSOR_ENDPOINT 9
#define TEMP_SENSOR_ENDPOINT_NUMBER 10
#define FLOW_SENSOR_ENDPOINT_NUMBER 11
#define PRESSURE_SENSOR_ENDPOINT_NUMBER 12
#define OCCUPANCY_SENSOR_ENDPOINT_NUMBER 13
#define ZIGBEE_LIGHT_ENDPOINT 14

//-------------------------Variables..---------------------------
#define DHTPIN 4
#define DHTTYPE DHT22
uint8_t rl_pin = 2;
uint8_t illuminance_sensor_pin = 6;
uint8_t sensor_pin = 11;
uint8_t led = RGB_BUILTIN;
uint8_t button = BOOT_PIN;
struct tm timeinfo;
struct tm *localTime;
int32_t timezone;

//-------------------------Define.Parts---------------------------
DHT dht(DHTPIN, DHTTYPE);
ZigbeePowerOutlet zbOutlet = ZigbeePowerOutlet(ZIGBEE_OUTLET_ENDPOINT);
ZigbeeIlluminanceSensor zbIlluminanceSensor = ZigbeeIlluminanceSensor(ZIGBEE_ILLUMINANCE_SENSOR_ENDPOINT);
ZigbeeTempSensor zbTempSensor = ZigbeeTempSensor(TEMP_SENSOR_ENDPOINT_NUMBER);
ZigbeeFlowSensor zbFlowSensor = ZigbeeFlowSensor(FLOW_SENSOR_ENDPOINT_NUMBER);
ZigbeePressureSensor zbPressureSensor = ZigbeePressureSensor(PRESSURE_SENSOR_ENDPOINT_NUMBER);
ZigbeeOccupancySensor zbOccupancySensor = ZigbeeOccupancySensor(OCCUPANCY_SENSOR_ENDPOINT_NUMBER);
ZigbeeLight zbLight = ZigbeeLight(ZIGBEE_LIGHT_ENDPOINT);

//-------------------------Function---------------------------------
static void illuminance_sensor_value_update(void *arg) {
  for (;;) {
    int lsens_analog_raw = analogRead(illuminance_sensor_pin);
    int lsens_illuminance_raw = map(lsens_analog_raw, 0, 4095, 0, 50000);
    int lsens_illuminance_lux = round(pow(10, (lsens_illuminance_raw / 10000.0)) - 1);
    Serial.printf("[Illuminance Sensor] lux value: %d lux\r\n", lsens_illuminance_lux);
    zbIlluminanceSensor.setIlluminance(lsens_illuminance_raw);
    delay(25000);
  }
}

static void temp_sensor_value_update(void *arg) {
  for (;;) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\r\n", temperature, humidity);
      zbTempSensor.setTemperature(temperature);
      zbTempSensor.setHumidity(humidity);
    }
    delay(30000);
  }
}

void setLED(bool value) {
  digitalWrite(led, value);
}

void setRL(bool value) {
  digitalWrite(rl_pin, value);
}

//-------------------------Setup--------------------------------
void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(button, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  pinMode(button, INPUT_PULLUP);
  pinMode(sensor_pin, INPUT);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  pinMode(rl_pin, OUTPUT);
  digitalWrite(rl_pin, LOW); 

  zbLight.setManufacturerAndModel("TGN", "Modul1");
  zbOutlet.setManufacturerAndModel("TGN", "Modul1");
  zbIlluminanceSensor.setManufacturerAndModel("TGN", "Modul1");
  zbTempSensor.setManufacturerAndModel("TGN", "Modul1");
  zbFlowSensor.setManufacturerAndModel("TGN", "Modul1");
  zbPressureSensor.setManufacturerAndModel("TGN", "Modul1");
  zbOccupancySensor.setManufacturerAndModel("TGN", "Modul1");

  zbTempSensor.setMinMaxValue(10, 50);
  zbTempSensor.setTolerance(1);
  zbTempSensor.addTimeCluster();
  zbTempSensor.addHumiditySensor(20.0, 90.0, 5.0);

  zbFlowSensor.setMinMaxValue(0.0, 100.0);
  zbFlowSensor.setTolerance(1.0);
  zbPressureSensor.setMinMaxValue(0, 10000);
  zbPressureSensor.setTolerance(1);

  zbIlluminanceSensor.setPowerSource(ZB_POWER_SOURCE_MAINS);
  zbIlluminanceSensor.setMinMaxValue(0, 50000);
  zbIlluminanceSensor.setTolerance(1);

  zbLight.onLightChange(setLED);
  zbOutlet.onPowerOutletChange(setRL);

  Zigbee.addEndpoint(&zbIlluminanceSensor);
  Zigbee.addEndpoint(&zbTempSensor);
  Zigbee.addEndpoint(&zbFlowSensor);
  Zigbee.addEndpoint(&zbPressureSensor);
  Zigbee.addEndpoint(&zbOccupancySensor);
  Zigbee.addEndpoint(&zbOutlet);
  Zigbee.addEndpoint(&zbLight);

  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  } else {
    Serial.println("Zigbee started successfully!");
  }

  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  timeinfo = zbTempSensor.getTime();
  timezone = zbTempSensor.getTimezone();
  Serial.println("UTC time:");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  time_t local = mktime(&timeinfo) + timezone;
  localTime = localtime(&local);
  Serial.println("Local time with timezone:");
  Serial.println(localTime, "%A, %B %d %Y %H:%M:%S");

  xTaskCreate(temp_sensor_value_update, "temp_sensor_update", 2048, NULL, 10, NULL);
  xTaskCreate(illuminance_sensor_value_update, "illuminance_sensor_update", 2048, NULL, 10, NULL);

  zbIlluminanceSensor.setReporting(1, 0, 1000);
  zbTempSensor.setReporting(1, 60, 0.1);
  zbTempSensor.setHumidityReporting(1, 60, 1);
  zbFlowSensor.setReporting(0, 30, 1.0);
  zbPressureSensor.setReporting(0, 30, 1);
}

//-------------------------Main-Loop---------------------------
void loop() {

   static bool occupancy = false;
  if (digitalRead(sensor_pin) == HIGH && !occupancy) {
    Serial.println("Sensore true");
    zbOccupancySensor.setOccupancy(true);
    delay(100);
    zbOccupancySensor.report();
    occupancy = true;
  } else if (digitalRead(sensor_pin) == LOW && occupancy) {
    Serial.println("Sensore false");
    zbOccupancySensor.setOccupancy(false);
    delay(100);
    zbOccupancySensor.report();
    occupancy = false;
  }

  static uint32_t timeCounter = 0;
  if (!(timeCounter++ % 300)) {
    float flow_value = temperatureRead();
    uint16_t pressure_value = (uint16_t)temperatureRead() * 100;  //*100 for demonstration so the value is in 1000-3000hPa
    Serial.printf("Flow: %.2f m3/h\r\n", flow_value);
    zbFlowSensor.setFlow(flow_value);
    Serial.printf("Pressure: %d hPa\r\n", pressure_value);
    zbPressureSensor.setPressure(pressure_value);
  }

  if (digitalRead(button) == LOW) {
    delay(100);
    int startTime = millis();
    while (digitalRead(button) == LOW) {
      delay(50);
      if ((millis() - startTime) > 3000) {
        Serial.println("Resetting Zigbee to factory and rebooting in 1s.");
        delay(1000);
        Zigbee.factoryReset();
      }
    }
    zbIlluminanceSensor.report();
    zbTempSensor.reportTemperature();
    zbTempSensor.reportHumidity();
    zbLight.setLight(!zbLight.getLightState());
    zbOutlet.setState(!zbOutlet.getPowerOutletState());
  }
  delay(100);
}