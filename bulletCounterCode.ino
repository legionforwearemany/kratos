#include <ArduinoBLE.h>
#include <VL53L0X.h>
#include <Wire.h>

BLEService bulletCounterService("1101");
BLEUnsignedCharCharacteristic bulletCounterChar("2101", BLERead | BLENotify);

VL53L0X sensor;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) 
  {
    Serial.println("starting BLE failed!");
    while (1);  
  }
  
  BLE.setLocalName("BulletCounter");
  BLE.setAdvertisedService(bulletCounterService);
  bulletCounterService.addCharacteristic(bulletCounterChar);
  BLE.addService(bulletCounterService);

  sensor.setTimeout(500);
    if (!sensor.init())
    {
      Serial.println("Failed to detect and initialize sensor!");
      while (1) {}
    }
    else{
      Serial.println("Sensor initialized");
    }

  sensor.setMeasurementTimingBudget(200000);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() 
{
  BLEDevice central = BLE.central();
  
  if (central) 
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    
    while (central.connected()) {
        PollSensor();
        delay(3000);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

void PollSensor()
{
  float distance = sensor.readRangeSingleMillimeters();
  
  if(!isnan(distance)){
    bulletCounterChar.setValue(distance);
    Serial.print(F("Distance:"));
    Serial.println(distance);
  }
  
}