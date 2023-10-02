#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <WiFi.h>
#include <WiFiClient.h>

// Configuration lcd
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Component pins
const int waterLevelSensorPin = 2;  // default 2 // ESP32
const int flow1SensorPin = 36;      // ESP32
const int flow2SensorPin = 13;      // ESP32
const int relayV1Pin = 25;          // ESP32
const int relayV2Pin = 27;          // ESP32
const int relayPoPin = 16;          // ESP32
const int relayMoPin = 4;           // ESP32


// Confiration DS18B20 Sensor
OneWire ds(15);  // ESP32
float celsius;

// Kebutuhan PPM
int kebutuhanPPM[] = {
  0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200,
  220, 240, 260, 280, 300, 320, 340, 360, 380, 400, 420
};

// Variable flow sensors
volatile int pulseCount1 = 0;
float flowRate1 = 0.0;
unsigned int flowMilliLitres1 = 0;
volatile int pulseCount2 = 0;
float flowRate2 = 0.0;
unsigned int flowMilliLitres2 = 0;



#define TdsSensorPin 33  // ESP32
#define VREF 3.3
#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
int waterLevelSensorValue;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;

// TDS Sensor
// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}



void tdsSensor() {
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      //temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      //convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      Serial.print("TDS Value:");
      Serial.print(tdsValue, 0);
      Serial.println(" ppm");
      lcd.setCursor(0, 1);
      lcd.print("PPM : " + String(tdsValue));
      //Blynk.virtualWrite(V1, tdsValue);
    }
  }
}

// DS (Temperature) Sensor
void dsSensor(void) {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Not Found");
      delay(2000);
      lcd.clear();
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);  // start conversion, with parasite power on at the end
  delay(1000);
  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);  // Read Scratchpad

  for (i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3;  // 9 bit resolution default
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw & ~7;       // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3;  // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1;  // 11 bit res, 375 ms
  }
  celsius = (float)raw / 16.0;
  Serial.print("Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius ");
  lcd.setCursor(0, 0);
  lcd.print("SUHU: " + String(celsius) + " C");


}
void volFlow1() {
  delay(1000);
  flowRate1 = pulseCount1 / 7.5;
  flowMilliLitres1 = (flowRate1 / 60) * 1000;
  Serial.print("Flow 1: ");
  Serial.print(flowMilliLitres1);
  Serial.println(" ml/s");
  lcd.setCursor(0, 2);
  lcd.print("FLOW: " + String(flowMilliLitres1) + " mL/s");
}
void volFlow2() {
  delay(1000);
  flowRate2 = pulseCount2 / 7.5;
  flowMilliLitres2 = (flowRate2 / 60) * 1000;
  Serial.print("Flow 2: ");
  Serial.print(flowMilliLitres2);
  Serial.println(" ml/s");
}

void IRAM_ATTR pulseCounter1() {
  pulseCount1++;
}

void IRAM_ATTR pulseCounter2() {
  pulseCount2++;
}


void setup() {
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  pinMode(waterLevelSensorPin, INPUT_PULLUP);
  pinMode(flow1SensorPin, INPUT);
  pinMode(flow2SensorPin, INPUT);
  pinMode(relayV1Pin, OUTPUT);
  pinMode(relayV2Pin, OUTPUT);
  pinMode(relayPoPin, OUTPUT);
  pinMode(relayMoPin, OUTPUT);


  // Relay Low
  digitalWrite(relayV1Pin, HIGH);
  digitalWrite(relayV2Pin, HIGH);
  digitalWrite(relayPoPin, HIGH);
  digitalWrite(relayMoPin, HIGH);

  attachInterrupt(digitalPinToInterrupt(flow1SensorPin), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(flow2SensorPin), pulseCounter2, FALLING);

  lcd.begin();
  lcd.backlight();
}

void loop() {
  int waterLevelSensorValue = digitalRead(waterLevelSensorPin);

  tdsSensor();
  dsSensor();

  if (waterLevelSensorValue == HIGH) {
    lcd.setCursor(0, 3);
    lcd.print("WL  : Penuh ");
    /*
    digitalWrite(relayV1Pin, LOW);
    digitalWrite(relayV2Pin, LOW);
    */
    digitalWrite(relayMoPin, LOW);
    delay(5000);
    digitalWrite(relayMoPin, HIGH);

  } else {
    lcd.setCursor(0, 3);
    lcd.print("WL  : Kosong");
    digitalWrite(relayPoPin, LOW);
    delay(4000);
    digitalWrite(relayPoPin, HIGH);
  }
  delay(2000);
  lcd.clear();

  if (tdsValue > 500 || waterLevelSensorValue == LOW) {
    volFlow1();
    digitalWrite(relayV1Pin, LOW);
    if (flowMilliLitres1 > 50) {
      digitalWrite(relayV1Pin, HIGH);
      delay(2000);

      flowMilliLitres1 = 0;
      digitalWrite(relayV2Pin, LOW);
      delay(2000);

      if (flowMilliLitres1 > 50) {
        digitalWrite(relayV2Pin, HIGH);
        flowMilliLitres1 = 0;
        delay(2000);
      }
    }
    delay(5000);
    volFlow2();
    digitalWrite(relayPoPin, HIGH);
    if (flowMilliLitres2 >= 5000) {
      digitalWrite(relayPoPin, LOW);
      flowMilliLitres2 = 0;
    }

  } if (tdsValue > 1500) {
      delay(5000);
      volFlow2();
        digitalWrite(relayPoPin, HIGH);
      if (flowMilliLitres2 >= 5000) {
        digitalWrite(relayPoPin, LOW);
        flowMilliLitres2 = 0;
      }
    }
    else {

    }
  
  if (celsius > 29) {
    digitalWrite(relayPoPin, LOW);
    digitalWrite(relayMoPin, LOW);
  } else {
    digitalWrite(relayPoPin, HIGH);
    digitalWrite(relayMoPin, HIGH);
  }
}
