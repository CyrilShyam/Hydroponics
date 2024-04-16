#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <time.h>
#include <FirebaseESP32.h>
#include <WiFi.h>


void sendToFirebase(String sensorID, float value);
#define FIREBASE_HOST "has-vidhai-e8b7a-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "AIzaSyBx2Z-oBJHDpOxgFlROY8kmV4pyXQr6sDE"
#define WIFI_SSID "Pixel"
#define WIFI_PASSWORD "vedhu123"


FirebaseData firebaseData;
FirebaseConfig firebaseConfig;


#define TdsSensorPin 34  // Assuming pin 34 is used for the TDS sensor on ESP32
#define SensorPin 32     // pH meter Analog output to ESP32 Analog Input 32
#define VREF 3.3         // Analog reference voltage(Volt) of the ADC for ESP32
#define Offset -1.5     // Deviation compensate
#define LED 2            // Use the appropriate GPIO pin for the LED
#define SCOUNT 30        // Sum of sample point
#define ArrayLenth  40   // Times of collection
#define samplingInterval 20
#define printInterval 800

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;

const int TEMP_RELAY_PIN = 2; //temp relay pin
const int PH_DOWN_RELAY_PIN = 23; // pH down relay pin
const int PH_UP_RELAY_PIN = 19; // pH up relay pin
const int NUTRIENT_A_RELAY_PIN = 16;
const int NUTRIENT_B_RELAY_PIN = 17;
const int WATER_RELAY_PIN = 5;
//SDA PIN 21
//SDL PIN 22

// LCD configuration
const int LCD_ADDRESS = 0x27;  // I2C address of the LCD
const int LCD_COLS = 16;       // Number of columns in the LCD
const int LCD_ROWS = 2;        // Number of rows in the LCD
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

int getMedianNum(int bArray[], int iFilterLen);  // Function prototype
double avergearray(int* arr, int number); // Function prototype
float getTemperature(); // Function prototype
float getTDSValue(); // Function prototype
float getPHValue(); // Function prototype

int analogBuffer[SCOUNT];    // Store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25, pHValue = 7.0;
int pHArray[ArrayLenth];   // Store the average value of the sensor feedback
int pHArrayIndex = 0;

int lcdInterval = 2000; // Display interval on LCD in milliseconds
unsigned long lcdPreviousMillis = 0;
int lcdState = 0; // 0: Temperature, 1: TDS Value, 2: pH Value

bool pHUpPumpActive = false;
bool pHDownPumpActive = false;
bool nutAPumpActive = false;
bool nutBPumpActive = false;
bool waterpumpActive = false;
bool mixingInProgressPH = false;
bool mixingInProgressTDS = false;
unsigned long pHStartTime = 0;  // Track the start time of pH adjustment
unsigned long TDSStartTime = 0;  // Track the start time of TDS adjustment
unsigned long mixStartTime = 0;
const unsigned long pumpDuration = 2000; // Duration for pH/TDS adjustment pumps (2 seconds)
const unsigned long mixDuration = 30000; // Mixing duration (30 seconds)


void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  firebaseConfig.host = FIREBASE_HOST;
  firebaseConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&firebaseConfig, nullptr);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase initialized");

  configTime(3, 3600, "pool.ntp.org", "time.nist.gov");
  Serial.println("Waiting for time");
  while (!time(nullptr)) {
    Serial.println(".");
    delay(1000);
  }
  Serial.println("Time set");


  pinMode(TdsSensorPin, INPUT);
  pinMode(LED, OUTPUT);
  // Start the DS18B20 sensor
  sensors.begin();
  pinMode(TEMP_RELAY_PIN, OUTPUT); // Set temp relay pin as output
  pinMode(PH_DOWN_RELAY_PIN, OUTPUT);
  pinMode(PH_UP_RELAY_PIN, OUTPUT);
  pinMode(NUTRIENT_A_RELAY_PIN, OUTPUT);
  pinMode(NUTRIENT_B_RELAY_PIN, OUTPUT);
  pinMode(WATER_RELAY_PIN, OUTPUT);

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
}

void loop() {
  temperature = getTemperature();
  tdsValue = getTDSValue();
  pHValue = getPHValue();

  // Print sensor values
  Serial.print("TDS Value: ");
  Serial.print(tdsValue, 0);
  Serial.println(" ppm");
  Serial.print("pH Value: ");
  Serial.println(pHValue, 2);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  sendToFirebase("TDS Sensor", tdsValue);
  sendToFirebase("pH Sensor", pHValue);
  sendToFirebase("Temperature Sensor", temperature);

  delay(1000);

  unsigned long currentMillis = millis();
  if (currentMillis - lcdPreviousMillis >= lcdInterval) {
    lcdPreviousMillis = currentMillis;
    lcd.clear();
    switch (lcdState) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Temp:");
        lcd.setCursor(0, 1);
        lcd.print(temperature);
        lcd.print(" C");
        break;
      case 1:
        lcd.setCursor(0, 0);
        lcd.print("TDS Value:");
        lcd.setCursor(0, 1);
        lcd.print(tdsValue, 0);
        lcd.print(" ppm");
        break;
      case 2:
        lcd.setCursor(0, 0);
        lcd.print("pH Value:");
        lcd.setCursor(0, 1);
        lcd.print(pHValue, 2);
        break;
    }
    lcdState++;
    if (lcdState > 2) {
      lcdState = 0;
    }
  }

  // Control relay based on temperature
  if (temperature > 28) {
    digitalWrite(TEMP_RELAY_PIN, LOW); // Turn on relay
  } else {
    digitalWrite(TEMP_RELAY_PIN, HIGH); // Turn off relay
  }

  // Control pH down and pH up relays based on pH value
  // Check if pH is within the ideal range (5.5 - 6.5)
  if (pHValue >= 5.5 && pHValue <= 6.5) {
    digitalWrite(PH_DOWN_RELAY_PIN, HIGH); // Turn off pH down relay
    digitalWrite(PH_UP_RELAY_PIN, HIGH);   // Turn off pH up relay
  } else {
    // pH is outside the ideal range, proceed with adjustments
    if (pHValue < 5.5 && !pHUpPumpActive && !mixingInProgressPH) {
      // pH is below 5.5, trigger pH up adjustment
      pHUpPumpActive = true;
      pHStartTime = currentMillis;  // Update pH adjustment start time
      digitalWrite(PH_DOWN_RELAY_PIN, HIGH);  // Turn off pH down relay
      digitalWrite(PH_UP_RELAY_PIN, LOW);  // Turn on pH up relay
    } else if (pHValue > 6.5 && !pHDownPumpActive && !mixingInProgressPH) {
      // pH is above 6.5, trigger pH down adjustment
      pHDownPumpActive = true;
      pHStartTime = currentMillis;  // Update pH adjustment start time
      digitalWrite(PH_UP_RELAY_PIN, HIGH);  // Turn off pH up relay
      digitalWrite(PH_DOWN_RELAY_PIN, LOW);  // Turn on pH down relay
    }

    // Check if pH adjustment pump has been active and its time is up
    if ((pHUpPumpActive && currentMillis - pHStartTime >= pumpDuration) || 
        (pHDownPumpActive && currentMillis - pHStartTime >= pumpDuration)) {
      // Turn off the active pump relay after 2 seconds
      if (pHUpPumpActive) {
        digitalWrite(PH_UP_RELAY_PIN, HIGH); // Turn off pH up relay
        pHUpPumpActive = false;
      } else if (pHDownPumpActive) {
        digitalWrite(PH_DOWN_RELAY_PIN, HIGH); // Turn off pH down relay
        pHDownPumpActive = false;
      }
      mixStartTime = currentMillis; // Start the mixing timer
      mixingInProgressPH = true;
    }

    // pH adjustment mixing
    if (mixingInProgressPH && currentMillis - mixStartTime >= mixDuration) {
      mixingInProgressPH = false;
      // Here you can add code to take pH reading again and continue
    }
  }

  // Control TDS down and TDS up relays based on TDS value
  if (tdsValue >= 400 && tdsValue <= 500) {
    digitalWrite(NUTRIENT_A_RELAY_PIN, HIGH);  // Turn off nutrient A relay
    digitalWrite(NUTRIENT_B_RELAY_PIN, HIGH);  // Turn off nutrient B relay
    digitalWrite(WATER_RELAY_PIN, HIGH);       // Turn off water relay
  } else {
    // TDS is outside the ideal range, proceed with adjustments
    if (tdsValue < 400 && !nutAPumpActive && !nutBPumpActive && !mixingInProgressTDS) {
      // TDS is below 400 ppm, trigger nutrient A and B adjustment
      nutAPumpActive = true;
      nutBPumpActive = true;
      TDSStartTime = currentMillis;  // Update TDS adjustment start time
      digitalWrite(NUTRIENT_B_RELAY_PIN, LOW);   // Turn on nutrient B relay
      digitalWrite(NUTRIENT_A_RELAY_PIN, LOW);   // Turn on nutrient A relay
      digitalWrite(WATER_RELAY_PIN, HIGH);       // Turn off water relay
      mixingInProgressTDS = true;                 // Start mixing
    } else if (tdsValue > 500 && !waterpumpActive && !mixingInProgressTDS) {
      // TDS is above 500 ppm, trigger water adjustment
      waterpumpActive = true;
      TDSStartTime = currentMillis;  // Update TDS adjustment start time
      digitalWrite(NUTRIENT_A_RELAY_PIN, HIGH);  // Turn off nutrient A relay
      digitalWrite(NUTRIENT_B_RELAY_PIN, HIGH);  // Turn off nutrient B relay
      digitalWrite(WATER_RELAY_PIN, LOW);        // Turn on water relay
      mixingInProgressTDS = true;                 // Start mixing
    }
  }

  // Check if pH or TDS adjustment pump has been active and its time is up
  if (((pHUpPumpActive || pHDownPumpActive) && (currentMillis - pHStartTime >= pumpDuration)) || 
      ((nutAPumpActive && nutBPumpActive) || waterpumpActive) && (currentMillis - TDSStartTime >= pumpDuration)) {
    // Turn off the active pump relay after 2 seconds
    if (pHUpPumpActive || pHDownPumpActive) {
      digitalWrite(PH_UP_RELAY_PIN, HIGH); // Turn off pH up relay
      digitalWrite(PH_DOWN_RELAY_PIN, HIGH); // Turn off pH down relay
      pHUpPumpActive = false;
      pHDownPumpActive = false;
    }
    if (nutAPumpActive || nutBPumpActive) {
      digitalWrite(NUTRIENT_A_RELAY_PIN, HIGH);  // Turn off nutrient A relay
      digitalWrite(NUTRIENT_B_RELAY_PIN, HIGH);  // Turn off nutrient B relay
      nutAPumpActive = false;
      nutBPumpActive = false;
    }
    if (waterpumpActive) {
      digitalWrite(WATER_RELAY_PIN, HIGH);       // Turn off water relay
      waterpumpActive = false;
    }
    mixStartTime = currentMillis; // Start the mixing timer
  }

  // pH and TDS adjustment mixing
  if ((mixingInProgressPH || mixingInProgressTDS) && currentMillis - mixStartTime >= mixDuration) {
    mixingInProgressPH = false;
    mixingInProgressTDS = false;
    // Here you can add code to take pH and TDS readings again and continue
  }
}

float getTemperature() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float getTDSValue() {
  static unsigned long analogSampleTimepoint = millis();

  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;

    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }

  for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

  averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4095.0;  // Adjusted for 12-bit ADC on ESP32

  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = averageVoltage / compensationCoefficient;
  // Adjusted TDS calculation with scaling factor
  float scaledTDS = (133.42 * pow(compensationVoltage, 3) - 255.86 * pow(compensationVoltage, 2) + 857.39 * compensationVoltage) * 0.5;
  float adjustedTDS = scaledTDS * 3.5; // Scaling factor

  return adjustedTDS;

}

float getPHValue() {
  static unsigned long samplingTime = millis();
  static float voltage;

  if (millis() - samplingTime > samplingInterval) {
    pHArray[pHArrayIndex++] = analogRead(SensorPin);
    if (pHArrayIndex == ArrayLenth) pHArrayIndex = 0;
    voltage = avergearray(pHArray, ArrayLenth) * 5.0 / 1024;
    samplingTime = millis();
  }

  return 1.321 * voltage + Offset;
}

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

  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;

  return bTemp;
}

double avergearray(int* arr, int number) {
  int i;
  int max, min;
  double avg;
  long amount = 0;

  if (number <= 0) {
    Serial.println("Error number for the array to averaging!\n");
    return 0;
  }

  if (number < 5) {   // less than 5, calculated directly statistics
    for (i = 0; i < number; i++) {
      amount += arr[i];
    }
    avg = amount / number;
    return avg;
  }
  else {
    if (arr[0] < arr[1]) {
      min = arr[0]; max = arr[1];
    }
    else {
      min = arr[1]; max = arr[0];
    }

    for (i = 2; i < number; i++) {
      if (arr[i] < min) {
        amount += min;        // arr < min
        min = arr[i];
      }
      else {
        if (arr[i] > max) {
          amount += max;    // arr > max
          max = arr[i];
        }
        else {
          amount += arr[i]; // min <= arr <= max
        }
      }
    }

    avg = (double)amount / (number - 2);
  }
  return avg;
}


void sendToFirebase(String sensorID, float value) {
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char timestamp[30];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%d_%H-%M-%S", timeinfo);
  String path = "Sensors/Live/" + sensorID + "/";

  Firebase.setFloat(firebaseData, path + String(timestamp), value);
}