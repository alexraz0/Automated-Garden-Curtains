#include <Arduino.h>
#include <BH1750.h>
#include "DHT.h"

// Tempurature and Humidity Sensor DHT11
#define DHTPIN 2 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11  // DHT11

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor

const int target_Temperature = 25.0; // Desired target temperature
const int target_Humidity = 60.0; // Desired target humidity
const int target_Lux = 3000;// Desired target Light

// Light Sensor BH1750
const unsigned long SamplePeriod = 1000; // Sampling period in milliseconds

BH1750 GY302; // Initialize BH1750 object


// Cross reference classification with Database 

String plant_type = "tomato"; // replace with actual method of reading plant type

// Hysteresis values for different plant types
// Add more types and values as needed
float high_hysteresis_plant;
float low_hysteresis_plant;

float high_hysteresis_def = 1.0; // zone 1 plant hysteresis
float low_hysteresis_def = 0.0;

float high_hysteresis_1 = 1.1; // zone 2 plant hysteresis
float low_hysteresis_1 = 0.1;

float high_hysteresis_2 = 1.2; // zone 3 plant hysteresis
float low_hysteresis_2 = 0.2;

float high_hysteresis_3 = 1.3; // default plant hysteresis
float low_hysteresis_3 = 0.3;

float hysteresis[4][2] = {
  {high_hysteresis_def, low_hysteresis_def},
  {high_hysteresis_1, low_hysteresis_1},
  {high_hysteresis_2, low_hysteresis_2},
  {high_hysteresis_3, low_hysteresis_3}
};


// Pin Definition for the Shift Register (165)
const uint8_t ISRDataPin = 3;   // connected to 74HC165 QH (9) pin
const uint8_t ISRLatchPin = 4;  // connected to 74HC165 SH/LD (1) pin
const uint8_t ISRClockPin = 5;  // connected to 74HC165 CLK (2) pin

// Pin definition for the Shift Register (595)
const uint8_t OSRDataPin = 6;   // connected to 74HC595 SER (14) pin
const uint8_t OSRLatchPin = 7;  // connected to 74HC595 RCLK (12) pin
const uint8_t OSRClockPin = 8;  // connected to 74HC595 SRCLK (11) pin

// Pin definition for the Limit Switches
const uint8_t MOTOR1_SW_UP = 0;  // bit position for 74HC165 Motor 1 Limit Switch UP
const uint8_t MOTOR1_SW_DN = 1;  // bit position for 74HC165 Motor 1 Limit Switch DN

const uint8_t MOTOR2_SW_UP = 2;  // bit position for 74HC165 Motor 2 Limit Switch UP
const uint8_t MOTOR2_SW_DN = 3;  // bit position for 74HC165 Motor 2 Limit Switch DN

const uint8_t MOTOR3_SW_UP = 4;  // bit position for 74HC165 Motor 3 Limit Switch UP
const uint8_t MOTOR3_SW_DN = 5;  // bit position for 74HC165 Motor 3 Limit Switch DN


// Define values for the shift registers
const uint8_t shiftReg1Value = 0; // (165N)
const uint8_t shiftReg2Value = 0; // (595N)


void setup() {

  // Serial Monitor
  Serial.begin(9600);  // initialize serial bus
  while (!Serial);     // wait for serial connection


  //Initialize DHT module
  Serial.println(F("DHT Test!"));
  dht.begin(); 

  //Initialize GY-302 module
  GY302.begin(); 

  // 74HC165 shift register
  setupISR();
 // 74HC595 shift register
  setupOSR();

  // Turn off all motors initially
  osrWriteRegister(0);

  // Get Hysteresis values for the plant types
  getHysteresis();
   
}


void loop() {

 // --------------------------
 // Temperature and Humidity 
 //---------------------------
 // Read temperature and humidity from DHT sensor
 float* values = readDHT();
 float temperature = values[0];
 float humidity = values[1];
 float heatIndex = values[2];

 serialPrint(humidity, temperature, heatIndex);


 // Light Sensor
 int lux = ReadGY302();


 //------------------
 // Hysteresis
 // The difference between the actual value and the expected value
 //------------------

 String plant_type = "tomato";

 // Get Hysteresis values for the plant type



 // Calculate temperature difference
 float tempDiff = temperature - 25;

 // --------------------------
 // Reed Switches State
 //---------------------------

   bool MOTOR1_SW_UP_STATE = isrDigitalRead(MOTOR1_SW_UP);
   bool MOTOR1_SW_DN_STATE = isrDigitalRead(MOTOR1_SW_DN);

   bool MOTOR2_SW_UP_STATE = isrDigitalRead(MOTOR2_SW_UP);
   bool MOTOR2_SW_DN_STATE = isrDigitalRead(MOTOR2_SW_DN);

   bool MOTOR3_SW_UP_STATE = isrDigitalRead(MOTOR3_SW_UP);
   bool MOTOR3_SW_DN_STATE = isrDigitalRead(MOTOR3_SW_DN);

   // Read and print inputs at the specified sampling rate
   static unsigned long previousTime = 0;
   unsigned long currentTime = millis();
   if (currentTime - previousTime >= SamplePeriod) {
      readInputsWithDigitalRead();
      previousTime = currentTime;
   }
   delay(100);   

 //-----------------------------------------------------------------------------------
 // Motor State Controller
 //-----------------------------------------------------------------------------------
 // Temperature conditions 
 // The current temperature is greater than the optimal plant temperature or current light is less than optimal plant lighting
  if( temperature > target_Temperature + high_hysteresis_plant || lux < target_Lux - low_hysteresis_plant ) { 
    // Determine the direction of motor 1 movement
   
    if (MOTOR1_SW_UP_STATE == LOW && MOTOR1_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor1Control(1);
    } 
    else if (MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE ==  LOW){
      // motor stays at current position
     motor1Control(0);
    }
    else {
      // motor default
      motor1Control(0);
    }

    // Determine the direction of motor 2 movement
    if (MOTOR2_SW_UP_STATE == LOW && MOTOR2_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor2Control(1);
    } 
    else if (MOTOR2_SW_UP_STATE == HIGH && MOTOR2_SW_DN_STATE ==  LOW) {
      // Motor stays at current position
      motor2Control(0);
    } 
    else {
      // motor default
      motor2Control(1);
    }

    // Determine the direction of motor 3 movement
    if (MOTOR3_SW_UP_STATE == LOW && MOTOR3_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor3Control(2);
    } 
    else if (MOTOR3_SW_UP_STATE == HIGH && MOTOR3_SW_DN_STATE ==  LOW) {
      // Motor stays at current position
      motor3Control(0);
    } 
    else {
      // motor default
      motor3Control(1);
    }
  }  

 // The current temperature is less than the optimal plant temperature or current light is greater than optimal plant lighting
 else if(temperature < target_Temperature - low_hysteresis_plant || lux > target_Lux + high_hysteresis_plant) { 
  // Determine the direction of motor 1 movement
  if (MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE ==  LOW) {
    // Move motor down
   motor1Control(2);
  } 
  else if (MOTOR1_SW_UP_STATE == LOW && MOTOR1_SW_DN_STATE ==  HIGH) {
    // Motor stays in at current position
   motor1Control(0);
  } 
  else {
    // Motor default
    motor1Control(0);
  }

   // Determine the direction of motor 2 movement
  if (MOTOR2_SW_UP_STATE == HIGH && MOTOR2_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor2Control(2);
  } 
    else if (MOTOR2_SW_UP_STATE == LOW && MOTOR2_SW_DN_STATE ==  HIGH) {
    // Motor stays in at current position
   motor2Control(0);
  } 
  else {
    // Motor default
    motor2Control(0);
  }

   // Determine the direction of motor 3 movement
  if (MOTOR3_SW_UP_STATE == HIGH && MOTOR3_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor3Control(2);
  } 
  else if (MOTOR3_SW_UP_STATE == LOW && MOTOR3_SW_DN_STATE ==  HIGH) {
    // Motor stays in at current position
   motor3Control(0);
  } 
  else {
    // Motor default
    motor3Control(0);
  }

 }

 //---------------------------
 // Humidity conditions 
 //---------------------------
 // The current humidity is greater than the optimal plant humidity or current light is less than optimal plant lighting
  else if( humidity > target_Humidity + high_hysteresis_plant || lux < target_Lux - low_hysteresis_plant) { 
    if (MOTOR1_SW_UP_STATE == LOW && MOTOR1_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor1Control(1);
    } 
    else if (MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE ==  LOW){
      // motor stays at current position
     motor1Control(0);
    }
    else {
      // motor default
      motor1Control(0);
    }

    // Determine the direction of motor 2 movement
    if (MOTOR2_SW_UP_STATE == LOW && MOTOR2_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor2Control(1);
    } 
    else if (MOTOR2_SW_UP_STATE == HIGH && MOTOR2_SW_DN_STATE ==  LOW) {
      // Motor stays at current position
      motor2Control(0);
    } 
    else {
      // motor default
      motor2Control(0);
    }

    // Determine the direction of motor 3 movement
    if (MOTOR3_SW_UP_STATE == LOW && MOTOR3_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor3Control(2);
    } 
    else if (MOTOR3_SW_UP_STATE == HIGH && MOTOR3_SW_DN_STATE ==  LOW) {
      // Motor stays at current position
      motor3Control(0);
    } 
    else {
      // motor default
      motor3Control(0);
    }
  }  

  // The current humidity is less than the optimal plant temperature or current light is greater than optimal plant lighting
 else if(humidity < target_Humidity - low_hysteresis_plant || lux > target_Lux - high_hysteresis_plant) { // if tempurature and humidity are higher than hysteresis
  // Determine the direction of motor 1 movement
  if (MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE ==  LOW) {
    // Move motor down
   motor1Control(2);
  } 
  else if (MOTOR1_SW_UP_STATE == LOW && MOTOR1_SW_DN_STATE ==  HIGH) {
    // Motor stays in at current position
   motor1Control(0);
  } 
  else {
    // Motor default
    motor1Control(0);
  }

   // Determine the direction of motor 2 movement
  if (MOTOR2_SW_UP_STATE == HIGH && MOTOR2_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor2Control(2);
  } 
    else if (MOTOR2_SW_UP_STATE == LOW && MOTOR2_SW_DN_STATE ==  HIGH) {
    // Motor stays in at current position
   motor2Control(0);
  } 
  else {
    // Motor default
    motor2Control(0);
  }

   // Determine the direction of motor 3 movement
  if (MOTOR3_SW_UP_STATE == HIGH && MOTOR3_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor3Control(2);
  } 
  else if (MOTOR3_SW_UP_STATE == LOW && MOTOR3_SW_DN_STATE ==  HIGH) {
    // Motor stays in at current position
   motor3Control(0);
  } 
  else {
    // Motor default
    motor3Control(0);
  }
 }

}


//---------------------------------------------------------------------------------
//----------------- DHT11 Functions -----------------------------------------------
//---------------------------------------------------------------------------------

float* readDHT(){
  static float values[3]; // Static array to store temperature, humidity, and heat index values
  // Read temperature and humidity from DHT sensor
  float celsius = dht.readTemperature(); // Read temperature as Celsius
  float temperature = dht.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)
  float humidity = dht.readHumidity();

    // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(celsius) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    values[0] = NAN;
    values[1] = NAN;
    values[2] = NAN;

    return values;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(temperature, humidity);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(celsius, humidity, false);

  // Store temperature, humidity, and heat index values in array
  values[0] = temperature;
  values[1] = humidity;
  values[2] = hif;

  return values;

}

void serialPrint(float humidity, float temperature, float hif){
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hif);
  Serial.println(F("°F")); 
}


//---------------------------------------------------------------------------------
//----------------- GY302 Functions -----------------------------------------------
//---------------------------------------------------------------------------------


uint8_t ReadGY302(){
  uint16_t lux = GY302.readLightLevel();
  // Display to Serial Monitor
   Serial.print("Light: ");
   Serial.print(lux);
   Serial.println(" lx");

   delay(1000);

   return lux;

}

//---------------------------------------------------------------------------------
//----------------- Hysteresis Functions ------------------------------------------
//---------------------------------------------------------------------------------

void getHysteresis(){
 // Get Hysteresis values for the plant types
  if (plant_type == "tomato") {
    high_hysteresis_plant = hysteresis[1][0];
    low_hysteresis_plant = hysteresis[0][1];
  }
  else if (plant_type == "lettuce") {
    high_hysteresis_plant = hysteresis[2][0];
    low_hysteresis_plant = hysteresis[0][2];
  } 
  else if (plant_type == "orange") {
    high_hysteresis_plant = hysteresis[3][0];
    low_hysteresis_plant = hysteresis[0][3];
 }
  else {
    // Default hysteresis values if plant is not recognized
    high_hysteresis_plant = hysteresis[0][0];
    low_hysteresis_plant = hysteresis[0][0];

  }
}


//---------------------------------------------------------------------------------
//----------------- Reed(Magnetic Switch) Functions -----------------------------------------------
//---------------------------------------------------------------------------------
void setupISR(){
  pinMode(ISRDataPin, INPUT);
  pinMode(ISRLatchPin, OUTPUT);
  pinMode(ISRClockPin, OUTPUT);
}

uint8_t isrReadRegister() {
   uint8_t inputs = 0;
   digitalWrite(ISRClockPin, HIGH);  // preset clock to retrieve first bit
   digitalWrite(ISRLatchPin, HIGH);  // disable input latching and enable shifting
   inputs = shiftIn(ISRDataPin, ISRClockPin, MSBFIRST);  // capture input values
   digitalWrite(ISRLatchPin, LOW);  // disable shifting and enable input latching
   return inputs;
}

int isrDigitalRead(uint8_t pin) {
   return bitRead(isrReadRegister(), pin);
}

void readInputsWithDigitalRead() {
   // Read and print individual inputs
   Serial.print("MOTOR1_SW_UP = ");  Serial.println(isrDigitalRead(MOTOR1_SW_UP) ? "HIGH" : "LOW");
   Serial.print("MOTOR1_SW_DN = ");  Serial.println(isrDigitalRead(MOTOR1_SW_DN) ? "HIGH" : "LOW");
   Serial.print("MOTOR2_SW_UP = ");  Serial.println(isrDigitalRead(MOTOR2_SW_UP) ? "HIGH" : "LOW");
   Serial.print("MOTOR2_SW_DN = ");  Serial.println(isrDigitalRead(MOTOR2_SW_DN) ? "HIGH" : "LOW");
   Serial.print("MOTOR3_SW_UP = ");  Serial.println(isrDigitalRead(MOTOR3_SW_UP) ? "HIGH" : "LOW");
   Serial.print("MOTOR3_SW_DN = ");  Serial.println(isrDigitalRead(MOTOR3_SW_DN) ? "HIGH" : "LOW");

   Serial.println();
}

void printInputswithDigitalRead(){

    // Read and print inputs at the specified sampling rate
    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - previousTime >= SamplePeriod) {
      readInputsWithDigitalRead();
      previousTime = currentTime;
   }
}
//---------------------------------------------------------------------------------
//----------------- Motor Functions -----------------------------------------------
//---------------------------------------------------------------------------------

void setupOSR(){
  pinMode(OSRDataPin, OUTPUT);
  pinMode(OSRLatchPin, OUTPUT);
  pinMode(OSRClockPin, OUTPUT);
}

void osrWriteRegister(uint8_t outputs) {
  // Initiate latching process, next HIGH latches data
  digitalWrite(OSRLatchPin, LOW);
  // Shift output data into the shift register, most significant bit first
  shiftOut(OSRDataPin, OSRClockPin, MSBFIRST, outputs);
  // Latch outputs into the shift register
  digitalWrite(OSRLatchPin, HIGH);
}

void osrDigitalWrite(uint8_t pin, uint8_t value) {
  static uint8_t outputs = 0;  // retains shift register output values

  if (value == HIGH) bitSet(outputs, pin);  // set output pin to HIGH
  else if (value == LOW) bitClear(outputs, pin);  // set output pin to LOW
  osrWriteRegister(outputs);  // write all outputs to shift register
}

void motor1Control(uint8_t inputvalue1){

  // Output pin definitions
  const uint8_t MOTOR1_IN1 = 0;
  const uint8_t MOTOR1_IN2 = 1;

  unsigned long startTime = 0;
  const unsigned long MOTOR_RUN_TIME = 8000;

  if(inputvalue1 == 0){ // motor stays off

   osrDigitalWrite(MOTOR1_IN1, LOW);
   osrDigitalWrite(MOTOR1_IN2, LOW);
   delay(MOTOR_RUN_TIME);

 }

  else if(inputvalue1 == 1){ // if magnetic switch1 is High and magnetic switch2 is Low
  // motor moves up 
   osrDigitalWrite(MOTOR1_IN1, HIGH);
   osrDigitalWrite(MOTOR1_IN2, LOW);
   startTime = millis();
   while(millis() - startTime < MOTOR_RUN_TIME){
      // do nothing and run motor for 8 sec
   }
  }

  else if(inputvalue1 == 2){ // if magnetic switch1 is High and magnetic switch2 is Low
  // motor moves down
   osrDigitalWrite(MOTOR1_IN1, LOW);
   osrDigitalWrite(MOTOR1_IN2, HIGH);
   startTime = millis();
   while(millis() - startTime < MOTOR_RUN_TIME){
      // do nothing and run motor for 8 sec
   }
  }

}

void motor2Control(uint8_t inputvalue2){

  // Output pin definitions
  const uint8_t MOTOR2_IN1 = 2;
  const uint8_t MOTOR2_IN2 = 3;

  unsigned long startTime = 0;
  const unsigned long MOTOR_RUN_TIME = 8000;

  if(inputvalue2 == 0){
                             // motor stays off 
   osrDigitalWrite(MOTOR2_IN1, LOW);
   osrDigitalWrite(MOTOR2_IN2, LOW);
   delay(MOTOR_RUN_TIME);

  }

  else if(inputvalue2 == 1){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves up  
   osrDigitalWrite(MOTOR2_IN1, HIGH);
   osrDigitalWrite(MOTOR2_IN2, LOW);
   startTime = millis();
   while(millis() - startTime < MOTOR_RUN_TIME){
      // do nothing and run motor for 8 sec
   }

  }

  else if(inputvalue2 == 2){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves down 
   osrDigitalWrite(MOTOR2_IN1, LOW);
   osrDigitalWrite(MOTOR2_IN2, HIGH);
   startTime = millis();
   while(millis() - startTime < MOTOR_RUN_TIME){
      // do nothing and run motor for 8 sec
   }

  }

}

void motor3Control(uint8_t inputvalue3){

  // Output pin definitions
  const uint8_t MOTOR3_IN1 = 4;
  const uint8_t MOTOR3_IN2 = 5;

  unsigned long startTime = 0;
  const unsigned long MOTOR_RUN_TIME = 8000;

  if(inputvalue3 == 0){
                             // motor stays off 
   osrDigitalWrite(MOTOR3_IN1, LOW);
   osrDigitalWrite(MOTOR3_IN2, LOW);
   delay(MOTOR_RUN_TIME);

  }

  else if(inputvalue3 == 1){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves up 
   osrDigitalWrite(MOTOR3_IN1, HIGH);
   osrDigitalWrite(MOTOR3_IN2, LOW);
   startTime = millis();
   while(millis() - startTime < MOTOR_RUN_TIME){
      // do nothing and run motor for 8 sec
   }

  }

  else if(inputvalue3 == 2){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves down
   osrDigitalWrite(MOTOR3_IN1, LOW);
   osrDigitalWrite(MOTOR3_IN2, HIGH);
   startTime = millis();
   while(millis() - startTime < MOTOR_RUN_TIME){
      // do nothing and run motor for 8 sec
   }

  }

}


