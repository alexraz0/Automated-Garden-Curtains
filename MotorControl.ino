#include <SPI.h>
#include <BH1750.h>
#include <DHT.h>

// Tempurature and Humidity Sensor DHT11

#define DHTPIN 8 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11  // DHT11

DHT dht(DHTPIN, DHTTYPE);

const int tempLowThreshold = 24;
const int tempAVGThreshold = 25;
const int tempHighThreshold = 
const int humThreshold = 40;


// Light Sensor BH1750
const unsigned long SamplePeriod = 1000; // Sampling period in milliseconds

BH1750 GY302; // Initialize BH1750 object


// Pin Definition for the Shift Register (165)
const uint8_t ISRDataPin = 2;   // connected to 74HC165 QH (9) pin
const uint8_t ISRLatchPin = 3;  // connected to 74HC165 SH/LD (1) pin
const uint8_t ISRClockPin = 4;  // connected to 74HC165 CLK (2) pin

// Pin definition for the Shift Register (595)
const uint8_t OSRDataPin = 5;   // connected to 74HC595 SER (14) pin
const uint8_t OSRLatchPin = 6;  // connected to 74HC595 RCLK (12) pin
const uint8_t OSRClockPin = 7;  // connected to 74HC595 SRCLK (11) pin

// Pin definition for the Limit Switches
const uint8_t MOTOR1_SW_UP = 0;  // bit position for 74HC165 Motor 1 Limit Switch UP
const uint8_t MOTOR1_SW_DN = 1;  // bit position for 74HC165 Motor 1 Limit Switch DN

const uint8_t MOTOR2_SW_UP = 2;  // bit position for 74HC165 Motor 2 Limit Switch UP
const uint8_t MOTOR2_SW_DN = 3;  // bit position for 74HC165 Motor 2 Limit Switch DN

const uint8_t MOTOR3_SW_UP = 4;  // bit position for 74HC165 Motor 3 Limit Switch UP
const uint8_t MOTOR3_SW_DN = 5;  // bit position for 74HC165 Motor 3 Limit Switch DN


// Define values for the shift registers
const uint8_t shiftReg1Value = 0; // (165)
const uint8_t shiftReg2Value = 0; // (595)


void setup() {

   // Serial Monitor
   Serial.begin(9600);  // initialize serial bus
   while (!Serial);     // wait for serial connection

   // 74HC165 shift register
   pinMode(ISRDataPin, INPUT);
   pinMode(ISRLatchPin, OUTPUT);
   pinMode(ISRClockPin, OUTPUT);

   // 74HC595 shift register
   pinMode(OSRDataPin, OUTPUT);
   pinMode(OSRLatchPin, OUTPUT);
   pinMode(OSRClockPin, OUTPUT);

    // Turn off all motors initially
   osrWriteRegister(0);

   //Initialize GY-302 module
//   GY302.begin(); 

}

void loop() {

 // Read temperature and humidity from DHT sensor

 float tempurature = dht.readTemperature();
 float humidity = dht.readHumidity();

 // Calculate temperature difference
 float tempDiff = temperature - 25;

 // Calculate motor speed based on tempurature difference  
 int motorspeed = map(tempDiff, -10, 10, 0, 255); Maps tempDiff range (-10 to 10) to motorSpeed range (0 to 255)


 //   motor1Control(2);
  // Get the Reed Switches State

   bool MOTOR1_SW_UP_STATE = isrDigitalRead(MOTOR1_SW_UP);
   bool MOTOR1_SW_DN_STATE = isrDigitalRead(MOTOR1_SW_DN);

   bool MOTOR2_SW_UP_STATE = isrDigitalRead(MOTOR2_SW_UP);
   bool MOTOR2_SW_DN_STATE = isrDigitalRead(MOTOR2_SW_DN);

   bool MOTOR3_SW_UP_STATE = isrDigitalRead(MOTOR3_SW_UP);
   bool MOTOR3_SW_DN_STATE = isrDigitalRead(MOTOR3_SW_DN);


 // Control motor direction based on the temperature, humidity, and light intensity
  if(temperature >= tempThreshold || humidity >= humThreshold ) { // if tempurature and humidity are higher than threshold
    // Determine the direction of motor 1 movement
    if (MOTOR1_SW_UP_STATE == LOW && MOTOR1_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor1Control(2);
    } 
    else {
      motor1Control(1);
    }

    // Determine the direction of motor 1 movement
    if (MOTOR2_SW_UP_STATE == LOW && MOTOR2_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor2Control(2);
    } 
    else {
      motor2Control(1);
    }

    // Determine the direction of motor 1 movement
    if (MOTOR3_SW_UP_STATE == LOW && MOTOR3_SW_DN_STATE ==  HIGH) {
      // Move motor up
      motor3Control(2);
    } 
    else {
      motor3Control(1);
    }

    delay(100);
 }

 if(temperature >= tempThreshold || humidity >= humThreshold ) { // MEDIAN TEMPURTURE AND HUMIDITY
  // Determine the direction of motor 1 movement
  if (MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor1Control(2);
  } 
  else {
    motor1Control(1);
  }

   // Determine the direction of motor 1 movement
  if (MOTOR2_SW_UP_STATE == HIGH && MOTOR2_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor2Control(2);
  } 
  else {
    motor2Control(1);
  }

   // Determine the direction of motor 1 movement
  if (MOTOR3_SW_UP_STATE == HIGH && MOTOR3_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor3Control(2);
  } 
  else {
    motor3Control(1);
  }

  delay(100);
 }


  // Determine the direction of motor 1 movement
  if (MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE ==  LOW) {
    // Move motor up
   motor1Control(2);
  } 
  else {
    motor1Control(1);
  }
  delay(100);
 }
  delay(100);
 }
  // Determine the direction of motor 1 movement
  if (MOTOR1_SW_UP_STATE == HIGH && !MOTOR1_SW_DN_STATE == HIGH) {
    // Move motor up
   motor1Control(2);
  } 
  // else if (!MOTOR1_SW_UP_STATE == HIGH && MOTOR1_SW_DN_STATE == HIGH) {
  //   // Move motor down
  //  motor1Control(3);
  // }
  //  else {
  //   // Stop the motor
  //  motor1Control(1);
  // }

  // Determine the direction of motor 1 movement
  // if (MOTOR2_SW_UP_STATE == HIGH && !MOTOR2_SW_DN_STATE == HIGH) {
  //   // Move motor up
  //  motor2Control(2);
  // } 
  // else if (!MOTOR2_SW_UP_STATE == HIGH && MOTOR2_SW_DN_STATE == HIGH) {
  //   // Move motor down
  //  motor2Control(3);
  // }
  //  else {
  //   // Stop the motor
  //  motor2Control(1);
  // }

   // Read and print inputs at the specified sampling rate
   static unsigned long previousTime = 0;
   unsigned long currentTime = millis();
   if (currentTime - previousTime >= SamplePeriod) {
      readInputsWithDigitalRead();
      previousTime = currentTime;
   }
   delay(100);
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

}


//---------------------------------------------------------------------------------
//----------------- Reed(Magnetic Switch) Functions -----------------------------------------------
//---------------------------------------------------------------------------------

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

void printInputswithDigitalRead()
{
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

  if(inputvalue1 == 1){ // motor stays off

   osrDigitalWrite(MOTOR1_IN1, LOW);
   osrDigitalWrite(MOTOR1_IN2, LOW);
   delay(3000);

 }

  else if(inputvalue1 == 2){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves up 
   osrDigitalWrite(MOTOR1_IN1, HIGH);
   osrDigitalWrite(MOTOR1_IN2, LOW);
   delay(3000);

  }

  else if(inputvalue1 == 3){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves down
   osrDigitalWrite(MOTOR1_IN1, LOW);
   osrDigitalWrite(MOTOR1_IN2, HIGH);
   delay(3000);

  }

}

void motor2Control(uint8_t inputvalue2){

  // Output pin definitions
  const uint8_t MOTOR2_IN1 = 2;
  const uint8_t MOTOR2_IN2 = 3;

  if(inputvalue2 == 1){
                             // motor stays off 
   osrDigitalWrite(MOTOR2_IN1, LOW);
   osrDigitalWrite(MOTOR2_IN2, LOW);
   delay(3000);

  }

  else if(inputvalue2 == 2){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves up  
   osrDigitalWrite(MOTOR2_IN1, HIGH);
   osrDigitalWrite(MOTOR2_IN2, LOW);
   delay(3000);

  }

  else if(inputvalue2 == 3){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves down 
   osrDigitalWrite(MOTOR2_IN1, LOW);
   osrDigitalWrite(MOTOR2_IN2, HIGH);
   delay(3000);

  }

}

void motor3Control(uint8_t inputvalue3){

  // Output pin definitions
  const uint8_t MOTOR3_IN1 = 4;
  const uint8_t MOTOR3_IN2 = 5;

  if(inputvalue3 == 1){
                             // motor stays off 
   osrDigitalWrite(MOTOR3_IN1, LOW);
   osrDigitalWrite(MOTOR3_IN2, LOW);
   delay(3000);

  }

  else if(inputvalue3 == 2){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves up 
   osrDigitalWrite(MOTOR3_IN1, HIGH);
   osrDigitalWrite(MOTOR3_IN2, LOW);
   delay(3000);

  }

  else if(inputvalue3 == 3){ // if magnetic switch1 is High and magnetic switch2 is Low
                             // motor moves down
   osrDigitalWrite(MOTOR3_IN1, LOW);
   osrDigitalWrite(MOTOR3_IN2, HIGH);
   delay(3000);

  }

}


