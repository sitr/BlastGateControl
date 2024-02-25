/*
   This program is inspired by the work of Bob Clagett from I Like To Make Stuff 
   https://github.com/iliketomakestuff/iltms_automated_dust_collection/blob/master/DustCollectionAutomation_v2.ino

   It's heavily modified, because the AC current sensor he used, does not work well with 230V. I have replaced the
   sensors with two ADS1115 and current clamps, which gives me control over a total of 4 blast gates. I also have an additional
   blast gate controled with a push button. It's possible to hook up another 2 ADS115 so the total number of current-
   controlled blast gates can be 8.

   Uses https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library and 
   https://github.com/adafruit/Adafruit_ADS1X15
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_ADS1X15.h>
 
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Uses the default I2C address 0x40
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;

boolean collectorIsOn = false;
int dustCollectorSpinDown = 3000;

int activeTool = 50;
const int NUMBER_OF_TOOLS = 5;
const int NUMBER_OF_GATES = 5;
String tools[NUMBER_OF_TOOLS] = { "Søylebor", "Bordsag", "Båndsag", "Båndsliper", "Arbeidsbord" };
double ampThreshold = .20;
const int dustCollectionRelayPin = 11;
const int manualSwitchPin = 2;
bool manualToolOn = false;

int gateMinMax[NUMBER_OF_GATES][2] = {
  /*open, close*/
  { 210, 90 }, // Søylebor
  { 327, 223 }, // Bordsag
  { 298, 180 }, // Båndsag
  { 220, 100 }, // Båndsliper
  { 222, 106 }  // Arbeidsbord
};

// Keep track of gates to be toggled ON/OFF for each tool
int gates[NUMBER_OF_TOOLS][NUMBER_OF_GATES] = {
  { 1, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0 },
  { 0, 0, 1, 0, 0 },
  { 0, 0, 0, 1, 0 },
  { 0, 0, 0, 0, 1 }
};

bool buttonPressed = false;
volatile int buttonState = HIGH;
bool initialisation_complete = false;
const float FACTOR = 30;
const float multiplier = 0.0625F;

const int debounceTime = 50;  // Debounce in milliseconds
unsigned long startMillis;

boolean checkForAmperageChange(int which);
void turnOnDustCollection();
void turnOffDustCollection();
void closeGate(uint8_t num);
void openGate(uint8_t num);
void closeAllGates();
float getCorriente(int toolId);
void manualSwitchPressed();
void setGatePositions(int activeTool);

void setup() { 
   Serial.begin(9600);
   pinMode(dustCollectionRelayPin, OUTPUT);
   pwm.begin();
   pwm.setPWMFreq(50);
  
   ads1.setGain(GAIN_TWO);
   if (!ads1.begin(0x48)) { // Sets the I2C address to 0x48: ADDR=GND
      Serial.println("Failed to initialize ADS 1.");
      while (1);
   }

   ads2.setGain(GAIN_TWO);
   if (!ads2.begin(0x49)) { // Sets the I2C address to 0x49: ADDR=VCC
      Serial.println("Failed to initialize ADS 2.");
      while (1);
   }
   pinMode(manualSwitchPin, INPUT_PULLUP);
   EIFR |= bit(INTF0); // Clear interrupt flag
   closeAllGates();
   initialisation_complete = true;
   startMillis = millis();
   attachInterrupt(digitalPinToInterrupt(manualSwitchPin), manualSwitchPressed, FALLING);
   Serial.println("Initializing complete");
}

void loop() {
   // openGate(3);
   // delay(2000);
   // closeGate(3);
   // delay(2000);

   if (buttonPressed == true) {
      Serial.println("Button was pressed");
      if(manualToolOn == false) {
         manualToolOn = true;
         activeTool = 4;
      } 
      else {
         manualToolOn = false;
         activeTool = 50;
      }
      buttonPressed = false;
   }

   for (int i = 0; i < NUMBER_OF_TOOLS; i++) {
      if(checkForAmperageChange(i) && manualToolOn == false) {
         activeTool = i;
         break;
      }
   }

   if (activeTool != 50) {
      setGatePositions(activeTool);
   }
   else {
      if (collectorIsOn == true) {
         delay(dustCollectorSpinDown);
         turnOffDustCollection();
         delay(2000);
         closeAllGates();
      }
   }
}

void setGatePositions( int activeTool) {
   if (collectorIsOn == false) {
      // Manage all gate positions
      for (int s = 0; s < NUMBER_OF_GATES; s++) {
         int pos = gates[activeTool][s];
         if (pos == 1) {
            openGate(s);    
         }
         else {
            closeGate(s);
         }
      }
      // Wait a bit to let blast gate to open
      delay(500);
      turnOnDustCollection();
   }
}

void closeAllGates() {
   for(int i = 0; i < NUMBER_OF_GATES; i++) {
      closeGate(i);
   }
}

void manualSwitchPressed() {
   buttonState = digitalRead(manualSwitchPin);
   if ((millis() - startMillis) > debounceTime) {
      if (buttonState == HIGH) {
         buttonPressed = true;
         startMillis = millis();
      }
   }
}

boolean checkForAmperageChange(int which) {
   float currentRMS = getCorriente(which);
   if(currentRMS > ampThreshold) {
      return true;
   }
   return false; 
}

float getCorriente(int toolId) {
   float voltage;
   float Corriente;
   float sum = 0;
   long tiempo = millis();
   int counter = 0;
  
   while (millis() - tiempo < 1000) {
      if(toolId == 0) {
         voltage = ads1.readADC_Differential_0_1() * multiplier;
      }
      else if(toolId == 1) {
         voltage = ads1.readADC_Differential_2_3() * multiplier;
      }
      else if(toolId == 2) {
         voltage = ads2.readADC_Differential_0_1() * multiplier;
      }
      else if(toolId == 3) {
         voltage = ads2.readADC_Differential_2_3() * multiplier;
      }
      else {
         return 0;
      }
      Corriente = voltage * FACTOR;
      Corriente /= 1000.0;
      sum += sq(Corriente);
      counter = counter + 1;
   }

   Corriente = sqrt(sum / counter);
   return(Corriente);
}

void turnOnDustCollection() {
   digitalWrite(dustCollectionRelayPin, 1);
   collectorIsOn = true;
}

void turnOffDustCollection() {
   digitalWrite(dustCollectionRelayPin, 0);
   collectorIsOn = false;
}

void closeGate(uint8_t num) {
   pwm.setPWM(num, 0, gateMinMax[num][1]);
}

void openGate(uint8_t num) {
   pwm.setPWM(num, 0, gateMinMax[num][0]);
}