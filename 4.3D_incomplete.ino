#include <Arduino.h>
// Define pin numbers
const int buttonPin = 2;              // Push button pin
const int motionSensorPin = 3;        // Motion sensor pin
const int usTriggerPin = 4;           // Ultrasonic Trigger pin
const int usEchoPin = 5;              // Ultrasonic Echo pin
const int ssDO = 6;                   // Sound sensor DO pin
const int ledPin1 = 7;                // LED controlled by push button
const int ledPin2 = 8;                // LED controlled by sound sensor
const int ledPin3 = 9;		            // LED controlled by timer interrupt 

// Variables to keep track of things
volatile bool ledState1 = false; // State of LED controlled by push button
volatile bool ledState2 = false; // State of LED controlled by sound sensor
volatile bool ledState3 = false; // State of LED controlled by timer interrupt
volatile bool soundDetected = false; // Flag to indicate sound detection
volatile uint8_t timerCounter = 0; //count
const uint8_t timerThreshold = 10; // 10 seconds
const int DISTANCE_THRESHOLD = 20; // centimeters
float duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement

// Timer Configuration
#define TC_TIMER       TC3
#define TC_TIMER_IRQn  TC3_IRQn
#define TC_HANDLER     TC3_Handler

void setupTimer() {
  // Enable clock for TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK
                     GCLK_CLKCTRL_GEN_GCLK0 |     // Select GCLK0 (48MHz)
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK to TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Reset TC3
  TC_TIMER->COUNT16.CTRLA.bit.SWRST = 1;
  while (TC_TIMER->COUNT16.CTRLA.bit.SWRST);
  
  // Set TC3 in 16-bit mode
  TC_TIMER->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |   // 16-bit counter mode
                               TC_CTRLA_WAVEGEN_MFRQ |    // Match frequency mode
                               TC_CTRLA_PRESCALER_DIV1024; // Prescaler: 1024
  
  // Set the period (5 second toggle)
  // 48MHz/1024 = 46875 Hz, so for 1 second we need 46875 ticks
  TC_TIMER->COUNT16.CC[0].reg = 46875;
  while (TC_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
  
  // Configure interrupt
  NVIC_SetPriority(TC_TIMER_IRQn, 0);    // Set highest priority
  NVIC_EnableIRQ(TC_TIMER_IRQn);         // Enable the interrupt
  
  // Enable the TC3 interrupt
  TC_TIMER->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  
  // Enable TC3
  TC_TIMER->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
}

// Interrupt Service Routines (ISRs)
void toggleLed1() {
    delay(100);
    ledState1 = !ledState1; // Toggle LED state
    digitalWrite(ledPin1, ledState1); // Update LED state
    if(ledState1){
      Serial.println("Button pushed: LED 1 is ON");
    } else {
      Serial.println("Button pushed: LED 1 is OFF");
    }
    
}

void toggleLed2() {
    ledState2 = !ledState2; // Toggle LED state
    delay(100);
    digitalWrite(ledPin2, ledState2); // Update LED state
    if(ledState2){
      Serial.println("Motion detected: LED 2 is ON");
    } else {
      Serial.println("Motion detected: LED 2 is OFF");
    }
}

void toggleLed3() {
    delay(100);
    ledState3 = !ledState3; // Toggle LED state
    digitalWrite(ledPin3, ledState3); // Update LED state
    if(ledState3){
      Serial.println("Sound detected: LED 3 is ON");
    } else {
      Serial.println("Sound detected: LED 3 is OFF");
    }
}
void motionHandler() {
  Serial.println("Motion Detected!!!");
}
void soundSensorISR() {
    soundDetected = true; // Set the flag when sound is detected
}
// Timer Interrupt Service Routine
void TC_HANDLER() {
  // Check for match counter 0 (MC0) interrupt
  if (TC_TIMER->COUNT16.INTFLAG.bit.MC0) {
    // Clear the interrupt flag
    TC_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
    

    // Tăng bộ đếm
    timerCounter++;
    
    // Nếu đạt ngưỡng 5 giây, đảo trạng thái LED và reset bộ đếm
    if (timerCounter >= timerThreshold) {
      // Toggle LED
      ledState3 = !ledState3;
      digitalWrite(ledPin3, ledState3);
      
      // Reset bộ đếm
      timerCounter = 0;
      
      // Optional: Print to serial for debugging
      // if (ledState3) {
      //   Serial.println("LED ON");
      // } else {
      //   Serial.println("LED OFF");
      // }
    }
  }
}

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for Serial to be ready
    // Set pin modes
    pinMode(buttonPin, INPUT_PULLUP); // Use internal pull-up resistor
    pinMode(motionSensorPin, INPUT_PULLDOWN);  // Motion sensor input
    pinMode(usTriggerPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(usEchoPin, INPUT); // Sets the echoPin as an INPUT
    pinMode(ssDO, INPUT); // Sets the sound sensor DO pin as input
    pinMode(ledPin1, OUTPUT); // LED 1 output
    pinMode(ledPin2, OUTPUT); // LED 2 output
    pinMode(ledPin3, OUTPUT); // LED 3 output

    // Configure Timer (TC3)
    setupTimer();

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(motionSensorPin), motionHandler, RISING); // Trigger on motion detected 
    attachInterrupt(digitalPinToInterrupt(buttonPin), toggleLed1, FALLING); // Trigger on button press
    attachInterrupt(digitalPinToInterrupt(ssDO), soundSensorISR, RISING);
    Serial.println("4.3D project start!!!!");
}

void loop() {
  // Check if sound was detected
    if (soundDetected) {
        Serial.println("Sound detected!");
        soundDetected = false; // Reset the flag
    }
  delay(200); // Small delay to prevent the loop from running too fast
}
