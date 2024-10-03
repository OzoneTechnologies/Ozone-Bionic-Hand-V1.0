/*
* Copyright 2017, OYMotion Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,t
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "EMGFilters.h"

#define TIMING_DEBUG 1

#define SensorInputPin A0 // input pin number

// Library for servo use
#include <Servo.h>

// Digital pin for servo motor
int servo_pin = 9;

// constants won't change. They're used here to set pin numbers:
#define buttonPin 2  // the number of the pushbutton pin

// Threshold value that allows the hand to open and close.
// The scaled value of the muscle sensor's value is compared
// to this threshold value.
// Sample 6-year old was 20.0, Adult was 100.0
float toggle_threshold = 150;

// Current state of the hand's position
boolean hand_opened;
boolean hand_opened1;
boolean hand_opened2;
int Readhand;

// The angle of the servo in the opened and closed states
int opened_angle1 = 60, opened_angle2 = 70, closed_angle = 20;

// Servo object
Servo servo;

// Timer used to allow the muscle to relax before toggling the
// hand. Prevents toggling too quickly.
int servo_timer = 0;

// Maximum value of the timer value. Prevents overflow errors.
int timer_threshold = 2000;

EMGFilters myFilter;
// discrete filters must works with fixed sample frequence
// our emg filter only support "SAMPLE_FREQ_500HZ" or "SAMPLE_FREQ_1000HZ"
// other sampleRate inputs will bypass all the EMG_FILTER
int sampleRate = SAMPLE_FREQ_1000HZ;
// For countries where power transmission is at 50 Hz
// For countries where power transmission is at 60 Hz, need to change to
// "NOTCH_FREQ_60HZ"
// our emg filter only support 50Hz and 60Hz input
// other inputs will bypass all the EMG_FILTER
int humFreq = NOTCH_FREQ_50HZ;

// Calibration:
// put on the sensors, and release your muscles;
// wait a few seconds, and select the max value as the throhold;
// any value under throhold will be set to zero
static int Throhold = 200;

unsigned long timeStamp;
unsigned long timeBudget;


// variables will change:
int lastButtonState = HIGH; //the previous reading from the input pin
unsigned long lastDebounceTime = 0; //the last time the output was toggled
unsigned long debounceDelay = 20; // the debounce time; 
bool buttonState = HIGH;  // variable for reading the pushbutton status
byte tapCounter; // for saving num. of times the switch is pressed
int timediff; // for saving the time in between each press and release of the switch
bool Read1, Read2; // just two variables 
long double presstime, releasetime; // for saving millis at press and millis at release

const int ledPin = 14; // the onboard led (pin 13 for Arduino Uno)

// Variables will change:
int ledState = LOW;             // ledState used to toggle the LED
unsigned long previousMillis = 0;   // will store last time LED was updated
//unsigned long currentMillis; 
//bool enable;
int enable;
unsigned long interval = 1000;   // interval between toggles in milliseconds
int count1;                     // keep track of how many times the led has blinked
int maxBlink;         // number to blink

void setup() {
    /* add setup code here */
    myFilter.init(sampleRate, humFreq, true, true, true);

    // open serial
    Serial.begin(115200);

    // setup for time cost measure
    // using micros()
    timeBudget = 1e6 / sampleRate;
    // micros will overflow and auto return to zero every 70 minutes

    // Assign the servo to it's respective pin
  servo.attach(servo_pin);

  // Set default angle of servo as opened
  servo.write(opened_angle1);


  Serial.begin(9600); // serial monitor 
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP); // setting the button Pin 2 as input with internal pull up resistor

  // set the digital pin as output:
  pinMode(ledPin, OUTPUT); digitalWrite(ledPin, LOW);
  
  // Start the blinking
  //previousMillis = millis();
  count1 = 0;
  enable = true;
  maxBlink = 0;

  hand_opened = true;
  hand_opened1 = true;
  hand_opened2 = true;
  Readhand = 0;
  
}

void loop() {
    /* add main program code here */
    // In order to make sure the ADC sample frequence on arduino,
    // the time cost should be measured each loop
    /*------------start here-------------------*/
    timeStamp = micros();

    int Value = analogRead(SensorInputPin);

    // filter processing
    int DataAfterFilter = myFilter.update(Value);

    int envlope = sq(DataAfterFilter);
    // any value under throhold will be set to zero
    envlope = (envlope > Throhold) ? envlope : 0;

    timeStamp = micros() - timeStamp;
    if (TIMING_DEBUG) {
        // Serial.print("Read Data: "); Serial.println(Value);
        // Serial.print("Filtered Data: ");Serial.println(DataAfterFilter);
        Serial.print("Squared Data: ");
        Serial.println(envlope);
        // Serial.print("Filters cost time: "); Serial.println(timeStamp);
        // the filter cost average around 520 us
    }
  
  int reading = digitalRead(buttonPin);
  
  if(reading != lastButtonState) {
    //reset the debouncing timer
    lastDebounceTime = millis();
    
    }   
   if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state: 
      
      // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
    }
    }
    //Serial.println(buttonState);

    //when switch is pressed
    if (buttonState == 0 && Read2 == 0)
    {
      //presstime = millis(); // time from millis will save to presstime variable
      releasetime = millis(); //time from millis will be saved to releasetime variable
      Read1 = 0;
      Read2 = 1;
      tapCounter++; //tap counter will increase by 1
      //delay(10); //for avoiding debouncing of the switch
      }
      //when the switch is released 
      if (buttonState == 1 && Read1 == 0)
      {
        // releasetime = millis(); //time from millis will be saved to releasetime variable
        presstime = millis(); // time from millis will save to presstime variable
        Read1 = 1;
        Read2 = 0;

        //timediff = presstime - releasetime; //here we find the time gap between press and release
        timediff = releasetime - presstime; //here we find the time gap between press and release
        //Serial.printIn(timediff);
        //delay(10);
        }

      if ((millis() - presstime) > 400 && buttonState == 1) // wait for some time and if Switch is in release position
      {
        if (tapCounter == 1) //if tap counter is 1
        {
          if (timediff >= 400) // if time diff is larger than 400 then its a hold
          {
            Serial.println("Hold");
            hold(); // function to call when the button is hold
          }
          else //if timediff is less than 400 then its a single tap
          {
            Serial.println("single tap-Normal Mode");
            singleTap(); //funtion to call when the button is single taped
          }
         }
          else if (tapCounter == 2) //if tapcounter is 2
          {
            if (timediff >= 400) // if timediff is greater than 400 then its single tap and hold
            {
              Serial.println("single tap and hold-Freeze Mode");
              tapAndHold(); //function to call when the button is single tap and hold
              }
            else //if timediff is less than 400 then its just double tap
            {
              Serial.println("double tap-Grip Mode");
              doubleTap(); //function to call when doubletap
            }
        } 
        else if (tapCounter == 3) //if tapcounter is 3 //then its triple tap
    {
      Serial.println("triple tap - Freeze mode");
      tripleTap(); //fn to call when triple tap
    }
        tapCounter = 0;
        //timediff = 0;
      }
      lastButtonState = reading;

      

  // A 'currentMillis' variable is not needed, but often handy
//  int readingss = digitalRead(ledPin);
//  if (readingss != ledState){
//    currentMillis = millis();
//    }
  unsigned long currentMillis = millis();
 
  if(enable)                    // should we blink ?
  {
    if( currentMillis - previousMillis >= interval)  // millis-timer
    {
      //previousMillis = currentMillis;   

      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;

      // set the LED with the ledState of the variable:
      digitalWrite(ledPin, ledState);
    
      if( ledState == LOW)
      {
        count1++;
      }
      
      if( count1 >= maxBlink)
      {
        enable = false;   // enough with this blinking, stop it.
        count1 = 0;
      }
     previousMillis = currentMillis;
     
    }
  
  }
    if (envlope >= toggle_threshold && servo_timer == timer_threshold)
  {
    // Change position of hand
    if (Readhand == 1)
    {
    hand_opened = !hand_opened;
    //hand_condition();
    
    if (hand_opened)
      for(int pos1 = opened_angle1 * 2; pos1 > closed_angle * 2; pos1-=2)
      { // Closes the hand by gradually adjusting the written angle.     
        servo.write(pos1);
        delay(2);
      } 
    else 
      for(int pos1 = closed_angle * 2; pos1 < opened_angle1 * 2; pos1+=2) 
      { // Opens the hand by gradually adjusting the written angle.           
        servo.write(pos1);
        delay(2);
      }
    }

    else if (Readhand == 2)
    {
      hand_opened1 = !hand_opened1;
    if (hand_opened1)
      for(int pos = opened_angle2 * 2; pos > closed_angle * 2; pos-=2)
      { // Closes the hand by gradually adjusting the written angle.     
        servo.write(pos);
        delay(2);
      } 
    else 
      for(int pos = closed_angle * 2; pos < opened_angle2 * 2; pos+=2) 
      { // Opens the hand by gradually adjusting the written angle.           
        servo.write(pos);
        delay(2);
      }
    }

    else if (Readhand == 3)
    {
      hand_opened2 = !hand_opened2;
    if (hand_opened2)
      { // Closes the hand by gradually adjusting the written angle.  
        int p = servo.read();   
        servo.write(p);
        delay(2);
      }
    }

    
    // Reset the timer
    servo_timer = 0;
  }
  
  // Don't allow the servo_timer to get too big. Overflow errors
  // crash the Arduino.
  if (servo_timer < timer_threshold)
    servo_timer++;
  
  // Delay for the servo. Don't want to overload it.
  delay(1);

    /*------------end here---------------------*/
    // if less than timeBudget, then you still have (timeBudget - timeStamp) to
    // do your work
    delayMicroseconds(500);
    // if more than timeBudget, the sample rate need to reduce to
    // SAMPLE_FREQ_500HZ
}

void nolight()
{
  digitalWrite(ledPin, HIGH);
  Readhand = 0;
}

//void hand_condition()
//{
//  hand_opened;
//  }

void singleTap()
{
   nolight();
   enable = true;
   maxBlink = 1;
   //hand_opened = false;
   Readhand = 1; 
}
void doubleTap()
{
   nolight();
   enable = true;
   maxBlink = 2;
   //hand_opened1;
   Readhand = 2;  
}
void tapAndHold()
{
   nolight();
   //digitalWrite(ledPin_Yellow, LOW);
}
void hold()
{
   nolight();
   //digitalWrite(ledPin_Green, LOW);
}
void tripleTap()
{
   nolight();
   //digitalWrite(ledPin_Green, HIGH);
   enable = true;
   maxBlink = 10;  
   //hand_opened2;
   Readhand = 3;
   
   //hand_freeze = !hand_freeze;
   
}
