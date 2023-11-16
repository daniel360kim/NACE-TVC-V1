/*
These functions are to control the outputs of the NACE Flight computer: the LED, Buzzer, pyro channel, and servo outputs
There are some cool LED driving functions and this uses the Buzzer library for Arduinos /.pio/libdeps
Also initializes Serial port for comms with a computer
*/

//Functions for use of the onboard LED aboard the NACE Flight Computer
#ifndef Outputs_h
#define Outputs_h
#include <Arduino.h>
#include <Buzzer.h>
#include <Servo.h>
#include <Settings.h>

//creating servo objects for the two thrust vector control servo motors
Servo servoX;
Servo servoY;

//assinging pinouts of the mcu to vars
const uint8_t redLED = 8;
const uint8_t grnLED = 18;
const uint8_t bluLED = 9;
//these two vars are for a totally "useless" (but cool) function :)
int counter = 0;
int numColors = 255; 
const unsigned long animationDelayMillis = 5;
unsigned long previousAnimationMillis = 0;

//creating buzzer object and assigning pinout
Buzzer buzzer(5);

//pinouts for the pyro continiuty pin and firing pin
const uint8_t pyroCont = A2;
const uint8_t pyro = 4;

//tracking the LED state
bool LEDstate = false;

void initializeOutputs(){

 Serial.begin(115200);
 
 //setting pin modes
 pinMode(bluLED, OUTPUT);
 pinMode(grnLED, OUTPUT);
 pinMode(redLED, OUTPUT);

 pinMode(pyro, OUTPUT);
 pinMode(12, OUTPUT);
 pinMode(A2, INPUT);
 digitalWrite(pyro, LOW);
 digitalWrite(12, HIGH);//disable auto-retry functionality on load driver

 pinMode(13, OUTPUT); //servo 1
 pinMode(11, OUTPUT); //servo 2

 //attaching and initializing servo motors
 servoX.attach(11);
 servoY.attach(13);
 servoX.write(servoXstart);
 servoY.write(servoYstart);

}

//sets pyro channel off, so it doesn't fire in my face
void allPyrosLow(){
    digitalWrite(pyro, LOW);
}

//RGB to color: analog values map perfectly to RGB and to uint8_ts...
void setColor(uint8_t R, uint8_t G, uint8_t B) {
  analogWrite(redLED, R);
  analogWrite(grnLED, G);
  analogWrite(bluLED, B);
  LEDstate = true;
}

void LEDoff() { //self explanatory
    setColor(0,0,0);
    LEDstate = false;
}

void setLEDbrightness(uint8_t R, uint8_t G, uint8_t B, uint8_t intensity){ //scale of 0 - 100
    R = (R * intensity) / 100;
    G = (G * intensity) / 100;
    B = (B * intensity) / 100;

    setColor(R, G, B);
}

//blinks the LED without delay() function
void blink(uint8_t R, uint8_t G, uint8_t B, unsigned long intervalmillis){
    unsigned long currentMillis = millis();
    unsigned long previousMillis = 0;

    if(currentMillis - previousMillis >= intervalmillis){
        previousMillis = currentMillis;

        if(LEDstate == false){
            setColor(R,G,B);
        } else{
            LEDoff();
        }

    }

}

//random colors at a delay
void randomColors(unsigned int delaymillis){
    setColor(random(0,255), random(0,255), random(0,255));
    delay(delaymillis);
    
} 

//These next two functions are for a smooth sequence of LEDs that looks pretty cool
long HSBtoRGB(float _hue, float _sat, float _brightness) {
    float red = 0.0;
    float green = 0.0;
    float blue = 0.0;
    
    if (_sat == 0.0) {
        red = _brightness;
        green = _brightness;
        blue = _brightness;
    } else {
        if (_hue == 360.0) {
            _hue = 0;
        }

        int slice = _hue / 60.0;
        float hue_frac = (_hue / 60.0) - slice;

        float aa = _brightness * (1.0 - _sat);
        float bb = _brightness * (1.0 - _sat * hue_frac);
        float cc = _brightness * (1.0 - _sat * (1.0 - hue_frac));
        
        switch(slice) {
            case 0:
                red = _brightness;
                green = cc;
                blue = aa;
                break;
            case 1:
                red = bb;
                green = _brightness;
                blue = aa;
                break;
            case 2:
                red = aa;
                green = _brightness;
                blue = cc;
                break;
            case 3:
                red = aa;
                green = bb;
                blue = _brightness;
                break;
            case 4:
                red = cc;
                green = aa;
                blue = _brightness;
                break;
            case 5:
                red = _brightness;
                green = aa;
                blue = bb;
                break;
            default:
                red = 0.0;
                green = 0.0;
                blue = 0.0;
                break;
        }
    }


    long ired = red * 255.0;
    long igreen = green * 255.0;
    long iblue = blue * 255.0;
    
    return long((ired << 16) | (igreen << 8) | (iblue));
}
//about this function: very very very useless and prob chonks up sram and compile time and my personal life, but its super cool soooooooooooo :)
void displaySpectrum(){
  unsigned long currentAnimationMillis = millis();
  if(currentAnimationMillis - previousAnimationMillis >= animationDelayMillis){
      previousAnimationMillis = currentAnimationMillis;
      float colorNumber = counter > numColors ? counter - numColors: counter;
  
  // Play with the saturation and brightness values
  // to see what they do
      float saturation = 1; // Between 0 and 1 (0 = gray, 1 = full color)
      float brightness = 1; // Between 0 and 1 (0 = dark, 1 is full brightness)
      float hue = (colorNumber / float(numColors)) * 360; // Number between 0 and 360
      long color = HSBtoRGB(hue, saturation, brightness); 
  
  // Get the red, blue and green parts from generated color
      int red = color >> 16 & 255;
      int green = color >> 8 & 255;
      int blue = color & 255;

      setColor(red, green, blue);
  
  // Counter can never be greater then 2 times the number of available colors
  // the colorNumber = line above takes care of counting backwards (nicely looping animation)
  // when counter is larger then the number of available colors
      counter = (counter + 1) % (numColors * 2);
  
  // If you uncomment this line the color changing starts from the
  // beginning when it reaches the end (animation only plays forward)
   //counter = (counter + 1) % (numColors);
  }
}
// used for function above...
void setSolidRGB(uint8_t color){
    if(color == 1){
        setColor(255, 0, 0);
    }

    if(color == 2){
        setColor(0,255,0);
    }
    
    if(color == 3){
        setColor(0,0,255);
    }
}
////////////////////All Sequences: Combination of LEDs and Buzzer////////////////
void startupSequence(){ //Self explanatory
  setSolidRGB(1);
  buzzer.sound(NOTE_C5, 600);
  LEDoff();
  delay(100);

  setSolidRGB(2);
  buzzer.sound(NOTE_G4, 350);
  delay(100);

  setSolidRGB(3);
  buzzer.sound(NOTE_G5, 700);
  setColor(255, 255, 255);
}

void indicateError() { //Buzzer and LEDs show the user that there is some type of error
  while(1){
    buzzer.begin(0);
    setSolidRGB(1);
 
    buzzer.sound (NOTE_C7, 100);
 
    LEDoff();
    buzzer.end(100);
  }
}

void indicateCompleteStartup(){ //self explanatory
      setSolidRGB(2);
 
  buzzer.sound(NOTE_D5, 150);
 
  setSolidRGB(3);
 
  buzzer.sound(NOTE_F5, 250);
 
  LEDoff();
}

void indicateCompleteDatalog(){ //self explanatory
    buzzer.sound(NOTE_D7, 50);
    buzzer.sound(NOTE_D7, 50);

    randomColors(0);
}

void indicateLanding(){ //self explanatory
    buzzer.sound(NOTE_A7, 10);
    randomColors(0);
}

float degs2us(float degrees){ //converts degrees outputted from PID loop to microseconds which is the unit used by the Servo motors
    return 1000.0 + degrees * 50.0/9.0;
}

void ServoXYwrite(float angleDegsX, float angleDegsY){ //writes the microseconds to the servos using PWM
    servoX.writeMicroseconds(degs2us(angleDegsX));
    servoY.writeMicroseconds(degs2us(angleDegsY));
}
void firePyro(){ //sets pyro channel high and fires it (hopefully not in my face)
    digitalWrite(pyro, HIGH);
}

void detachServos(){ //detaches servos from the pin so that they are not locked up 
  servoX.detach();
  servoY.detach();
}
#endif