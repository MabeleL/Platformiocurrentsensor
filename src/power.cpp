
#include "Arduino.h"
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <stdio.h>

//analog pins
//#define LightSensor A0
#define currentSensor A1

//digital pins
//#define Light_LED 8
#define LED 13

//RN2483 UART pins
#define rxPin 8
#define txPin 7

SoftwareSerial lora = SoftwareSerial(rxPin, txPin);

const int led = 13;
String inputString = "";
boolean stringComplete;
String is;
String currentValueToSend;


//int thresholdValue = 700;
double vRMS = 0;
float voltageV, voltageC;
float instVoltage, instCurrent;
int sampleV, sampleC;
float sumI, sumV, sumP;

const int numberofSamples = 3000;

float realPower, apparentPower, ReactivePower;
float powerFactor, Voltage, Current;
unsigned long KWhTime, last_KWhTime;
float units = 12.45;


LiquidCrystal lcd(12,11,5,4,3,2);

void sendcmd(String data){
  Serial.println(data);
  lora.println(data);
  delay(200);
  Serial.println(lora.readStringUntil('\n'));
}

void sendmsg(String data){
	Serial.println(data);
	lora.println(data);
	delay(200);

	//radio tx command has two responses, command and transmission
	Serial.println(lora.readStringUntil('\n')); //command response
	Serial.println(lora.readStringUntil('\n')); //transmission response
}

String strtohex(String data){
	String sh;
	char ch;
	for(int i = 0;i<data.length();i++){
		ch = data.charAt(i);
		sh += String(ch,HEX);
	}
	return sh;

}

//LoRa mode to initialise status and parameters
void RN2483_init(){
      	sendcmd("radio set freq 868100000");
        sendcmd("radio set pwr 14");
        sendcmd("radio set sf sf12");
 	      sendcmd("radio set cr 4/5");
      	sendcmd("radio set bw 125");
      	sendcmd("mac pause");
}


void calculatePower(){
	for(int i = 0; i<numberofSamples; i++)
	{
		sampleV= 240;
		sampleC = analogRead(A1);

		voltageC = sampleC*5.0/1023.0;
		voltageV = sampleV*5.0/1023.0;

		instCurrent = (voltageC-2.5)/0.66;
		instVoltage = (voltageV-2.46)*7.8;

		sumV += instVoltage*instVoltage;
		sumI += instCurrent*instCurrent;

		sumP += abs(instVoltage * instCurrent);
	}


      Voltage = sqrt(sumV/numberofSamples);
      Current = sqrt(sumI/numberofSamples);

      realPower = sumP / numberofSamples;
      apparentPower = Voltage * Current;
      powerFactor = realPower/apparentPower;
      ReactivePower=sqrt(apparentPower*apparentPower - realPower*realPower);

      last_KWhTime = KWhTime;
      KWhTime = millis();

      units += (realPower/1000)*((KWhTime - last_KWhTime)/3600000.0);

      sumV = 0;
      sumI = 0;
      sumP = 0;

      Serial.print(Voltage);
      Serial.println(" V");
      Serial.print(Current);
      Serial.println(" A");
      Serial.print(realPower);
      Serial.println(" KW");
      Serial.print(units);
      Serial.println(" KWH");
}
void blinkLED(int ledID, int repeat, int wait) {
  if (repeat == 999) {
    digitalWrite(ledID, HIGH);
    return;
  }
  if (repeat == 0) {
    digitalWrite(ledID, LOW);
    return;
  }
  for (int i = 0; i < repeat; i++) {
    digitalWrite(ledID, HIGH);
    delay(wait);
    digitalWrite(ledID, LOW);
    delay(wait);
  }
  delay(1000);
}



 void setup()
{
	  // initialize LED digital pin as an output.
	  lora.begin(57600);
	  Serial.begin(115200);
	  delay(2000);
	  //pinMode(8, OUTPUT);
	  pinMode(13, OUTPUT);
    lcd.begin(16,2);
	  // lcd.setCursor(0,1);
	 // lcd.print("LED blinking");
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);
    delay(50);
    digitalWrite(9, LOW);
    delay(50);
    digitalWrite(9, HIGH);
    delay(500);
	  RN2483_init();
	     }

void loop()
	 {
  //read current sensor
int sensorValue = analogRead(A1);

/*  if(sensorValue>thresholdValue){
	  digitalWrite(13, HIGH);
  }
  else{
	  digitalWrite(13, LOW);
  }*/
   Serial.print(sensorValue);
   Serial.println("Amperes");

   calculatePower();
   //print to LCD

   lcd.setCursor(0,0);
   lcd.print("Power: ");
   lcd.print(realPower);
   lcd.setCursor(0,1);
   lcd.print("units: ");
   lcd.print(units);
   delay(1000);
   delay(1000);
   currentValueToSend= String(sensorValue);
   is = "radio tx " + currentValueToSend;
   sendmsg(is);
   blinkLED(led, 13, 200);
   delay(3000);
}
