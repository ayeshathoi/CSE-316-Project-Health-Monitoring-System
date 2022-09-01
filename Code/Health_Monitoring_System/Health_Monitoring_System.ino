#include <MAX3010x.h>
#include "MAX30105.h"
#include "filters.h"

#include <SoftwareSerial.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "DHT.h" 
#define DHTTYPE DHT11 
#define DHTPIN 2
DHT dht(DHTPIN, DHTTYPE);

#define BUZZER_PIN 6

#define RXgsm 10
#define TXgsm 11


const uint32_t GSMBaud = 115200;
SoftwareSerial gsmSerial(RXgsm, TXgsm);

MAX30105 sensor;

LiquidCrystal_I2C lcd(0x27, 16, 2);
int val = 0;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

const float kEdgeThreshold = -2000.0;

const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

void setup() {
  
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  gsmSerial.begin(GSMBaud);
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  delay(100);
  
  pinMode(BUZZER_PIN,OUTPUT);

  dht.begin();
  
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Sensor starts");
    
    lcd.setCursor(0,1);
    lcd.print("DHT11 starts");
    delay(2000); 

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Put Index finger");
    lcd.setCursor(0,1);
    lcd.print("On the Sensor");
    delay(2000);
  }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Sensor not found"); 
    delay(2000);
    while(1);
  }

  
}

LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
float kSpO2_A = 2.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 124.6898759;

long last_heartbeat = 0;

long finger_timestamp = 0;
bool finger_detected = false;

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop() {
  
  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.println("Calculating....... ");
   
    }
  }
  else {
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("No Finger detected"); 
    delay(2000);
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);
    
    if(!isnan(current_diff) && !isnan(last_diff)) {
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
      
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          int bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          int average_spo2 = averager_spo2.process(spo2);
          
          if(bpm > 50 && bpm < 140) {
             if(bpm<60 || bpm>125 || spo2<90)
             { 
                Serial.println("BPM " + String(bpm));
                Serial.println("SPO2 " + String(spo2));
                
                printLCD();
                val = 1;
                Buzzer();
                String msg = "Patient's BPM " + String(bpm) ;
                msg += " Spo2 " + String(spo2);
                sendMessage(msg);
                Serial.println("Buzzer Functioning");
             }

           
            if(kEnableAveraging) 
            {
            int average_bpm = averager_bpm.process(bpm);
            int average_r = averager_r.process(r);
            int average_spo2 = averager_spo2.process(spo2);
            float temp = (sensor.readTemperature()*9/5)+32;
            if(averager_bpm.count() >= kSampleThreshold) {

            if(bpm >50 && average_spo2<=100)
            {  
              Patient(bpm,average_spo2,temp);
              delay(2000);
              DHTS();
              delay(2000);
            }

            else
            {
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.println("Calculating....");
              delay(1000);
            }
         }
    }
    else {
         float temp = (sensor.readTemperature()*9/5)+32;

         //human bpm, spo2,temperature show on lcd
         if(bpm >50 && average_spo2<=100)
         {  Patient(bpm,average_spo2,temp);
            delay(2000);
            DHTS();
            delay(2000);
         } 
         else
         {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.println("Calculating.....");
          delay(1000);
         }
    }
  }

    stat_red.reset();
    stat_ir.reset();
  }
      crossed = false;
      last_heartbeat = crossed_time;
    }
  }
  last_diff = current_diff;
  } 
}
void printLCD()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Critical          ");
  lcd.setCursor(0,1);
  lcd.println("Condition         ");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Sending msg    ");
  lcd.setCursor(0,1);
  lcd.println("to Doctor      ");
}
void Patient(float BPM,int spo2,float Temp)
{
  //body temperature,Spo2 and BPM of patient
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("BPM: "+ String(BPM)); 
  lcd.setCursor(0,1);
  lcd.print("BodyTEMP:" + String(Temp,3)+"F"); 
  delay(2000);
                
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sp02 : " + String(spo2)); 
  delay(2000);
}

void Buzzer()
{
  for(int i=0;i<5;i++)
  {
    if(val == 1)
  {
    tone(6,10);
    delay(500);
    noTone(6);
    delay(500);
  }
 }
  val = 0;
}

void sendMessage(String msg)
{
  gsmSerial.println("AT+CMGF=1");
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  gsmSerial.println("AT+CMGS=\"+8801610920878\"\r");
  delay(1000);
  Serial.print(" Sending SMS!");
  gsmSerial.println(msg);
  delay(100);
  gsmSerial.println((char)26);
  delay(1000);
}


void DHTS()
{
  //Room Temperature and Room Humadity DHT11
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = t*9/5+32;
                
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Humidity: "+ String(h,2)+ "%"); 
  lcd.setCursor(0,1);
  lcd.println("Room Temp:"+ String(t,2) + "C ");

  
  if(h<40 || t>36)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Room Condition");
    lcd.setCursor(0,1);
    lcd.print("Not Good");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Sending Msg");
    lcd.setCursor(0,1);
    lcd.print("To doctor");
    delay(2000);
    val = 1;
    Buzzer();
    String msg = "Room Condition Alarming : Humidity " + String(h) ;
    msg += " Room Temp " + String(t);
    sendMessage(msg);
 }
}
