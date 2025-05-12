#include "BluetoothSerial.h"
#include <vector>
#include <string>
#include <iostream>
#include <cstring>
#include <EEPROM.h>
using namespace std;
std::vector<String> charArray;

#define EEPROM_ADDRESS_1 3 // Address to store the variable in EEPROM
#define EEPROM_ADDRESS_2 2
#include <ESP32Encoder.h> 
BluetoothSerial SerialBT;
ESP32Encoder encoder_l;
ESP32Encoder encoder_r;
int motorIn3 = 27;
int motorIn4 = 26;
int en_l = 25;  // left motor enable pin
int motorIn1 = 12;
int motorIn2 = 13;
int en_r = 23; //right motor pins
int pwmChannel_r = 2;  // PWM channel for right motor
int pwmChannel_l = 1;  // PWM channel for left motor

int encoderPin_l_c1 = 34;
int encoderPin_l_c2 = 35;
int encoderPin_r_c1 = 32;
int encoderPin_r_c2 = 33;
int d7 = 2;
int d6 = 4;
int d5 = 16;
int d4 = 17;
int d3 = 5;
int d2 = 18;
int d1 = 19;
int d0 = 21;
int error_l = 0;
int error_r = 0;
int r0;
int r1;
int r2;
int r3;
int r4;
int r5;
int r6;
int r7;
int led_pin=14;
int state_switch=15;
int prv_left=0;
int curr_left;
int prv_right=0;
int curr_right;
float target_l;
float target_r;
int output_l;
int output_r;
int error_m_l;
int error_m_r;
float actual_l;
float actual_r;
int prevTime=0;
int prevTime_1=0;
float integral_error_l = 0;
float integral_error_r = 0;
float derivative_error_l = 0;
float derivative_error_r = 0;
float prev_error_l = 0;
float prev_error_r = 0;
float d_l=0;
float d_r=0;
int kp_m_l = 1;
int kp_m_r = 1;
float ki_m_l=0.1;
float ki_m_r=0.1;
float kd_m_l = 0.1;
float kd_m_r = 0.1;
int kp_l = 1;
int kp_r = 1;
float ki_l=0.01;
float ki_r=0.01;
float kd_l = 0;
float kd_r = 0;
float prev_error_l_l = 0, prev_error_l_r = 0;  // Previous errors for D term
float integral_l_l = 0, integral_l_r = 0;      // Accumulated errors for I term
unsigned long last_time_l = 0;
float derivative_l_l=0;
float derivative_l_r=0;  
int switchstate;
char* optimized;
char cmd;
int i = 0;
int end_flag=0;
int pref_1=22;
int pref;
std::vector<String> deserializeVector(const String& data);

void setup() {
  Serial.begin(9600);           // Initialize serial communication for debugging
  EEPROM.begin(512);
  SerialBT.begin("Tech_fest_new");  // Bluetooth device name
  Serial.println("Bluetooth Device is Ready to Pair");
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  pinMode(led_pin,OUTPUT);
  pinMode(encoderPin_l_c1,INPUT);
  pinMode(encoderPin_r_c1,INPUT);
  pinMode(encoderPin_l_c2,INPUT);
  pinMode(encoderPin_r_c2,INPUT);
  pinMode(state_switch,INPUT);
  pinMode(d0, INPUT);
  pinMode(d1, INPUT);
  pinMode(d2, INPUT);
  pinMode(d3, INPUT);
  pinMode(d4, INPUT);
  pinMode(d5, INPUT);
  pinMode(d6, INPUT);
  pinMode(d7, INPUT);
  pinMode(pref,INPUT);
  // Initialize PWM channels
  ledcSetup(pwmChannel_r, 1000, 8);  // 1000 Hz frequency, 8-bit resolution
  ledcSetup(pwmChannel_l, 1000, 8);

  // Attach PWM channels to enable pins
  ledcAttachPin(en_r, pwmChannel_r);
  ledcAttachPin(en_l, pwmChannel_l);

  ledcWrite(pwmChannel_l, 128 );
  ledcWrite(pwmChannel_r, 128 );
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, LOW);
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder_l.attachFullQuad(encoderPin_l_c1, encoderPin_l_c2);  // Attach encoder pins (Pin A: GPIO 5, Pin B: GPIO 4)
  encoder_l.clearCount();          // Reset encoder count
  encoder_r.attachFullQuad(encoderPin_r_c1, encoderPin_r_c2);  // Attach encoder pins (Pin A: GPIO 5, Pin B: GPIO 4)
  encoder_r.clearCount();          // Reset encoder count
   
}
void loop(){
  pref = digitalRead(pref_1);
  switchstate = digitalRead(state_switch);
  //SerialBT.println(EEPROM.read(EEPROM_ADDRESS_2));
  
  if(switchstate==1){
    if(EEPROM.read(EEPROM_ADDRESS_2) == 1 && pref==0)
    {
    
    for (int i = 0; i < charArray.size(); i++) {
      SerialBT.println(charArray[i]);
    }
    // Deserialize the String back into a vector
    if (EEPROM.read(EEPROM_ADDRESS_2) == 1 && switchstate == 1) {
        String storedData = EEPROM.readString(EEPROM_ADDRESS_1);
        std::vector<String> C_array = deserializeVector(storedData);
        optimized= optimizePath_l(convertVectorToCharArray(C_array));
        removeChar(optimized,'B');
        SerialBT.println(optimized);
    
    SerialBT.println("left_main");
    }
    while(true){
      switchstate = digitalRead(state_switch);
      i = 0;
      //SerialBT.print(i);
      while(switchstate == 1){
        switchstate = digitalRead(state_switch);
        digitalWrite(led_pin,LOW);
        r0 = !digitalRead(d0);
        r1 = !digitalRead(d1);
        r2 = !digitalRead(d2);
        r3 = !digitalRead(d3);
        r4 = !digitalRead(d4);
        r5 = !digitalRead(d5);
        r6 = !digitalRead(d6);
        r7 = !digitalRead(d7);
        error_l = 20 * r4 + 35 * r5 + 45 * r6 + 50 * r7;
        error_r = 50 * r0 + 45 * r1 + 35 * r2 + 20 * r3;

        ledcWrite(pwmChannel_l, constrain(190 + kp_l * error_l -kp_r * error_r,0,255));
        ledcWrite(pwmChannel_r, constrain(190 + kp_r * error_r -kp_l * error_l,0,255));  // 200/255 is about 78% duty cycle


        // Set motor direction
        digitalWrite(motorIn1, HIGH);
        digitalWrite(motorIn2, LOW);  // Right motor forward
        digitalWrite(motorIn3, HIGH);
        digitalWrite(motorIn4, LOW);
        if((r0 == 1 && r1 == 1 && r2 == 1 && r3==1)  || ((r0 + r1 + r2 + r3 + r4 + r5 + r7) >= 6) || (r4==1 && r5==1 && r6==1 && r7==1) && (r0==0 && r1==0)){
          cmd = optimized[i];
          ++i;
          if(cmd=='L'){
            left_1();
          }
          if(cmd=='R'){
            right_1();
          }
          if(cmd=='S'){
              SerialBT.println("Straight_mr");
              ledcWrite(pwmChannel_l, 120);
              ledcWrite(pwmChannel_r, 120);
              digitalWrite(motorIn1, HIGH);
              digitalWrite(motorIn2, LOW);
              digitalWrite(motorIn3, HIGH);
              digitalWrite(motorIn4, LOW);
              delay(100);
          }
          if(i==strlen(optimized)+1){
            while(switchstate == 1){
                switchstate = digitalRead(state_switch);
                digitalWrite(led_pin,HIGH);
                digitalWrite(motorIn1, LOW);
                digitalWrite(motorIn2, LOW);  // Right motor forward
                digitalWrite(motorIn3, LOW);
                digitalWrite(motorIn4, LOW);}
          }
        }

      }
    }

  
    }
    if(EEPROM.read(EEPROM_ADDRESS_2) == 1 && pref==1){

    for (int i = 0; i < charArray.size(); i++)
     {
      SerialBT.println(charArray[i]);
    }
   if (EEPROM.read(EEPROM_ADDRESS_2) == 1 && switchstate == 1) {
        String storedData = EEPROM.readString(EEPROM_ADDRESS_1);
        std::vector<String> C_array = deserializeVector(storedData);

        optimized= optimizePath_R(convertVectorToCharArray(C_array));
        removeChar(optimized,'B');
        SerialBT.println(optimized);
    // optimized= optimizePath_l(convertVectorToCharArray(charArray));
        SerialBT.println("right_main");
   }
    while(true){
      switchstate = digitalRead(state_switch);
      i = 0;
      //SerialBT.print(i);
      while(switchstate == 1){
        switchstate = digitalRead(state_switch);
        digitalWrite(led_pin,LOW);
        r0 = !digitalRead(d0);
        r1 = !digitalRead(d1);
        r2 = !digitalRead(d2);
        r3 = !digitalRead(d3);
        r4 = !digitalRead(d4);
        r5 = !digitalRead(d5);
        r6 = !digitalRead(d6);
        r7 = !digitalRead(d7);
        error_l = 20 * r4 + 35 * r5 + 45 * r6 + 50 * r7;
        error_r = 50 * r0 + 45 * r1 + 35 * r2 + 20 * r3;

        ledcWrite(pwmChannel_l, constrain(190 + kp_l * error_l -kp_r * error_r,0,255));
        ledcWrite(pwmChannel_r, constrain(190 + kp_r * error_r -kp_l * error_l,0,255));  // 200/255 is about 78% duty cycle


        // Set motor direction
        digitalWrite(motorIn1, HIGH);
        digitalWrite(motorIn2, LOW);  // Right motor forward
        digitalWrite(motorIn3, HIGH);
        digitalWrite(motorIn4, LOW);
        if((r0+r1+r2+r3+r4+r5+r6+r7)>=4){
          cmd = optimized[i];
          ++i;
          if(cmd=='L'){
            left_1();
          }
          if(cmd=='R'){
            right_1();
          }
          if(cmd=='S'){
              SerialBT.println("Straight_mr");
              ledcWrite(pwmChannel_l, 140);
              ledcWrite(pwmChannel_r, 140);
              digitalWrite(motorIn1, HIGH);
              digitalWrite(motorIn2, LOW);
              digitalWrite(motorIn3, HIGH);
              digitalWrite(motorIn4, LOW);
              delay(120);
          }
          if(i==strlen(optimized)+1){
            while(switchstate == 1){
                switchstate = digitalRead(state_switch);
                digitalWrite(led_pin,HIGH);
                digitalWrite(motorIn1, LOW);
                digitalWrite(motorIn2, LOW);  // Right motor forward
                digitalWrite(motorIn3, LOW);
                digitalWrite(motorIn4, LOW);}
          }
        }

      }
    }

  
    }

  }
  if(switchstate == 0){
    if(pref==0 && end_flag==0){


  r0 = !digitalRead(d0);
  r1 = !digitalRead(d1);
  r2 = !digitalRead(d2);
  r3 = !digitalRead(d3);
  r4 = !digitalRead(d4);
  r5 = !digitalRead(d5);
  r6 = !digitalRead(d6);
  r7 = !digitalRead(d7);
                                 


  error_l = 20 * r4 + 30 * r5 + 40 * r6 + 50 * r7;
  error_r = 50 * r0 + 40 * r1 + 30 * r2 + 20 * r3;

  // unsigned long current_time = millis();
  // if((current_time-last_time_l)>100){
  // float delta_time = (current_time - last_time_l) ;
  // last_time_l=current_time;
  // last_time_l = current_time;
  // derivative_l_l = (error_l - prev_error_l_l) / delta_time;
  // derivative_l_r = (error_r - prev_error_l_r) / delta_time;
  // d_l = kd_l * derivative_l_l;
  // d_r = kd_r * derivative_l_r;
  // prev_error_l_l = error_l;
  // prev_error_l_r = error_r;
  // }
  float p_l = kp_l * error_l-kp_r * error_r;
  float p_r = kp_r * error_r-kp_l * error_l;

  // Calculate target speeds using PID terms
  float target_l = 180 + p_l;// + d_l;
  float target_r = 180 + p_r;// + d_r;

  // Ensure motor speeds are within range (e.g., 0-255)
  target_l = constrain(target_l, 0, 255);
  target_r = constrain(target_r, 0, 255);

  // Write PWM signals to motors
  ledcWrite(pwmChannel_l, target_l);
  ledcWrite(pwmChannel_r, target_r);

  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  if ((r0 == 1 && r1 == 1 && r2 == 1 && r3==1)  || ((r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7) >= 8)){

    left_L();
  }
  if((r4 == 1 && r5==1 && r6==1 && r7==1) && (r0==0)){
    delay(30);
    r0 = !digitalRead(d0);
    r1 = !digitalRead(d1);
    r2 = !digitalRead(d2);
    r3 = !digitalRead(d3);
    r4 = !digitalRead(d4);
    r5 = !digitalRead(d5);
    r6 = !digitalRead(d6);
    r7 = !digitalRead(d7);
  if ((r0 == 1 && r1 == 1 && r2 == 1 && r3==1)  || ((r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7) >= 8)){
    left_L();
  }
  else{
    right_L();
  }}
  if (r0 + r1 + r2 + r3 + r4 + r5 + r7 == 0){
    uturn_R();
  }
  }
  if(pref==1 && end_flag==0){
    r0 = !digitalRead(d0);
  r1 = !digitalRead(d1);
  r2 = !digitalRead(d2);
  r3 = !digitalRead(d3);
  r4 = !digitalRead(d4);
  r5 = !digitalRead(d5);
  r6 = !digitalRead(d6);
  r7 = !digitalRead(d7);
  error_l = 20 * r4 + 30 * r5 + 40 * r6 + 50 * r7;
    error_r = 50 * r0 + 40 * r1 + 30 * r2 + 20 * r3;
    float p_l = kp_l * error_l-kp_r * error_r;
    float p_r = kp_r * error_r-kp_l * error_l;
    float target_l = 180 + p_l ;//+ d_l;
    float target_r = 180 + p_r ;//+ d_r;
    target_l = constrain(target_l, 0, 255);
    target_r = constrain(target_r, 0, 255);
    ledcWrite(pwmChannel_l, target_l);
    ledcWrite(pwmChannel_r, target_r);
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
  if ((r4 == 1 && r5 == 1 && r6 == 1 && r7==1)  || ((r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7) >= 8)){

    right_R();
  }
  if((r0 == 1 && r1==1 && r2==1 && r3==1) && ( r7==0)){
    delay(30);
    r0 = !digitalRead(d0);
    r1 = !digitalRead(d1);
    r2 = !digitalRead(d2);
    r3 = !digitalRead(d3);
    r4 = !digitalRead(d4);
    r5 = !digitalRead(d5);
    r6 = !digitalRead(d6);
    r7 = !digitalRead(d7);
    
  if ((r4 == 1 && r5 == 1 && r6 == 1 && r7==1)  || ((r0 + r1 + r2 + r3 + r4 + r5 + r6 + r7) >= 8)){
    right_R();
  }
  else{
    left_R();
  }}
  if (r0 + r1 + r2 + r3 + r4 + r5 + r7 == 0){
    uturn_r_new();
  }    

  }


  }
  }
void left_L(){
  ledcWrite(pwmChannel_l,110);
  ledcWrite(pwmChannel_r,110); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // OVERSHOOT forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(200);


  //checking for end
  r0 = !digitalRead(d0);
  r1 = !digitalRead(d1);
  r2 = !digitalRead(d2);
  r3 = !digitalRead(d3);
  r4 = !digitalRead(d4);
  r5 = !digitalRead(d5);
  r6 = !digitalRead(d6);
  r7 = !digitalRead(d7);
  if((r0 + r1 + r2 + r3 + r4 + r5 + r6+ r7) >= 4){   

    end_flag=1;
    
    digitalWrite(led_pin,HIGH);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, LOW);
    String serializedData = serializeVector(charArray);

    // Write the serialized String to EEPROM
    EEPROM.writeString(EEPROM_ADDRESS_1, serializedData);
    EEPROM.commit();
    delay(100);
    // Read back the String from EEPROM

    
    EEPROM.write(EEPROM_ADDRESS_2, end_flag);
    EEPROM.commit();               //Writing end flag in EEPROM
    delay(100);
    SerialBT.println(EEPROM.read(EEPROM_ADDRESS_2));
    return;       //end reached
  }
  SerialBT.println("Left");
  charArray.push_back("L");
  ledcWrite(pwmChannel_l,190); //170
  ledcWrite(pwmChannel_r,170); //150
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  delay(220); // Coming out of straight line 200

  int dec = 0;
  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  target_l=370;
  target_r=340;
  encoder_l.clearCount();
  encoder_r.clearCount();
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
      prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,85,255));
    ledcWrite(pwmChannel_r,constrain(output_r,85,255));
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, HIGH);
    dec = dec + 1;
    delay(4); //USED TO BE 4
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
return;
}

void right_L(){
  ledcWrite(pwmChannel_l, 110);
  ledcWrite(pwmChannel_r, 110);
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(100);  // Checking straight/end
  r0 = !digitalRead(d0);
  r1 = !digitalRead(d1);
  r2 = !digitalRead(d2);
  r3 = !digitalRead(d3);
  r4 = !digitalRead(d4);
  r5 = !digitalRead(d5);
  r6 = !digitalRead(d6);
  r7 = !digitalRead(d7);
  //checking for end
  if((r0+r1+r2+r3+r4+r5+r6+r7)>=4){          //end found
  end_flag=1;
  digitalWrite(led_pin,HIGH);
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, LOW);
  String serializedData = serializeVector(charArray);

    // Write the serialized String to EEPROM
    EEPROM.writeString(EEPROM_ADDRESS_1, serializedData);
    EEPROM.commit();
    delay(100);
    // Read back the String from EEPROM

    
    EEPROM.write(EEPROM_ADDRESS_2, end_flag);
    EEPROM.commit();               //Writing end flag in EEPROM
    delay(100);
    SerialBT.println(EEPROM.read(EEPROM_ADDRESS_2));
    return;       //end reached
  }
    
  if (r2 + r3 + r4 + r1 + r5 +r6 >= 1) {
    SerialBT.println("Straight");
    charArray.push_back("S");
    ledcWrite(pwmChannel_l, 110);
    ledcWrite(pwmChannel_r, 110);
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    delay(20);
    return;
}
  SerialBT.println("Right");
  charArray.push_back("R");
  ledcWrite(pwmChannel_l,110);
  ledcWrite(pwmChannel_r,110); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);  

  delay(110);
  ledcWrite(pwmChannel_l,170); //170
  ledcWrite(pwmChannel_r,190); //150
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(150); // Coming out of straight line 200

  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  // USED TO BE 350 350
  target_l=330; 
  target_r=360; 
  encoder_l.clearCount();
  encoder_r.clearCount();
  int dec=0;
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
        prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,110,255));
    ledcWrite(pwmChannel_r,constrain(output_r,110,255));
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);  // Right motor forward
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    dec = dec + 1;
    delay(10); //USED BE BE 13
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
return;
}
void uturn_R(){
  SerialBT.println("Uturn");
  charArray.push_back("B");
  ledcWrite(pwmChannel_l,120);
  ledcWrite(pwmChannel_r,120); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(200);
  ledcWrite(pwmChannel_l,190);
  ledcWrite(pwmChannel_r,250);
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(200); // Used to be 180

  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  target_l=265;
  target_r=360;
  encoder_l.clearCount();
  encoder_r.clearCount();
  int dec=0;
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
        prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,110,255));
    ledcWrite(pwmChannel_r,constrain(output_r,130,255));
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);  // Right motor forward
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    dec = dec + 1;
    delay(32);
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
  return;
    }
void right_R(){
  ledcWrite(pwmChannel_l,110);
  ledcWrite(pwmChannel_r,110); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(190); //from 200


  //checking for end
  r0 = !digitalRead(d0);
  r1 = !digitalRead(d1);
  r2 = !digitalRead(d2);
  r3 = !digitalRead(d3);
  r4 = !digitalRead(d4);
  r5 = !digitalRead(d5);
  r6 = !digitalRead(d6);
  r7 = !digitalRead(d7);
  if((r0 + r1 + r2 + r3 + r4 + r5 + r6+ r7) >= 4){           //end reached
    end_flag=1;
    digitalWrite(led_pin,HIGH);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, LOW);
      String serializedData = serializeVector(charArray);

    // Write the serialized String to EEPROM
    EEPROM.writeString(EEPROM_ADDRESS_1, serializedData);
    EEPROM.commit();
    delay(100);
    // Read back the String from EEPROM

    
    EEPROM.write(EEPROM_ADDRESS_2, end_flag);
    EEPROM.commit();               //Writing end flag in EEPROM
    delay(100);
    SerialBT.println(EEPROM.read(EEPROM_ADDRESS_2));
    return;       //end reached
  }
  SerialBT.println("Right");
  charArray.push_back("R");
  ledcWrite(pwmChannel_l,190);//180
  ledcWrite(pwmChannel_r,240); 
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(240);  // Coming out of straight line
  // was 180
  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  // USED TO BE 350 350
  target_l=310; 
  target_r=360; 
  encoder_l.clearCount();
  encoder_r.clearCount();
  int dec=0;
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
        prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,100,255));
    ledcWrite(pwmChannel_r,constrain(output_r,120,255)); //changed, was 100
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);  // Right motor forward
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    dec = dec + 1;
    delay(3); //USED BE BE 4
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
return;
}
void left_R(){
  ledcWrite(pwmChannel_l, 110);
  ledcWrite(pwmChannel_r, 110);
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(100);  // Checking straight/end
  r0 = !digitalRead(d0);
  r1 = !digitalRead(d1);
  r2 = !digitalRead(d2);
  r3 = !digitalRead(d3);
  r4 = !digitalRead(d4);
  r5 = !digitalRead(d5);
  r6 = !digitalRead(d6);
  r7 = !digitalRead(d7);
  //checking for end
  if((r0+r1+r2+r3+r4+r5+r6+r7)>=4){ 
    end_flag=1;
    digitalWrite(led_pin,HIGH);
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, LOW);
      String serializedData = serializeVector(charArray);

    // Write the serialized String to EEPROM
    EEPROM.writeString(EEPROM_ADDRESS_1, serializedData);
    EEPROM.commit();
    delay(100);
    // Read back the String from EEPROM

    
    EEPROM.write(EEPROM_ADDRESS_2, end_flag);
    EEPROM.commit();               //Writing end flag in EEPROM
    delay(100);
    SerialBT.println(EEPROM.read(EEPROM_ADDRESS_2));
    return;       //end reached
    }        //end found
    
    
  if (r1 + r2 + r3 + r4 + r5 +r6  >= 1) {
    SerialBT.println("Straight");
    charArray.push_back("S");
    ledcWrite(pwmChannel_l, 110);
    ledcWrite(pwmChannel_r, 110);
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    delay(20);
    return;
}
  SerialBT.println("Left");
  charArray.push_back("L");
  ledcWrite(pwmChannel_l,110);
  ledcWrite(pwmChannel_r,110); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);  

  delay(70);
  ledcWrite(pwmChannel_l,230);//220
  ledcWrite(pwmChannel_r,170); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  delay(150); // Coming out of straight line
    //   digitalWrite(motorIn1, LOW);
    // digitalWrite(motorIn2, LOW);  // Right motor forward
    // digitalWrite(motorIn3, LOW);
    // digitalWrite(motorIn4, LOW);
    // delay(20000);
  int dec = 0;
  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  target_l=320; //used to be 370,330
  target_r=260;
  encoder_l.clearCount();
  encoder_r.clearCount();
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
      prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,100,255));
    ledcWrite(pwmChannel_r,constrain(output_r,90,255));
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, HIGH);
    dec = dec + 1;
    delay(4); //USED TO BE 4
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
return;
}

void left_1(){
  SerialBT.println("left_mr");
  ledcWrite(pwmChannel_l,110);//move few inch
  ledcWrite(pwmChannel_r,110); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(250);
  ledcWrite(pwmChannel_l,185);
  ledcWrite(pwmChannel_r,185); 
  digitalWrite(motorIn1, HIGH);//get out of line
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  delay(200);
  int dec = 0;
  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  target_l=350;
  target_r=320;
  encoder_l.clearCount();
  encoder_r.clearCount();
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
      prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,70,255));
    ledcWrite(pwmChannel_r,constrain(output_r,70,255));
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, HIGH);
    dec = dec + 1;
    delay(4); //USED TO BE 4
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
return;
}

void right_1(){
  SerialBT.println("right_mr");
  ledcWrite(pwmChannel_l,110);
  ledcWrite(pwmChannel_r,110); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(180);
  ledcWrite(pwmChannel_l,185);
  ledcWrite(pwmChannel_r,185); 
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(200);//coming out of straight line
  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  // USED TO BE 350 350
  target_l=310; 
  target_r=340; 
  encoder_l.clearCount();
  encoder_r.clearCount();
  int dec=0;
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
        prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,70,255));
    ledcWrite(pwmChannel_r,constrain(output_r,70,255));
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);  // Right motor forward
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);
    dec = dec + 1;
    delay(11); //USED BE BE 10
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
return;
  
}
void uturn_L(){
  SerialBT.println("Uturn");
  charArray.push_back("B");
  ledcWrite(pwmChannel_l,120 );
  ledcWrite(pwmChannel_r,120); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(250);   // Move few inches front
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, LOW);
  digitalWrite(motorIn4, HIGH);
  delay(220);
  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  target_l=380;
  target_r=285;
  encoder_l.clearCount();
  encoder_r.clearCount();
  int dec=0;
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
    if (elapsedTime >= 4){
        prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,100,255));
    ledcWrite(pwmChannel_r,constrain(output_r,100,255));
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);  // Right motor forward
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, HIGH);
    dec = dec + 1;
    delay(5);
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
  return;
    }

char* convertVectorToCharArray(const std::vector<String>& vec) {
  static char path[100];  // Adjust the size as needed
  int index = 0;

  for (int i = 0; i < vec.size(); ++i) {
    // Copy each String element into the char array
    strcpy(&path[index], vec[i].c_str());
    index += vec[i].length();
  }
  path[index] = '\0';  // Null-terminate the array
  return path;
}

char* optimizePath_R(char path[]) {
  int pathIndex = strlen(path);  // Get the size of the path excluding null terminator
  bool isOptimized = false;      // Flag to track if any optimization happened

  char* optimizedPath = new char[100];  // Dynamically allocate memory for the optimized path

  while (!isOptimized) {
    int optimizedIndex = 0;
    isOptimized = true;  // Assume this pass will finish optimization

    for (int i = 0; i < pathIndex; i++) {
      // Look ahead by two steps for specific patterns to optimize
      if (i < pathIndex - 2) {
        // Check for LBR → B
        if (path[i] == 'R' && path[i + 1] == 'B' && path[i + 2] == 'L') {
          optimizedPath[optimizedIndex++] = 'B';
          i += 2;               // Skip the next two steps
          isOptimized = false;  // An optimization was made, keep looping
          continue;
        }
        // Check for LBS → R
        else if (path[i] == 'R' && path[i + 1] == 'B' && path[i + 2] == 'S') {
          optimizedPath[optimizedIndex++] = 'L';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for RBL → B
        else if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'R') {
          optimizedPath[optimizedIndex++] = 'B';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for SBL → R
        else if (path[i] == 'S' && path[i + 1] == 'B' && path[i + 2] == 'R') {
          optimizedPath[optimizedIndex++] = 'L';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for SBS → B
        else if (path[i] == 'S' && path[i + 1] == 'B' && path[i + 2] == 'S') {
          optimizedPath[optimizedIndex++] = 'B';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for LBL → S
        else if (path[i] == 'R' && path[i + 1] == 'B' && path[i + 2] == 'R') {
          optimizedPath[optimizedIndex++] = 'S';
          i += 2;
          isOptimized = false;
          continue;
        }
      }

      // Default action: log the movement
      optimizedPath[optimizedIndex++] = path[i];
    }

    optimizedPath[optimizedIndex] = '\0';  // Null-terminate the optimized path

    // If the optimized path length is the same as the original, break the loop
    if (optimizedIndex == pathIndex) {
      break;
    }

    // Update the original path and length for the next iteration
    strcpy(path, optimizedPath);
    pathIndex = optimizedIndex;
  }

  // Return the optimized path
  return optimizedPath;
}

char* optimizePath_l(char path[]) {
  int pathIndex = strlen(path);  // Get the size of the path excluding null terminator
  bool isOptimized = false;      // Flag to track if any optimization happened

  char* optimizedPath = new char[100];  // Dynamically allocate memory for the optimized path

  while (!isOptimized) {
    int optimizedIndex = 0;
    isOptimized = true;  // Assume this pass will finish optimization

    for (int i = 0; i < pathIndex; i++) {
      // Look ahead by two steps for specific patterns to optimize
      if (i < pathIndex - 2) {
        // Check for LBR → B
        if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'R') {
          optimizedPath[optimizedIndex++] = 'B';
          i += 2;               // Skip the next two steps
          isOptimized = false;  // An optimization was made, keep looping
          continue;
        }
        // Check for LBS → R
        else if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'S') {
          optimizedPath[optimizedIndex++] = 'R';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for RBL → B
        else if (path[i] == 'R' && path[i + 1] == 'B' && path[i + 2] == 'L') {
          optimizedPath[optimizedIndex++] = 'B';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for SBL → R
        else if (path[i] == 'S' && path[i + 1] == 'B' && path[i + 2] == 'L') {
          optimizedPath[optimizedIndex++] = 'R';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for SBS → B
        else if (path[i] == 'S' && path[i + 1] == 'B' && path[i + 2] == 'S') {
          optimizedPath[optimizedIndex++] = 'B';
          i += 2;
          isOptimized = false;
          continue;
        }
        // Check for LBL → S
        else if (path[i] == 'L' && path[i + 1] == 'B' && path[i + 2] == 'L') {
          optimizedPath[optimizedIndex++] = 'S';
          i += 2;
          isOptimized = false;
          continue;
        }
      }

      // Default action: log the movement
      optimizedPath[optimizedIndex++] = path[i];
    }

    optimizedPath[optimizedIndex] = '\0';  // Null-terminate the optimized path

    // If the optimized path length is the same as the original, break the loop
    if (optimizedIndex == pathIndex) {
      break;
    }

    // Update the original path and length for the next iteration
    strcpy(path, optimizedPath);
    pathIndex = optimizedIndex;
  }

  // Return the optimized path
  return optimizedPath;
}


void removeChar(char* str, char charToRemove) {
    int readIndex = 0, writeIndex = 0;

    // Traverse the array
    while (str[readIndex] != '\0') {
        // Copy characters that are not charToRemove
        if (str[readIndex] != charToRemove) {
            str[writeIndex] = str[readIndex];
            writeIndex++;
        }
        readIndex++;
    }
    
    // Null-terminate the modified string
    str[writeIndex] = '\0';
}

String serializeVector(const std::vector<String>& vec) {
    String serialized = "";
    for (size_t i = 0; i < vec.size(); i++) {
        serialized += vec[i];
        if (i < vec.size() - 1) {
            serialized += ","; // Use a delimiter
        }
    }
    return serialized;
}

// Function to deserialize a String back into a vector of Strings
std::vector<String> deserializeVector(const String& data) {
    std::vector<String> vec;
    int start = 0;
    int end = data.indexOf(',');

    while (end != -1) {
        vec.push_back(data.substring(start, end));
        start = end + 1;
        end = data.indexOf(',', start);
    }

    // Add the last element
    vec.push_back(data.substring(start));
    return vec;
}

void uturn_r_new(){

  SerialBT.println("Uturn");
  charArray.push_back("B");
  ledcWrite(pwmChannel_l,120);
  ledcWrite(pwmChannel_r,120); 
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(200);
  ledcWrite(pwmChannel_l,190);
  ledcWrite(pwmChannel_r,250);
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, HIGH);  // Right motor forward
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);
  delay(200); // Used to be 180

  int left_max=384;
  int right_max=360;
  r3=0;
  r4=0;
  target_l=265;
  target_r=360;
  encoder_l.clearCount();
  encoder_r.clearCount();
  int dec=0;
  while(r3+r4==0){
    unsigned long elapsedTime = millis() - prevTime;
     if (elapsedTime >= 4){
        prevTime=millis();
        long currentCount_l = abs(encoder_l.getCount());
        long currentCount_r= abs(encoder_r.getCount());
        curr_left = currentCount_l;
        curr_right = currentCount_r;
        actual_l=(curr_left-prv_left)*60000/(elapsedTime*840);
        actual_r=(curr_right-prv_right)*60000/(elapsedTime*840);
        prv_left=curr_left;
        prv_right=curr_right;
        error_m_l=target_l-actual_l;
        error_m_r=target_r-actual_r;
        integral_error_l += error_m_l * elapsedTime / 1000.0;  // Convert ms to seconds
        integral_error_r += error_m_r * elapsedTime / 1000.0;
        integral_error_l = constrain(integral_error_l, -500, 500);
        integral_error_r = constrain(integral_error_r, -500, 500);
        output_l= (kp_m_l*error_m_l+ki_m_l*integral_error_l)*255/384;
        output_r= (kp_m_r*error_m_r+ki_m_r*integral_error_r)*255/360;


    }
    target_l=target_l-dec;
    target_r=target_r-dec;
    ledcWrite(pwmChannel_l,constrain(output_l,110,255));
    ledcWrite(pwmChannel_r,constrain(output_r,130,255));
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);  // Right motor forward
    digitalWrite(motorIn3, HIGH);
digitalWrite(motorIn4, LOW);
    dec = dec + 1;
    delay(23);
    r3= !digitalRead(d3);
    r4= !digitalRead(d4);



  }
  return;


}
