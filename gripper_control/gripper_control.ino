////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
#include "AS5600.h"
AS5600 as5600_0(&Wire);
int enc_pos, L=48;
float target_size=123.45, actual_size=679.90, theta; // mm, rad
const float dist_const=34.76,theta_offset=0;
////////////////////////////////////////////////////////////// Encoder //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
float KP_pos=-0.350;
float KI_pos=-0.000150;
float KD_pos=-1.8;

float target_pos=0,previous_target_pos, err_sum_pos,pre_err; 
float PI_pos_output;
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Inputs //////////////////////////////////////////////////////////////
// Buttons
#define KEY1 33
#define KEY2 25
#define KEY3 26
#define KEY4 27
#define KEY5 14

#define enc_1A_pin 39
#define enc_1B_pin 34
#define enc_2A_pin 35
#define enc_2B_pin 32

int count_1=0; // Left Encoder
int count_2=0; // Right Encoder
////////////////////////////////////////////////////////////// Inputs //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////
// Gripper
#define M1_pin 19 // PWMA 
#define M1_PWM_CH 0
#define M1_PWM_res 12 
#define M1_PWM_freq 10000

// Pump
#define M2_pin 4 // PWMB
#define M2_PWM_CH 1
#define M2_PWM_res 12
#define M2_PWM_freq 10000

#define AIN1 5
#define AIN2 18
#define BIN1 0
#define BIN2 15

float M1_speed=0, M2_speed=0; // 0~+-4096

#define S1_pin 23
#define S2_pin 18

const int S1_soft=1000, S1_hard=2000;
const int S2_soft=1000, S2_hard=2000;
int S1_angle, S2_angle;

bool pump_status=0, mode=2, hardness=0;
String hardness_str="Hard", pump_status_str="OFF";
int gripper_status=0; // 0=stop, -1=open, 1=close
////////////////////////////////////////////////////////////// MOTOR //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// OLED //////////////////////////////////////////////////////////////
// #include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

// create task on core 1
TaskHandle_t display_update_task;
////////////////////////////////////////////////////////////// OLED //////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(460800);
  
  input_setup(); // encoder, button

  OLED_setup(); // I2C, screen
  // AS5600_setup(); // I2C, encoder

  motor_output_setup();

  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  digitalWrite(2,HIGH); delay(100); digitalWrite(2,LOW); delay(100);
  Serial.print("Set up ends");
  display_UI();
}

void loop() {
  loop_time_holder();
  control();
  motor_output_loop();
  // Serial.print(count_1); Serial.print("\t");
  // Serial.print(count_2); Serial.print("\t");
  Serial.print("Loop running on core ");
  Serial.print(xPortGetCoreID());
  Serial.println();
  taskYIELD();
}

////////////////////////////////////////////// TIME //////////////////////////////////////////////
unsigned long loop_start_time=0;
unsigned long loop2_start_time=0;
const int loop_time=10000; // micro seconds
void loop_time_holder(){
  vTaskDelay(5);
  
  if(micros() - loop_start_time > (loop_time+50)) digitalWrite(2, HIGH);
  else digitalWrite(2, LOW);

  unsigned long while_start_time=micros();
  while ((micros()- loop_start_time)<loop_time){} 
  // delayMicroseconds(loop_time-(while_start_time-loop_start_time));
  int dt=micros()-loop_start_time;
  // unsigned long while_end_time=micros();
  // Serial.print("while time:");Serial.print(while_end_time-while_start_time);Serial.print("  ");
  Serial.print("dt:");Serial.print(dt);Serial.print(" on core ");
  Serial.print(xPortGetCoreID());Serial.print("  ");
  loop_start_time = micros(); 
}
////////////////////////////////////////////// TIME //////////////////////////////////////////////
////////////////////////////////////////////////////////////// Inputs //////////////////////////////////////////////////////////////
void input_setup(){
  pinMode(enc_1A_pin, INPUT);
  pinMode(enc_1B_pin, INPUT);
  pinMode(enc_2A_pin, INPUT);
  pinMode(enc_2B_pin, INPUT);
  attachInterrupt(enc_1A_pin,enc_1_A,CHANGE);
  attachInterrupt(enc_1B_pin,enc_1_B,CHANGE);
  attachInterrupt(enc_2A_pin,enc_2_A,CHANGE);
  attachInterrupt(enc_2B_pin,enc_2_B,CHANGE);

  pinMode(KEY1, INPUT);
  pinMode(KEY2, INPUT);
  pinMode(KEY3, INPUT);
  pinMode(KEY4, INPUT);
  pinMode(KEY5, INPUT);
  attachInterrupt(KEY1,fc1,FALLING);
  attachInterrupt(KEY2,fc2,FALLING);
  attachInterrupt(KEY3,fc3,FALLING);
  attachInterrupt(KEY4,fc4,FALLING);
  attachInterrupt(KEY5,fc5,FALLING);
}

void enc_1_A(){
  if (digitalRead(enc_1A_pin) == digitalRead(enc_1B_pin)) count_1--;  
  else count_1++;
  // Serial.print("M1 count: ");
  // Serial.println(count_1);
}

void enc_1_B(){
  if (digitalRead(enc_1A_pin) == digitalRead(enc_1B_pin)) count_1++;  
  else count_1--;
  // Serial.print("M1 count: ");
  // Serial.println(count_1);
}

void enc_2_A(){
  if (digitalRead(enc_2A_pin) == digitalRead(enc_2B_pin)) count_2--;
  else count_2++;
  // Serial.print("M2 count: ");
  // Serial.println(count_2);
}

void enc_2_B(){
  if (digitalRead(enc_2A_pin) == digitalRead(enc_2B_pin)) count_2++;
  else count_2--;
  // Serial.print("M2 count: ");
  // Serial.println(count_2);
}

unsigned long mode_change_time=0;
void fc1(){
  if(!digitalRead(KEY1))
  Serial.println("Key 1 pressed");
}
void fc2(){
  if(!digitalRead(KEY2))
  Serial.println("Key 2 pressed");
}
void fc3(){
  // if (!mode){ 

  // }
  // gripper_status++;
  // if (gripper_status==2) gripper_status=-1;
  if(!digitalRead(KEY3))
  Serial.println("Key 3 pressed");
}
void fc4(){
  if(!digitalRead(KEY4))
  Serial.println("Key 4 pressed");
}
void fc5(){
  hardness=!hardness;
  if(!digitalRead(KEY5))
  Serial.println("Key 5 pressed");
}

////////////////////////////////////////////////////////////// Inputs //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
void control(){
  // Serial.print(0); // To freeze the lower limit
  // Serial.print(" ");

  // Serial.print(4100); // To freeze the upper limit
  // Serial.print(" ");

  theta=acos((target_size-dist_const)/2/L);
  int target_5600_reading=int(theta*4096/2/M_PI+theta_offset);

  // actual_size=(as5600_0.readAngle()-theta_offset)*M_PI*2/4096;

  // target_size=target_size+float(count_1*10/2+count_2);
  count_1=0, count_2=0;
  // target_size=constrain(target_size,0,120);
  // Serial.print("target_size:");Serial.print(target_size); Serial.print(" ");
  // position_PI(target_5600_reading,as5600_0.readAngle());
  // position_PI(target_size,actual_size);

  if (!digitalRead(KEY1)){
    M1_speed=0;
    M2_speed=2000;
    digitalWrite(17,HIGH);
    digitalWrite(2,HIGH);
    
  } else if (!digitalRead(KEY2)){
    M1_speed=-2000;
    M2_speed=0;
  } else if (!digitalRead(KEY3)){
    M1_speed=2000;
    M2_speed=0;
  } else {
    M1_speed=0;
    M2_speed=0;
    digitalWrite(17,LOW);
    digitalWrite(2,LOW);
  }

}

void position_PI(float target, float actual){
  // float speed=v1*(1-alpha_speed)+previous_speed*alpha_speed;
  // previous_speed=speed;
  // Serial.print("wheel:");Serial.print(speed); Serial.print(" ");
  // pre_speed=speed;
  float err=target-actual;
  // Serial.print("Speed err:");Serial.print(err); Serial.print("\t");
  // if (abs(err)<7){
  err_sum_pos+=err; // Serial.print("err sum:");Serial.print(err_sum_pos); Serial.print("\t");
  err_sum_pos=constrain(err_sum_pos,-20000,20000);
  // } else err_sum_pos=0;
  if (previous_target_pos != target) err_sum_pos=0;
  // if (previous_target_pos*target <= 0 ) err_sum_pos=0;
  previous_target_pos=target;

  // Serial.print("err sum:");Serial.print(err_sum_pos); Serial.print("\t");
  float KP_out,KI_out,KD_out;
  KP_out=KP_pos*err;
  KI_out=KI_pos*err_sum_pos;
  // KD_out=KD_pos*(err-pre_err);
  M1_speed=constrain(KP_out+KI_out+KD_out,-4096,4096);

  pre_err=err;

  // Serial.print("PI_output:"); Serial.print(PI_pos_output); Serial.print("\t");
}
////////////////////////////////////////////////////////////// Control //////////////////////////////////////////////////////////////
////////////////////////////////////////////// MOTOR OUTPUT //////////////////////////////////////////////
void motor_output_setup(){
  pinMode(17, OUTPUT);

  ledcAttachPin(M1_pin,M1_PWM_CH);
  ledcSetup(M1_PWM_CH,M1_PWM_freq,M1_PWM_res);

  ledcAttachPin(M2_pin,M2_PWM_CH);
  ledcSetup(M2_PWM_CH,M2_PWM_freq,M2_PWM_res);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void motor_output_loop(){
  if (M1_speed>=0){
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    ledcWrite(M1_PWM_CH, abs(M1_speed));
  } else if (M1_speed<0){
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    ledcWrite(M1_PWM_CH, abs(M1_speed));
  }

  if (M2_speed>=0){
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
    ledcWrite(M2_PWM_CH, abs(M2_speed));
  } else if (M2_speed<0){
    digitalWrite(BIN2, LOW); digitalWrite(BIN1, HIGH);
    ledcWrite(M2_PWM_CH, abs(M2_speed));
  }

  while ((micros()- loop2_start_time)<loop_time){} 
    loop2_start_time = micros(); 
    digitalWrite(S1_pin, HIGH);
    digitalWrite(S2_pin, HIGH);
    unsigned long S1_end_time=loop2_start_time+S1_angle;
    unsigned long S2_end_time=loop2_start_time+S2_angle;

    bool S1_state=1, S2_state=1;
    while(S1_state || S2_state){
      unsigned long now_time=micros();
      if (S1_end_time<=now_time){
        digitalWrite(S1_pin, LOW);
        S1_state=0;
      } else S1_state=1;
      if (S2_end_time<=now_time){
        digitalWrite(S2_pin, LOW); 
        S2_state=0;
      } else S2_state=1;
    }
}
////////////////////////////////////////////// MOTOR OUTPUT //////////////////////////////////////////////
////////////////////////////////////////////// I2C //////////////////////////////////////////////
void AS5600_setup(){
  // Wire.begin(21, 22);    // SDA=21, SCL=22
  // Wire.setClock(800000);
  if (!as5600_0.begin()) {
    Serial.println("No 1 AS5600 Found");
    while (1);
  }
}

void OLED_setup() {
  Wire.begin(21, 22);    // SDA=21, SCL=22
  Wire.setClock(800000);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // display.display();
  // delay(1000); // Pause for 2 seconds

  xTaskCreatePinnedToCore(
    OLED_task,   /* Task function. */
    "OLED Update",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &display_update_task,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
}

// void display_mode_dist(void * parameter) {
//   for(;;){
//     display.clearDisplay();
//     display.setCursor(14,0); display.println(pump_status_str);
//     display.setCursor(53,0); display.println(hardness_str);
//     display.setCursor(116,0); display.println(mode+1);
//     display.setCursor(45,16); display.println(target_size);
//     display.setCursor(45,40); display.println(actual_size);
//     display.display();
//   }
// }

void OLED_task(void * parameter){
  display_team();
  delay(10);
  for(;;){
    display_UI();
    vTaskDelay(pdMS_TO_TICKS(10));
    Serial.print("OLED running on core ");
    Serial.print(xPortGetCoreID());
  }
}

void display_team(void) {
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0,0); display.println(F("ME463"));
  display.setCursor(67,0); display.println(F("F2024"));

  // yellow ends at y=15
  display.setTextSize(3);             
  // display.setTextColor(SSD1306_WHITE);
  display.setCursor(3,16); 
  display.println(F("Remora"));
  display.setCursor(3,40); 
  display.println(F("Grip"));
  
  display.display();
  delay(3000);

  display.clearDisplay();
  display.setTextSize(2);             
  // display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0); 
  display.println(F("Welcome"));

  display.setTextSize(1); 
  int d=15;
  display.setCursor(0,16);     display.println(F("Aditya A"));
  display.setCursor(0,16+d);   display.println(F("Asher  M"));
  display.setCursor(0,16+2*d); display.println(F("Carter M"));
  display.setCursor(65,16);     display.println(F("Dingming L"));
  display.setCursor(65,16+d);   display.println(F("Owen     D"));
  display.setCursor(65,16+2*d); display.println(F("Ryan     K"));
  display.display();
  delay(2000);
}

void display_UI(void){
  display.clearDisplay();

  // display.setTextSize(2);
  // display.setTextColor(SSD1306_WHITE); 
  // display.setCursor(0,0); display.println(F("Mode"));
  // display.setCursor(67,0); display.println(F("F2024"));

  // yellow ends at y=15
  // UI
  display.setTextSize(1);  
  display.setCursor(0,16); display.println(F("Target"));
  display.setCursor(0,40); display.println(F("Actual"));
  display.setTextSize(2);             
  display.setCursor(0,0);  display.println(F("P"));
  display.setCursor(7,0);  display.println(F(":"));
  display.setCursor(105,0); display.println(F("M"));
  
  // data to change
  display.setCursor(14,0); display.println(pump_status_str);
  display.setCursor(53,0); display.println(hardness_str);
  display.setCursor(116,0); display.println(mode+1);
  display.setCursor(45,16); display.println(target_size);
  display.setCursor(45,40); display.println(actual_size);
  display.display();
  // delay(2000);

  // display.clearDisplay();
  // display.setCursor(0,0);  display.println(F("SU"));
  // display.setCursor(40,0); display.println(F("HA"));
  // display.setCursor(65,0); display.println(F("SO"));
  // display.setCursor(100,0); display.println(F("M:"));
  // display.setCursor(0,16); display.println(F("Actual"));
  // display.setCursor(0,46); display.println(F("Target"));
  

  // xTaskCreatePinnedToCore(
  //   display_mode_dist,   /* Task function. */
  //   "OLED Update",     /* name of task. */
  //   10000,       /* Stack size of task */
  //   NULL,        /* parameter of the task */
  //   0,           /* priority of the task */
  //   &display_update_task,      /* Task handle to keep track of created task */
  //   1);          /* pin task to core 1 */
}
////////////////////////////////////////////// I2C //////////////////////////////////////////////
