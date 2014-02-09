/* This code can control Light control board and Musik Shield. 
 * It's derived from _2013_12_12_Musik_SoundControl_SerialProtocol.
 * Add new meta protocol part to parse "motor start","motor stop","motor change direction"
 * and "motor acceleration" commands. And throught the motorRoutine() function calculate the 
 * actual speed of each motor. And throught the actual speed of each motor calculate the
 * Volume of 6 Channel of the sound effect (6 different sound effect for different speed of 
 * motors).
 * Msg. format: "start;", "stop;", "blink A100 B100;"
 * 
 *
 * Edit: 8 Feb. 2014
 * Writen by Su Gao
 */
 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>  // for type definitions
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define KerzeSim 0
#define Blink 1
#define ON 2
#define OFF 3
int LEDState = 1;

#define normal 0
#define finish 1

#include <SoftwareSerial.h>
SoftwareSerial MusikSerial(11,12); //rx, tx


#define fastSpeed 10
#define middleSpeed 30
#define slowSpeed 60
/*
Start Wert: fest,0x86
 Device ID: 0 ... 255
 Kommando: xx
 Parameter 1
 Parameter 2
 Checksumme (CRC)
 */

byte Message[6]={
  0x86, 0x01, 0x02, 0x01, 0x1F, 0x00};
byte CRC, Channel, Volume;

int VolumeNow[8] = { //0-255, Max-0, Min-255, Volume of each channel
  0,0,0,0,0,0,0,0}; 
int SoundBlendSpeed[8] = { //sound blend speed of each channel
  6,6,6,6,6,6,6,6};
  
int motorState[4] = { //motor state. '1' - motor moving, '0' - motor stopped
  0,0,0,0};
int motorState_Old[4] = {  //old motor state.
  0,0,0,0};

  
int motorStateAll = 0;  //if any of the motor is moving - 1, if none of four - 0
int motorStateAll_Old = 0; //the state of the motor in last loop
  
long SpedMax[4] = {  //save the maximum speed of each motor, Unit: Hz (frequenz)
  4000,4000,4000,4000};
  
float accel[4] = { //save the acceleration value for each motor, Unit: Hz/ms
  3,3,3,3};
float brak[4] = {//acceleration on braking part
  12,12,12,12};
int accel_scaler = 4; //the scaler for converting from acceleration to braking acceleration
  
int dir[4] = {  //save the direction of the motor. '0'-Anticlockwise-go down, '1'-clockwise-go up  
  1,1,1,1};
int dir_Old[4] = { //the motor rotate direction of last loop
  1,1,1,1};
  
unsigned long motorStartTime[4] = {  //motor start moving time point
  0,0,0,0};
unsigned long motorMoveTime[4] = {  //motor moving time after last motor speed read and calculation
  0,0,0,0};
unsigned long motorMoveTime_Old[4] = {  //motor moving time totally after start moving
  0,0,0,0};
  
//save the calculated speed of each motor for the loop should be. 
//Speed positive - Plate go up, negative - go down
long Sped[4] = {  
  0,0,0,0};

static const byte p[] = {
  151,160,137,91,90, 15,131, 13,201,95,96,
  53,194,233, 7,225,140,36,103,30,69,142, 8,99,37,240,21,10,23,190, 6,
  148,247,120,234,75, 0,26,197,62,94,252,219,203,117, 35,11,32,57,177,
  33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,
  48,27,166, 77,146,158,231,83,30,229,122,6,211,133,255,255,105,92,
  41,55,46,245,40,244,102,143,54,65,25,63,161, 1,216,80,73,209,76,132,
  187,208, 89, 18,169,200,196,135,130,116,188,159, 86,255,100,109,198,
  173,186, 3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,
  212,207,206, 59,227, 47,16,58,17,182,189, 1,42,255,183,170,213,119,
  248,152,2,44,154,163,70,221,153,101,155,167,43,172, 1,129,22,39,253,
  19,98,40,110,79,113,224,232,178,185,112,104,218,246, 97,228,251,34,
  242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,
  49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,1,4,1, 4,
  150,254,138,236,205, 93,222,114, 67,29,24, 2,243,141,128,195,78,66,
  215,61,156,180
};

double fade(double t){ 
  return t * t * t * (t * (t * 6 - 15) + 10); 
}
double lerp(double t, double a, double b){ 
  return a + t * (b - a); 
}
double grad(int hash, double x, double y, double z)
{
  int     h = hash & 15;          /* CONVERT LO 4 BITS OF HASH CODE */
  double  u = h < 8 ? x : y,      /* INTO 12 GRADIENT DIRECTIONS.   */
  v = h < 4 ? y : h==12||h==14 ? x : z;
  return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
}

#define P(x) p[(x) & 255]

double pnoise(double x, double y, double z)
{
  int   X = (int)floor(x) & 255,             /* FIND UNIT CUBE THAT */
  Y = (int)floor(y) & 255,             /* CONTAINS POINT.     */
  Z = (int)floor(z) & 255;
  x -= floor(x);                             /* FIND RELATIVE X,Y,Z */
  y -= floor(y);                             /* OF POINT IN CUBE.   */
  z -= floor(z);
  double  u = fade(x),                       /* COMPUTE FADE CURVES */
  v = fade(y),                       /* FOR EACH OF X,Y,Z.  */
  w = fade(z);
  int  A = P(X)+Y,
  AA = P(A)+Z,
  AB = P(A+1)+Z,                        /* HASH COORDINATES OF */
  B = P(X+1)+Y,
  BA = P(B)+Z,
  BB = P(B+1)+Z;                        /* THE 8 CUBE CORNERS, */

  return lerp(w,lerp(v,lerp(u, grad(P(AA  ), x, y, z),   /* AND ADD */
  grad(P(BA  ), x-1, y, z)),   /* BLENDED */
  lerp(u, grad(P(AB  ), x, y-1, z),        /* RESULTS */
  grad(P(BB  ), x-1, y-1, z))),       /* FROM  8 */
  lerp(v, lerp(u, grad(P(AA+1), x, y, z-1),  /* CORNERS */
  grad(P(BA+1), x-1, y, z-1)),          /* OF CUBE\ */
  lerp(u, grad(P(AB+1), x, y-1, z-1),
  grad(P(BB+1), x-1, y-1, z-1))));
}

#define ledPin 13

void blink(int dur)
{
  digitalWrite(ledPin, HIGH);
  delay(dur);
  digitalWrite(ledPin, LOW);
}

// Serial comm reception
#define MAX_BUF         (64)
static char buffer[MAX_BUF];  // Serial buffer
static int sofar;             // Serial buffer progress


//----------------------------------
int brightNow[4] = {
  1500,1500,1500,1500}; //default brightness 1500. Max. 4095
int brightBlend[4] = {
  300,300,300,300}; //blend speed, 300 is very natural blend.

//----------------------------------
void setKerzeBright(int num, int brightness){
  brightness = brightness > 4000 ? 4000 : brightness;
  brightness = brightness < 500 ? 500 : brightness;
  brightNow[num-1] = brightness; //set new brightness.
  //Serial.println("LED" + String(num) + "brightness: ");
  //Serial.println(brightness);
}

//----------------------------------
void setKerzeBlend(int num, int blendSpeed=10){
  blendSpeed = blendSpeed > 800 ? 800 : blendSpeed;
  blendSpeed = blendSpeed < 10 ? 10 : blendSpeed;
  brightBlend[num-1] = blendSpeed;
}

//----------------------------------
void kerzeON(){
  for(int i=0; i<4;i++) pwm.setPWM(i,0,4000);
}

//----------------------------------
void kerzeOFF(){
  for(int i=0; i<4;i++) pwm.setPWM(i,0,0);
}

//--------------------------------------------------------------------
//Check if any motor is still moving, returned valued saved in variable motorStateAll
int motorStartStop(int motorState[]){  //motorState[i] of each motor
  for(int i=0; i<5; i++){
    if(motorState[i] == 1)
      return 1;  //any of four motors still moving
  }
  return 0; //all motors stopped
}

//overloaded function.
//return one motor's status
//return 1 - start moving, 2-moving, 3-stop moving, 4-stopped
int motorStartStop(int motorNum){  //motor Number (Number started from 1)
  int k = motorNum - 1; //make it easy to read
  
  if(motorState[k] == 1 && motorState_Old[k] == 0){
    motorState_Old[k] = motorState[k];
    return 1;
  }
  else if(motorState[k] == 1 && motorState_Old[k] == 1){  
    return 2;
  }
  else if(motorState[k] == 0 && motorState_Old[k] == 1){
    motorState_Old[k] = motorState[k];
    return 3;
  }
  else if(motorState[k] == 0 && motorState_Old[k] == 0){
    return 4;
  }
}

//Calculate the speed of each motor right now.
//return motor speed (vector variable) Unit: Hz
//save return value also automatically in Sped[] (motor speed array)
long calSpeed(int MotorNum){  //the motor number
  int k = MotorNum - 1; //make the code easier to read
  
  if(motorStartStop(MotorNum) == 1){  //motor start to move
    motorMoveTime_Old[k] = millis();
    Sped[k] = 0;
    return 0; //motor start to move, but speed still 0
  }
  else if(motorStartStop(MotorNum) == 2){  //motor is moving
    motorMoveTime[k] = millis() - motorMoveTime_Old[k]; //time slice between with the last time check
    //caculate the speed with direction, time, acceleration
    if(Sped[k] >= 0 && dir[k] == 1){
      //When motor moving direciton and acceleration direction are the same
      Sped[k] = Sped[k] + motorMoveTime[k] * accel[k]; 
      motorMoveTime_Old[k] = millis(); //update time
    }
    else if(Sped[k] < 0 && dir[k] == 0 ){
      Sped[k] = Sped[k] - motorMoveTime[k] * accel[k];
      motorMoveTime_Old[k] = millis(); 
    }
    else if(Sped[k] >=0 && dir[k] == 0){
      //When plate moving up and acceleration pointing down and braking
      Sped[k] = Sped[k] - motorMoveTime[k] * brak[k]; //brak[k]
      motorMoveTime_Old[k] = millis(); 
    }
    else if(Sped[k] < 0 && dir[k] == 1){
      //When plate moving down and acceleration pointing up and braking
      Sped[k] = Sped[k] + motorMoveTime[k] * brak[k]; //brak[k]
      motorMoveTime_Old[k] = millis(); 
    }    
    
    if(abs(Sped[k]) > SpedMax[k]){
      if(Sped[k] > 0) {
        Sped[k] = SpedMax[k];
        return SpedMax[k];
      }
      else if(Sped[k] < 0) {
        Sped[k] = -SpedMax[k];
        return -SpedMax[k];
      }
    }
    else {
      return Sped[k];
    }
  }
  else if(motorStartStop(MotorNum) == 3){ //here is "hard stop"
    Sped[k] = 0;
    return 0; //when stopped, speed is zero
  }
  else if(motorStartStop(MotorNum) == 4){  //motor stopped
    Sped[k] = 0;
    return 0;
  }
}

//save the motor sound effect in a .wav file on 6 different channel
//Blend these channels to simulate the motors movements
//Decide which sound should be the loudest sound from the 6 channel
//this sound should be direct setted louder and the other 5 direct
//much smaller
int soundDecider(long MSpeed){ //the acutale maximum speed of four motors right now
  //six level for each sound effect channel.The faster the speed is, the bigger is the number.
  int Stufer = 0;
  if(MSpeed < 3000) { //Unit: Hz
    Stufer = 0;
  } else if(MSpeed < 4000) {    
    Stufer = 1;
  } else if(MSpeed < 5000) {    
    Stufer = 2;
  } else if(MSpeed < 6000) {    
    Stufer = 3;
  } else if(MSpeed < 7000) {    
    Stufer = 4;
  } else if(MSpeed > 6999) {    
    Stufer = 5;  
  }
 
  return Stufer;
}

//Blend the targeted Channel louder and the untargeted Channel moute
void motorSoundBlender(int Stufer, long blendSpeed){ //which channel to blend loud & how fast should be blended
  VolumeNow[Stufer] = 0; //the Channel that should be heared now
  for(int i=0; i<7; i++){
    SoundBlendSpeed[i] = blendSpeed;
    if(i != Stufer){
      VolumeNow[i] = 100; //set the other motor sound effect Channel mute.
    }
  }  
}

//When all motors do not move, stop the sound directicaly
void stopSound(){
  for(int i=0; i<7; i++){
    VolumeNow[i] = 100;
    SoundBlendSpeed[i] = 20; //speed up the blend speed
  }
}


//update the motor status in each loop
void motorRoutine(){
  /*
  motorStateAll = motorStartStop(motorState);
  
  if(motorStateAll == 1 && motorStateAll_Old == 0){  //motor start to move    
    
  }

  else if(motorStateAll == 0 && motorStateAll_Old == 1){  //motor stop to move
    stopSound();
  }
  
  if(motorStateAll == 1){  //when any motor is still moving
    int Tone = soundDecider(1000); //TODO: finde way to get the acutal Max speed of four motor right now
    motorSoundBlender(Tone,1000); //TODO: find way to get the blendSpeed
  }
  }
  else if(motorStateAll == 0){  //when all motor stopped
  }
  */
  
  //update the motor speed each loop
  for(int i=1; i<5; i++){  
    calSpeed(i);
    Serial.print(Sped[i-1]); //display the speed that calculated
    Serial.print(F(" "));
  }
  Serial.println();

  motorStateAll_Old = motorStateAll; //update the state of all the motors    
}



//---------------------------------//
void processCommand(){

  if(!strncmp(buffer,"kerze",5)) {
    LEDState = KerzeSim;
  }
  else if(!strncmp(buffer,"blink",5)) {
    LEDState = Blink;
  }
  else if(!strncmp(buffer,"play",4)) {
    //PlaySong(3); //the correct Song
    char *ptr=buffer;    
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case '1': // first song
        PlaySong(1);
        Serial.println(F("Song 1 played")); //savle the 
        break;
      case '2': // second song
        PlaySong(2);
        Serial.println(F("Song 2 played"));
        break;
      case '3': //third song
        PlaySong(3);
        Serial.println(F("Song 3 played"));
        break;
      case '4': //third song
        PlaySong(4);
        Serial.println(F("Song 4 played"));
        break;
      case '5': //third song
        PlaySong(5);
        Serial.println(F("Song 5 played"));
        break;  
      case '6': //third song
        PlaySong(6);
        Serial.println(F("Song 6 played"));
        break;
      }
    ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }
  else if(!strncmp(buffer,"stop",4)) {
    Serial.println("Song stopped");
    StopSong();
  }
  //  else if(!strncmp(buffer,"config",6)){
  /*
  else if(!strncmp(buffer,"blink",5)){
   
   char *ptr=buffer;
   //Serial.println(buffer); //the displayed buffer show that after strncmp() buffer, buffer don't be changed at all.
   while(ptr && ptr<buffer+sofar && strlen(ptr)) {
   switch(*ptr) {
   case 'A':
   blinkSpeed = atof(ptr+1); //atol have even smaller range.
   Serial.print("BlinkSpeed is ");
   Serial.println(blinkSpeed);
   break;
   case 'B':
   blinkSpeed2 = atof(ptr+1);
   Serial.print("BlinkSpeed2 is ");
   Serial.println(blinkSpeed2);
   }
   ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
   }
   }
   */
  else if(!strncmp(buffer,"lt",2)){

    //brightness of each LED
    static float brightness1;
    static float brightness2;
    static float brightness3;
    static float brightness4;

    //blend speed of each LED
    static float blendSpeed1;
    static float blendSpeed2;
    static float blendSpeed3;
    static float blendSpeed4;

    static float LEDtrigger;

    char *ptr=buffer;
    //Serial.println(buffer); //the displayed buffer show that after strncmp() buffer, buffer don't be changed at all.
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': // brightness for each LED
        brightness1 = atof(ptr+1);
        //Serial.print("LED1 brightness: ");
        //Serial.println(brightness1);
        setKerzeBright(1,brightness1);
        break;
      case 'B':
        brightness2 = atof(ptr+1);
        //Serial.print("LED2 brightness: ");
        //Serial.println(brightness2);
        setKerzeBright(2,brightness2);
        break;
      case 'C':
        brightness3 = atof(ptr+1);
        //Serial.print("LED3 brightness: ");
        //Serial.println(brightness3);
        setKerzeBright(3,brightness3);
        break;
      case 'D':
        brightness4 = atof(ptr+1);
        //Serial.print("LED4 brightness: ");
        //Serial.println(brightness4);
        setKerzeBright(4,brightness4);
        break;
      case 'a': // LED blend speed for each LED
        blendSpeed1 = atof(ptr+1);
        //Serial.print("LED1 blendSpeed: ");
        //Serial.println(blendSpeed1);
        break;
      case 'b':
        blendSpeed2 = atof(ptr+1);
        //Serial.print("LED2 blendspeed: ");
        //Serial.println(blendSpeed2);
        break;
      case 'c':
        blendSpeed3 = atof(ptr+1);
        //Serial.print("LED3 blendspeed: ");
        //Serial.println(blendSpeed3);
        break;
      case 'd':
        blendSpeed4 = atof(ptr+1);
        //Serial.print("LED4 blendspeed: ");
        //Serial.println(blendSpeed4);
        break;
      case 'E':
        LEDtrigger = atof(ptr+1);
        //Serial.print("LEDtrigger set:"); //lt E0;
        //Serial.println(LEDtrigger);
        if(LEDtrigger)
          LEDState = ON;
        else 
          LEDState = OFF;
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }
  else if(!strncmp(buffer,"sd",2)){

    char *ptr=buffer;
    //Serial.println(buffer); //the displayed buffer show that after strncmp() buffer, buffer don't be changed at all.
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': //set Volume of each channel
        VolumeNow[0] = atof(ptr+1);
        //Serial.print("Vol.1: ");
        //Serial.println(VolumeNow[0]);
        break;
      case 'B':
        VolumeNow[1] = atof(ptr+1);
        //Serial.print("Vol.2: ");
        //Serial.println(VolumeNow[1]);
        break;
      case 'C':
        VolumeNow[2] = atof(ptr+1);
        //Serial.print("Vol.3: ");
        //Serial.println(VolumeNow[2]);
        break;
      case 'D':
        VolumeNow[3] = atof(ptr+1);
        //Serial.print("Vol.4: ");
        //Serial.println(VolumeNow[3]);
        break;
      case 'E':
        VolumeNow[4] = atof(ptr+1);
        //Serial.print("Vol.5: ");
        //Serial.println(VolumeNow[4]);
        break;
      case 'F':
        VolumeNow[5] = atof(ptr+1);
        //Serial.print("Vol.6: ");
        //Serial.println(VolumeNow[5]);
        break;
      case 'G':
        VolumeNow[6] = atof(ptr+1);
        //Serial.print("Vol.7: ");
        //Serial.println(VolumeNow[6]);
        break;
      case 'H':
        VolumeNow[7] = atof(ptr+1);
        //Serial.print("Vol.8: ");
        //Serial.println(VolumeNow[7]);
        break;
      case 'a':
        SoundBlendSpeed[0] = atof(ptr+1);
        //Serial.print("Channel1 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[0]);
        break;
      case 'b':
        SoundBlendSpeed[1] = atof(ptr+1);
        //Serial.print("Channel2 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[1]);
        break;
      case 'c':
        SoundBlendSpeed[2] = atof(ptr+1);
        //Serial.print("Channel3 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[2]);
        break;
      case 'd':
        SoundBlendSpeed[3] = atof(ptr+1);
        //Serial.print("Channel4 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[3]);
        break;
      case 'e':
        SoundBlendSpeed[4] = atof(ptr+1);
        //Serial.print("Channel5 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[4]);
        break;
      case 'f':
        SoundBlendSpeed[5] = atof(ptr+1);
        //Serial.print("Channel6 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[5]);
        break;
      case 'g':
        SoundBlendSpeed[6] = atof(ptr+1);
        //Serial.print("Channel7 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[6]);
        break;
      case 'h':
        SoundBlendSpeed[7] = atof(ptr+1);
        //Serial.print("Channel8 blendSpeed: ");
        //Serial.println(SoundBlendSpeed[7]);
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }
  
  //------------------------------------------------------------------------------
  //from here is the "meta sub command serial"
  else if(!strncmp(buffer,"stt",3)){  //update the motor state flag - motorState[], state - STarT
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': //change the first motor state flag in musik light board
        motorState[0] = 1; //motorState flag set to 1
        Serial.print(F("motor1 started. motorFlag[0]: "));
        Serial.println(motorState[0]);      
        break;
      case 'B':
        motorState[1] = 1; //motorState flag set to 1
        //Serial.print("motor2 started. motorFlag[1]: ");
        //Serial.println(motorState[1]);      
        break;
      case 'C':
        motorState[2] = 1; //motorState flag set to 1
        //Serial.print("motor3 started. motorFlag[2]: ");
        //Serial.println(motorState[2]);      
        break;
      case 'D':
        motorState[3] = 1; //motorState flag set to 1
        //Serial.print("motor4 started. motorFlag[3]: ");
        //Serial.println(motorState[3]);      
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }
  else if(!strncmp(buffer,"stp",3)){ //change motor state to 0, SToP mode
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': //change the first motor state flag in musik light board
        motorState[0] = 0; //motorState flag set to 1
        //Serial.print("motor1 stopped. motorFlag[0]: ");
        //Serial.println(motorState[0]);      
        break;
      case 'B':
        motorState[1] = 0; //motorState flag set to 0
        //Serial.print("motor2 stopped. motorFlag[1]: ");
        //Serial.println(motorState[1]);      
        break;
      case 'C':
        motorState[2] = 0; //motorState flag set to 0
        //Serial.print("motor3 stopped. motorFlag[2]: ");
        //Serial.println(motorState[2]);      
        break;
      case 'D':
        motorState[3] = 0; //motorState flag set to 0
        //Serial.print("motor4 stopped. motorFlag[3]: ");
        //Serial.println(motorState[3]);      
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }    
  else if(!strncmp(buffer,"smx",3)){ //set motor SpedMaximum value
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': //change the first motor state flag in musik light board
        SpedMax[0] = atof(ptr+1);
        //Serial.print("Max Speed 1 ");
        //Serial.println(SpedMax[0]);      
        break;
      case 'B':
        SpedMax[1] = atof(ptr+1);
        //Serial.print("Max Speed 2 ");
        //Serial.println(SpedMax[1]);      
        break;
      case 'C':
        SpedMax[2] = atof(ptr+1);
        //Serial.print("Max Speed 3 ");
        //Serial.println(SpedMax[2]);      
        break;
      case 'D':
        SpedMax[3] = atof(ptr+1);
        //Serial.print("Max Speed 4 ");        
        //Serial.println(SpedMax[3]);      
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }  
  else if(!strncmp(buffer,"acr",3)){ //set motoracceleration value
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': //change motors' acceleration value
        accel[0] = atof(ptr+1);
        brak[0] = accel_scaler *  accel[0];
        //Serial.print(accel[0]);
        //Serial.print(F(" "));
        //Serial.println(brak[0]);
        break;
      case 'B':
        accel[1] = atof(ptr+1);
        brak[1] = accel_scaler * accel[1];        
        //Serial.println(accel[1]);      
        break;
      case 'C':
        accel[2] = atof(ptr+1);
        brak[2] = accel_scaler * accel[2];        
        //Serial.println(accel[2]);      
        break;
      case 'D':
        accel[3] = atof(ptr+1);
        brak[3] = accel_scaler * accel[3];        
        //Serial.println(accel[3]);      
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }  
  else if(!strncmp(buffer,"dir",3)){ //set motors' direction value
    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': //change motors' acceleration value
        dir[0] = atof(ptr+1);
        //Serial.println(dir[0]);      
        break;
      case 'B':
        dir[1] = atof(ptr+1);
        //Serial.println(dir[1]);      
        break;
      case 'C':
        dir[2] = atof(ptr+1);
        //Serial.println(dir[2]);      
        break;
      case 'D':
        dir[3] = atof(ptr+1);
        //Serial.println(dir[3]);      
        break;
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }
}


//----------------------------------
void kerzeBlink(){
  blink(5); //control the speed of the light from dark to Bright
  static int brightness = 100;
  static int val = 100;
  val = (brightness==0 || brightness==4000) ? -val : val;
  brightness += val;
  for(int i=0; i<4;i++) pwm.setPWM(i,0,brightness);
}

//----------------------------------
void kerzeSim(){
  double a[4];
  static double b[4];
  int outputTmp[4];
  int output[4];

  static  int brightOld[4] = {
    1500,1500,1500,1500                }; //default brightness 1500. Max. 4095

  b[0] += 0.07;

  for(int i=1; i<4; i++) {
    b[i] = b[i-1] + 0.01;
  }

  blink(25); //this delay() control the speed of the kertze Brighness variation speed. war 125ms //before sound(): 50

  a[0] = pnoise(b[0] + sin(b[0]),b[0] +cos(b[0]),b[0]);
  a[1] = pnoise(b[1] + sin(b[1]),b[1] +sin(b[1]),b[1]);
  a[2] = pnoise(b[2] + cos(b[2]),b[2] +cos(b[2]),b[2]);
  a[3] = pnoise(b[3] + cos(b[3]),b[3] +sin(b[3]),b[3]);

  for (int i=0; i<4; i++){
    if(abs(brightNow[i] - brightOld[i]) > brightBlend[i] + 5) {
      brightOld[i] = brightNow[i] > brightOld[i] ? (brightOld[i] + brightBlend[i]) : (brightOld[i] - brightBlend[i]);
    }
    else{
      brightOld[i] = brightNow[i];
    }

    outputTmp[i] = int(127 + (a[i] * 127)); //second 127 changed from 10
    output[i] = int(map(outputTmp[i], 0, 255, 5/*300 relative naturl*/, brightOld[i]/*brightNow[i]*/));
    //    Serial.print(brightOld[i]);
    //    Serial.print(" ");
    pwm.setPWM(i,0,output[i]);
  }
  //  Serial.println();
}

//----------------------------------
void setup(){
  //Serial.begin(115200);
  Serial.begin(38400); //try the functionality so use a safer speed.
  Serial.println("\nHello, This is light & Sound Control Board");

  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  TWBR = 12; // upgrade to 400KHz!

  MusikSerial.begin(115200); //Musik Bautrate 11520, Device ID: 1
  SetVolume(0,1);
  PlaySong(2);  
}

//----------------------------------
void loop() {

  // See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/

  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read(); //save the Serial.read() and then get the sofar++
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions, break out the while loop to excute the command
  }

  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && buffer[sofar-1]==';') {
    buffer[sofar]=0; //because the last Serial.read() saved at buffer[sofar-1] and then sofar++, so there is nothing saved it buffer[sofar]

    // echo confirmation
    Serial.println(buffer);

    // do something with the command
    processCommand();

    // reset the buffer
    sofar=0;

    // echo completion
    Serial.print(F("> "));
  }
  /*
  if(!LEDToggleOn && !LEDToggleOff){ //demo blinky part to show the state
   digitalWrite(13, HIGH);
   delay(long(blinkSpeed));
   digitalWrite(13, LOW);
   delay(long(blinkSpeed2));
   }
   */

  if(LEDState == KerzeSim){
    kerzeSim();
  }
  else if(LEDState == Blink){
    kerzeBlink();
  }
  else if(LEDState == ON){
    kerzeON();
  }
  else if(LEDState == OFF){
    kerzeOFF();
  }
  
  motorRoutine();
  sound();
}



































