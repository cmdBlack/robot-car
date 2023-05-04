#include <Wire.h>
#include<stdio.h>
#include<stdbool.h>


int dir1A=2;
int dir2A=3;
int dir1B=4;
int dir2B=5;
int speedPinA=6;
int speedPinB=9;
int mSpeed=200;

char incoming=0;

#define MOTOR_GO_FORWARD{digitalWrite(dir1A, HIGH);digitalWrite(dir2A, LOW);digitalWrite(dir1B, HIGH);digitalWrite(dir2B, LOW);analogWrite(speedPinA, mSpeed);analogWrite(speedPinB, mSpeed);}
#define MOTOR_GO_LEFT{digitalWrite(dir1A, LOW);digitalWrite(dir2A, HIGH);digitalWrite(dir1B, HIGH);digitalWrite(dir2B, LOW); analogWrite(speedPinA, mSpeed);analogWrite(speedPinB, mSpeed);}
#define MOTOR_GO_RIGHT{digitalWrite(dir1A, HIGH);digitalWrite(dir2A, LOW);digitalWrite(dir1B, LOW);digitalWrite(dir2B, HIGH); analogWrite(speedPinA, mSpeed);analogWrite(speedPinB, mSpeed);}
#define MOTOR_GO_BACK{digitalWrite(dir1A, LOW);digitalWrite(dir2A, HIGH);digitalWrite(dir1B, LOW);digitalWrite(dir2B, HIGH); analogWrite(speedPinA, mSpeed);analogWrite(speedPinB, mSpeed);}
#define MOTOR_GO_STOP{digitalWrite(dir1A, LOW);digitalWrite(dir2A, LOW);digitalWrite(dir1B, LOW);digitalWrite(dir2B, LOW);analogWrite(speedPinA, mSpeed);analogWrite(speedPinB, mSpeed);}

enum DN
{ 
  GO_ADVANCE, 
  GO_LEFT, 
  GO_RIGHT,
  GO_BACK,
  STOP_STOP,
  DEF
}

Drive_Num=DEF;
bool flag1=false;
bool stopFlag = true;
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;

//#define MAX_PACKETSIZE 32
//char buffUART[MAX_PACKETSIZE];
//unsigned int buffUARTIndex = 0;
//unsigned long preUARTTick = 0;


void UART_Control(){
if(Serial.available()){ 
  char Uart_Date = Serial.read();
  
//    if(buffUARTIndex > 0 && (millis() - preUARTTick >= 100))
//    { //data ready
//        buffUART[buffUARTIndex] = 0x00;
//        if((buffUART[0]=='C') && (buffUART[1]=='M') && (buffUART[2]=='D'))
//        {
//            ;
//        }
//        else Uart_Date=buffUART[0];
//        buffUARTIndex = 0;
//    }
    switch (Uart_Date)
    {
      case '2': Drive_Num=GO_ADVANCE;Serial.print(Uart_Date);break;
      case '6': Drive_Num=GO_LEFT; Serial.print(Uart_Date);break;
      case '4': Drive_Num=GO_RIGHT;Serial.print(Uart_Date); break;
      case '8': Drive_Num=GO_BACK; Serial.print(Uart_Date);break;
      default:Serial.print(Uart_Date);break;
    }
     }
}


void CAR_Control()
{
    switch (Drive_Num) 
    {
      case GO_ADVANCE:MOTOR_GO_FORWARD;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case GO_LEFT: MOTOR_GO_LEFT;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case GO_RIGHT:MOTOR_GO_RIGHT;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case GO_BACK:MOTOR_GO_BACK;JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;
      case STOP_STOP: MOTOR_GO_STOP;JogTime = 0;JogFlag=false;stopFlag=true;break;
      default:break;
    }
    Drive_Num=DEF;
    if(millis()-JogTime>=210)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        MOTOR_GO_STOP;
      }
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(dir1A, OUTPUT);
  pinMode(dir2A, OUTPUT);
  pinMode(dir1B, OUTPUT);
  pinMode(dir2B, OUTPUT);
  pinMode(speedPinA, OUTPUT);
  pinMode(speedPinB, OUTPUT);
  MOTOR_GO_STOP;
  delay(10);

}

void loop() {
  // put your main code here, to run repeatedly:
  UART_Control();
  CAR_Control();

}