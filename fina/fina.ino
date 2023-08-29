#include <Timer1.h>
#include <Timer2.h>

#define FORWARD 0x09
#define BACKWARD 0x06
#define calc_PWM(_per)((unsigned int)(_per*2.55))
#define STOP 0x00
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
uint8_t mpuIntStatus;   // 保存从MPU读取的中断状态值
uint8_t devStatus;      // 每次装置运行后返回操作状态（0表示成功，！0表示失败）
uint16_t packetSize;    // 预先设置的DMP数据缓存大小（缺省值为42字节）
uint16_t fifoCount;     // FIFO中当前的缓存字节总数
uint8_t fifoBuffer[64]; // 定义数组存储缓存中的数据
Quaternion q;           // [w, x, y, z]         四元数
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   偏航角、仰卧角、翻滚角
unsigned int Timer_flag=0,Encoder_value_L=0,Encoder_value_R=0;
signed int Right_cnt = 0, Left_cnt = 0;                           //定义变量存储左右编码值
signed int pwm_value_R = 0, pwm_value_L = 0;                      //定义变量左右PWM值
unsigned int ENCODER_CNT_L=0, ENCODER_CNT_R=0;                    //定时器标志
signed int R_pre_err = 0;
signed int R_cur_err = 0;
signed int R_cur_cnt = 0;
signed int R_dt_err = 0;
signed int R_err_sum = 0;
signed int L_pre_err = 0;
signed int L_cur_err = 0;
signed int L_cur_cnt = 0;
signed int L_dt_err = 0;
signed int L_err_sum = 0;
int is_stop = 0;
//右侧转速设置
int RPM_R = 350;
int RPM_R_value = ((unsigned int)(RPM_R * 1.8));
//左侧转速设置
int RPM_L = 350;
int RPM_L_value = ((unsigned int)(RPM_L * 1.8));
//PID参数设置
#define KP  3
#define KI  0//0.00025
#define KD  0//0.125
//注：RPM_L = 80时，编码器500个脉冲 = 0.2m, 恒定误差0.11m---------------------------------------------------------------------------------
float setdistance = 5;             //设置行驶距离(米)-----------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------------
float distance = setdistance - 0.11;
int cntNow = 0;
int cntNumber = int(distance*(500/0.2));
int Motor[6] = {22,23,24,25,4,5};
unsigned char recv[7]={0};
unsigned char tmp_recv=0;
unsigned char last_recv=0;
int count=0;
long int sp,angle;
char sned[10];
int no_data=0;
int inland_flag= 1;
int last_inland_flag= 1;
int inland_count = 0;
int start_angle;
int overtake_count = 0;
int overtake_count_flag = 0;


void setup(){
  int z;
  for(z=0;z<6;z++)
  {
    pinMode(Motor[z],OUTPUT);
    digitalWrite(Motor[z],LOW);
  }
  attachInterrupt(6,Encoder_count_L,RISING);    //外部中断6服务程序，上升沿触发，执行中断服务程序Encoder_count_L
  attachInterrupt(7,Encoder_count_R,RISING);    //外部中断7服务程序，上升沿触发，执行中断服务程序Encoder_count_R
  Timer2::set(20000,Timer1_ISR);               //定时器1设置为每500000us触发一次中断，中断服务程序为Timer_ISR
  Motor_Model(FORWARD);


//--------------------------------------------mpu6050 init--------------
  Serial.begin(9600);
  Wire.begin();
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
//----------------------------------------------------------------------

  Serial3.begin(38400);
//  delay(15000);                                               //mpu6050初始化，否者读数跳变
  delay(5000); 
  Go_straight(1.6, 0.11, 255);
//  start_angle = get_angle();
} 

void loop(){
  //overtake();       //超车
  //park();           //倒车入库


if(Serial3.available()>0)
//    Serial.println(start_angle);
//    Serial.println(get_angle());
//    Serial.println("--------------------");
//    Serial.println(" ");
    
    {
        //Serial.write(Serial3.read());
        tmp_recv=Serial3.read();                
//        Serial.println(inland_flag);
        if( tmp_recv==0xAA)
        {
            memset(recv,0,7);
            count=1;
            goto end;
        }
    
        if(count>0)
        {
            recv[count++]=tmp_recv;
        }
        if(count==6)
        {
                no_data=1000;
                count=0;
                sp = (unsigned int)recv[1] + (unsigned int)recv[2]*256;
                angle = (unsigned int)recv[3] + (unsigned int)recv[4]*256;
    //              Serial.print("recv[1]");Serial.print(recv[1]);Serial.print("\n");
    //              Serial.print("recv[2]");Serial.print(recv[2]);Serial.print("\n");
    //              Serial.print("recv[3]");Serial.print(recv[3]);Serial.print("\n");
    //              Serial.print("recv[4]");Serial.print(recv[4]);Serial.print("\n");
//                  Serial.print("sp");Serial.print(sp);Serial.print("\n");
//                  Serial.print("angle");Serial.print(angle);Serial.print("\n");
    //              if(angle>2000)
    //              {
    //                  angle=2000; 
    //              }
    //              if(angle<1000)
    //              {
    //                  angle=1000;  
    //              }
                if(sp>3000)
                 {
                  inland_count = 0;
                  last_inland_flag = inland_flag;
                  while(sp > 3000)
                   {
                    sp -= 3000;
                    inland_count += 1;
                    }
                    inland_flag = inland_count;
                 }
    
                if(sp>1600)
                 {
                  sp=1600; 
                 }
                 if(sp<1400)
                 {
                    sp=1400;
                 }
                angle = angle-1500;
                sp = sp - 1500;

                if(overtake_count_flag == 1)
                {
                  overtake_count += 1;
                }
                if(overtake_count_flag == 1 && overtake_count > 75){//超车结束
                  turnR(25, 85);
                  overtake_count_flag = 0;
                  speed5(sp-(angle)*0.3,sp+(angle)*0.3);
                }
                
                if(sp < 0 && angle == 0)
                {
                   brake(sp,sp);
                }
                else if(inland_flag == 1) // default
                {
                    speed(sp-(angle)*0.2,sp+(angle)*0.2); //!!!!!!! 1550 速度可能没满
                }

               

                else if(inland_flag == 2) // overtake
                {
                    speed5(sp-(angle)*0.3,sp+(angle)*0.3);  
                    inland_flag = last_inland_flag;
                    turnL(30, 75);
                    overtake_count_flag = 1;
                    inland_flag = 8;
                }
                else if(inland_flag == 8) // default
                {
                    speed5(sp-(angle)*0.3,sp+(angle)*0.3); //!!!!!!! 1550 速度可能没满
                }


                

                else if(inland_flag == 3) // park
                {
                    inland_flag = last_inland_flag;
                    park();
                    while(1);
                }


                else if(inland_flag == 4) //10开始
                {
                    speed4(sp-(angle)*0.2,sp+(angle)*0.2);
//                    speed4
                } 

                else if(inland_flag == 5) //10结束
                {
                  speed_end10(sp-(angle)*0.2,sp+(angle)*0.2);  
                }

                else if(inland_flag == 6) //turnleft
                {
                  speed6(sp-(angle)*0.2,sp+(angle)*0.2);  
                }


                else if(inland_flag == 7) //进入环岛
                {
                  speed7(sp-(angle)*0.2,sp+(angle)*0.2);  
                }
                /*
                else if(inland_flag == 1) // first corner
                {
                    speed1(sp-(angle)*0.2,sp+(angle)*0.2);
                }else if(inland_flag == 2) // arrive inlan 6000
                {
                    speed2(sp-(angle)*0.2,sp+(angle)*0.2);
                }else if(inland_flag == 3) // turn left 9000
                {
                    speed3(sp-(angle)*0.2,sp+(angle)*0.2);
                } 
                */
        }
        end:
          last_recv=tmp_recv;
    
    }




  
  //while(1);
}

void overtake(){
  //Go_straight(1, 0.11, 150);
  turnL(30, 85);
  Go_straight(0.51, 0.11, 150);
  turnR(20, 75);              //angle, speed(0~100)
//  speed(80,)
  
  Go_straight(0.37, 0.11, 100);
  
  turnR(30, 85);
  Go_straight(0.48, 0.11, 150);
  turnL(26, 75);              //angle, speed(0~100)                                                   
  
  //Go_straight(0.5, 0.11, 150);  
}

void park(){
  Go_straight(0.3, 0.11, -150);
  turn(82, 75);                 //angle , speed(+L,-R)
  Go_straight(0.31, 0.11, -150);
}

void brake(int L,int R){
//-----------------
    if(L>100)
    {
        L=100;
    }else if(L<-100)
    {       
      L=-100;
    };
    if(R>100)
    {
        R=100;
    }else if(R<-100)
    {       
      R=-100;
    };
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}


//有限幅
void speed(int L,int R)
{
  unsigned int OC_value = 0;
    if(L>100)
    {
        L=100;
    }
    else if(L<-30)
    {
      R = R - L;
      L=-30; 
    }
    if(R>100)
    {
      R=100;
    }
    else if(R<-30)
    {
      L = L - R;        
      R=-30;
    }
//-----------------
    if(L>100)
    {
        L=100;
    }else;
    if(R>100)
    {
        R=100;
    }else;
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}


void speed4(int L,int R)
{
  unsigned int OC_value = 0;
    if(L>70)
    {
        L=70;
    }
    else if(L<-30)
    {
//      R = R - L;
      L=-30; 
    }
    if(R>70)
    {
      R=70;
    }
    else if(R<-30)
    {
//      L = L - R;        
      R=-30;
    }
//-----------------
    if(L>100)
    {
        L=100;
    }else;
    if(R>100)
    {
        R=100;
    }else;
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}

void speed5(int L,int R)//超车结束
{
  unsigned int OC_value = 0;
    if(L>100)
    {
        L=100;
    }
    else if(L<-100)
    {
//      R = R - L;
      L=-100; 
    }
    if(R>100)
    {
      R=100;
    }
    else if(R<-100)
    {
//      L = L - R;        
      R=-100;
    }
//-----------------
    if(L>100)
    {
        L=100;
    }else;
    if(R>100)
    {
        R=100;
    }else;
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}




void speed7(int L,int R)
{
  unsigned int OC_value = 0;
    if(L>100)
    {
        L=100;
    }
    else if(L<-20)
    {
//      R = R - L;
      L=-20; 
    }
    if(R>100)
    {
      R=100;
    }
    else if(R<-20)
    {
//      L = L - R;        
      R=-20;
    }
//-----------------
    if(L>100)
    {
        L=100;
    }else;
    if(R>100)
    {
        R=100;
    }else;
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}





void speed_end10(int L,int R)
{
  unsigned int OC_value = 0;
    if(L>100)
    {
        L=100;
    }
    else if(L<-20)//超车结束，左转不限幅
    {
//      R = R - L;
      L=-20; 
    }
    if(R>100)
    {
      R=100;
    }
    else if(R<-20)
    {
//      L = L - R;        
      R=-20;
    }
//-----------------
    if(L>100)
    {
        L=100;
    }else;
    if(R>100)
    {
        R=100;
    }else;
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}





void speed6(int L,int R)
{
  unsigned int OC_value = 0;
    if(L>100)
    {
        L=100;
    }
    else if(L<-100)
    {
//      R = R - L;
      L=-100; 
    }
    if(R>100)
    {
      R=100;
    }
    else if(R<-100)
    {
//      L = L - R;        
      R=-100;
    }
//-----------------
    if(L>100)
    {
        L=100;
    }else;
    if(R>100)
    {
        R=100;
    }else;
//-----------------
  
    
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  

    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}








//无限幅
void speed_turn(int L,int R)
{
  unsigned int OC_value = 0;
    if(L>=0)
    {    
      digitalWrite(Motor[0],HIGH);  
      digitalWrite(Motor[1],LOW);  
    }  
    else
    {
      digitalWrite(Motor[0],LOW);  
      digitalWrite(Motor[1],HIGH);  
    }
    if(R>=0)
    {    
      digitalWrite(Motor[2],LOW);  
      digitalWrite(Motor[3],HIGH);  

    }  
    else
    {
      digitalWrite(Motor[2],HIGH);  
      digitalWrite(Motor[3],LOW);  
    }

    L = abs(L);
    R = abs(R);
    analogWrite(Motor[4],calc_PWM(L));
    analogWrite(Motor[5],calc_PWM(R));
}

void Motor_Model(int da)
{
  int z;
  for(z=0;z<4;z++)
  {
      digitalWrite(Motor[z],(da>>z)&0x01);
  }
}




//--------------------------------------------------------------------------------------------------------------------------------
void Go_straight(float tmp_distance, float tmp_error, int speedin){
  ENCODER_CNT_R = 0;
  ENCODER_CNT_L = 0;
  is_stop = 0;
  distance = tmp_distance - tmp_error;
  cntNow = 0;
  cntNumber = int(distance*(500/0.2));

  Timer2::start();
  while(1){
    if(Timer_flag)
    {
      if(is_stop == 0){
        if(speedin > 0)
          Motor_mode(FORWARD);
        else
          Motor_mode(BACKWARD);
        Motor_Control('R', speedin);
        Motor_Control('L', speedin);
//        Motor_Control('R', PID_R());
//        Motor_Control('L', PID_L());
      }
      else if(is_stop = 1){
        Timer2::stop();
//        Serial.println("Stop!!");
        if(speedin > 0)
          Motor_mode(BACKWARD);
        else
          Motor_mode(FORWARD);
        Motor_Control('R', 60);
        Motor_Control('L', 60);

        delay(300);
        speed(0,0);
        Motor_Control('R', 0);
        Motor_Control('L', 0);
        break;
      }
      Timer_flag = 0;
    }
    else{
      Timer_flag = 0;
    }
  }  
}

//马达模式---------------------------------------------------------------------------------------------------------------------------
void Motor_mode(int da)
{
  int z;
  for(z=0;z<4;z++)
    digitalWrite(Motor[z],(da>>z) & 0x01);
}

//马达控制函数，L为左轮运转，R为右轮运转，A为左右轮同时运转-----------------------------------------------------------------------------------------
void Motor_Control(char da, unsigned int speedValue)
{
  switch(da)
  {
    case 'L':
      analogWrite(Motor[4],speedValue);
      break;
    case 'R':
      analogWrite(Motor[5],speedValue);
      break;
    case 'A':
      analogWrite(Motor[4],speedValue);
      analogWrite(Motor[5],speedValue);
      break;
    default:
      analogWrite(Motor[4],0);
      analogWrite(Motor[5],0);
      break;
  }
}

//-------------------------------------------------------------add------------------------------------------------------------------------
void Encoder_count_L()
{
  ENCODER_CNT_L++;//自加1
  cntNow++;
//  Serial.println(cntNow);
}

void Encoder_count_R()
{
  ENCODER_CNT_R++;//自加1
//  cntNow++;
//  Serial.println(cntNow);
}

//定时器1中断服务程序，将定时器1标志位置1--------------------------------------------------------------------------------------------------------
void Timer1_ISR()
{
  Right_cnt = ENCODER_CNT_R;
  Left_cnt = ENCODER_CNT_L;
  ENCODER_CNT_R = 0;
  ENCODER_CNT_L = 0;
  Timer_flag = 1;
  //是否到达设定行驶距离判断
//  Serial.println(cntNow);
  if(cntNow >= cntNumber){
//    Serial.println("---------------------------");
//    Serial.print("number:");Serial.println(cntNumber);
//    Serial.print("cnt:");Serial.println(cntNow);
//    Serial.println("---------------------------");
    is_stop = 1;
  }
}

//PID速度计算-----------------------------------------------------------------------------------------------------------------------------
int PID_R(){
  R_pre_err = R_cur_err;
  R_cur_cnt = Right_cnt;
  R_cur_err = RPM_R_value - R_cur_cnt;
  R_err_sum += R_cur_err;
  R_dt_err = R_cur_err - R_pre_err;
  //PID算法计算右轮PWM值
  pwm_value_R += (R_cur_err * KP) + (R_err_sum * KI) + (R_dt_err * KD);

  if(pwm_value_R > 200)
    pwm_value_R = 200;
  else if(pwm_value_R > 200){
    pwm_value_R = 0;  
  }
  return pwm_value_R;
}

int PID_L(){
  L_pre_err = L_cur_err;
  L_cur_cnt = Left_cnt;
  L_cur_err = RPM_L_value - L_cur_cnt;
  L_err_sum += L_cur_err;
  L_dt_err = L_cur_err - L_pre_err;
  //PID算法计算左轮PWM值
  pwm_value_L += (L_cur_err * KP) + (L_err_sum * KI) + (L_dt_err * KD);

  if(pwm_value_L > 200)//最大值为255
    pwm_value_L = 200;
  else if(pwm_value_L < 0){
    pwm_value_L = 0;  
  }
  return pwm_value_L;
}

void turn(int turn_angle, int turn_speed){
  float begin_angle;
  float now_angle;
  int i;
  int angle_error;
  i = 0;
  angle_error = 0;
  begin_angle = get_angle();
  while(1){
    now_angle = get_angle();


    
//    ----------------------------------------------------------
    if(begin_angle > (360 - turn_angle)){
      
      if(now_angle > (begin_angle - (360 - turn_angle)) && now_angle < (begin_angle + 10 - (360 - turn_angle))){
        angle_error = turn_angle;  
      }
      else{
        angle_error = 0;  
      }
    }
    else{
      angle_error = int(abs(now_angle - begin_angle));  
    }
//    -----------------------------------------------------------


    if(angle_error >= turn_angle){
      speed(0, 0);
//      Serial.print("begin:");Serial.println(begin_angle);
//      Serial.print("now:");Serial.println(now_angle); 
//      Serial.print("error:");Serial.println(angle_error);
//      Serial.println("Done!!");
      break;
    }
    else{
      speed_turn(turn_speed, -turn_speed-25);
    }
  }
}

void turnR(int turn_angle, int turn_speed){
  float begin_angle;
  float now_angle;
  int i;
  int angle_error;
  i = 0;
  angle_error = 0;
  begin_angle = get_angle();
  while(1){
    now_angle = get_angle();


    
//    ----------------------------------------------------------
    if(begin_angle > (360 - turn_angle)){
      
      if(now_angle > (begin_angle - (360 - turn_angle)) && now_angle < (begin_angle + 10 - (360 - turn_angle))){
        angle_error = turn_angle;  
      }
      else{
        angle_error = 0;  
      }
    }
    else{
      angle_error = int(abs(now_angle - begin_angle));  
    }
//    -----------------------------------------------------------


    if(angle_error >= turn_angle){
//      speed(0, 0);
      break;
    }
    else{
      speed6(turn_speed, -turn_speed);
    }
  }
}

void turnL(int turn_angle, int turn_speed){
  float begin_angle;
  float now_angle;
  int i;
  int angle_error;
  i = 0;
  angle_error = 0;
  begin_angle = get_angle();
  while(1){
    now_angle = get_angle();


    
//    ----------------------------------------------------------
    if(begin_angle < turn_angle){
      
      if(now_angle < (360 - (turn_angle - begin_angle)) && now_angle > (360 - 10 - (turn_angle - begin_angle))){
        angle_error = turn_angle;  
      }
      else{
        angle_error = 0;  
      }
    }
    else{
      angle_error = int(abs(now_angle - begin_angle));  
    }
//    -----------------------------------------------------------


    if(angle_error >= turn_angle){
//      speed(0, 0);
      break;
    }
    else{
      speed6(-turn_speed, turn_speed);
    }
  }
}

float get_angle(){
  while(1)
    {
      mpu.resetFIFO();
      mpuIntStatus = mpu.getIntStatus();
      fifoCount = mpu.getFIFOCount();
      if(mpuIntStatus & 0x02)
      {
        while(fifoCount < packetSize)
          fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        break;
      }
    }
    return 180 - (ypr[0] * 180/M_PI);
}
