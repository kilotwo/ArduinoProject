/*
    每个轮相连的步进电机按下图与步进电机扩展版连接
           车头
   X:0--------------2:Z
     |              |
     |              |
     |              |
   Y:1--------------3:A
           车尾
    各电机与扩展版的每个插槽的连接顺序为:
    从上至下: 蓝, 红, 绿, 黑
*/
#include <ServoTimer2.h>
#include <Arduino.h>
#include <Wire.h>
#include <MultiLCD.h>
//Sensor
int debug_sensor = A3; //程序触发传感器

ServoTimer2 m_Servo[5]; 

#define MOVE_DBG  //底盘行走测试,前进, 后退, 左转, 右转, 左平移, 右平移
LCD_SSD1306 lcd; //OLED

//Servo
char angle_return[100];
const int frequency = 25; 
const int speedd = 25;
const int actionTimes = 2000;
int servo_port[5] = {22, 24, 26, 28, 29}; //定义机械臂各舵机引脚，基座至机械爪
float servo_angle[5] = {73, 112, 31, 113, 90}; //定义机械臂各舵机初始角度
const int servo_num = sizeof(servo_port) / sizeof(servo_port[0]); //定义舵机数量
//Path
static String data = ""; //存储二维码顺序
int order[2];  //存储抓取及放置的顺序
//====================================+变量定义枚举类型+=======================================
enum Direction{MOVE_FORWARD= 1, MOVE_BACK, MOVE_LEFT, MOVE_RIGHT, TURN_RIGHT, TURN_LEFT};
enum Cmd{QR=1, COLOR};

struct XY_POS{
  double x;
  double y;
  double theta;
}pos;
//====================================-变量定义-=======================================
void setup() {        //初始化
  Serial.begin(9600);
   pinMode(debug_sensor, INPUT);
     LcdInit();
  lcd.begin();
  lcd.clear();
   ServoInit();
  delay(1000);
  initMotors();
  XYStop();
} 
 /*
  moveTo(-0.5, 0, true, MOVE_LEFT); //右平移
  moveTo(0.5, 0, false, MOVE_RIGHT); //左平移
  moveTo(0, 0.5, false, MOVE_FORWARD); //前进
  moveTo(0, -0.5, false, MOVE_BACK); //后退
  */
  
void loop() 
{
#ifdef MOVE_DBG
 // moveTo(0.4, 0, false, MOVE_RIGHT); //左平移
   moveTo(0, 1.0, false, MOVE_FORWARD); //前进
   //moveTo(-0.4, 0, true, MOVE_LEFT); //右平移
//   moveTo(0, -0.4, false, MOVE_BACK); //后退
   delay(1000);
   QRDetect();                      //此时得到二维码顺序   
   delay(1000);
    moveTo(-0.7, 0, false, MOVE_LEFT); //右平移
  Colour()   ;                 //颜色检测
  // 根据抓取顺序进行物料抓取  1：left ；2：right
  Putt();
    delay(1000);
  moveTo(0, -0.5, false, MOVE_BACK); //后退 
 
  
#endif
  // if(!digitalRead(debug_sensor))
  //  SendCmd(QR);
  while(1);
}
void serialEvent(){
  while(Serial.available())
    Serial.read();
}
//colour显示函数
void LcdDisplay(String data){
  lcd.clear();
  lcd.setCursor(0, 2);
  lcd.print(data);
}

/*
  舵机初始化函数
*/
void ServoInit(){
  for(int i=0;i<servo_num;i++){
    m_Servo[i].attach(servo_port[i]);
    ServoGo(i, servo_angle[i]);
  }
}
 // OLED初始化函数

void LcdInit(){
  lcd.begin();
  lcd.clear();
}
//====================================+OLED显示函数+=====================================

//函数名: LcdDisplay();
//mode:QR 显示二维码识别结果
void LcdDisplayer(String datas){
    lcd.clear();
  lcd.setCursor(0, 3);
  lcd.printLong(datas.toInt(), FONT_SIZE_LARGE);
}


/*
  舵机角度最大值及最小值检测函数
  小于最小值, 返回最小值
  大于最大值, 返回最大值
*/
float ValueCompare(float value){
  if(value <= 0) 
    return 0;
  else if(value >= 180)
    return 180;
  else 
    return value;
}

/*
  舵机角度转换为PWM函数
  参数为: 舵机角度
  返回值: PWM
*/
float Angle2Pwm(float angle){
  return map(ValueCompare(angle), 0, 180, 500, 2500);
}

void ServoGo(int which, float angle){
  m_Servo[which].write(Angle2Pwm(angle));
}

void ServoMove(float * pCurrent, float * pTarget){
  float diff[servo_num];
  for(int i=0;i<servo_num;i++)
    diff[i] = (*(pTarget+i) - *(pCurrent+i)) == 0 ? 0 : (*(pTarget+i) - *(pCurrent+i)) / frequency;
  for(int i=0;i<frequency;i++){
    for(int j=0;j<servo_num;j++){
      *(pCurrent+j) += diff[j];
      ServoGo(j, *(pCurrent+j));
    }
    delay(speedd);
  }
}

void ServoSet(int count, ...){		
  float targetAngle[count];
  va_list ap;
  va_start(ap, count);
  for(int i=0;i<count;i++)
    targetAngle[i] = va_arg(ap, int);
  va_end(ap);
  ServoMove(servo_angle, targetAngle);
  delay(1000);
}
void  Colour()
{
  
    // 颜色检测
  ServoSet(servo_num, 73,61,71,85,83); //机械臂移动至第一个物料处
  ServoSet(servo_num, 73,61,54,82,83); //机械臂移动至第一个物料处
  ServoSet(servo_num, 55,61,54,82,83); //机械臂移动至第一个物料处
   SendCmd(COLOR);                     //检测放置台左侧物料颜色
  ServoSet(servo_num,107,61,54,82,83); //机械臂移动至第二个物料处
 SendCmd(COLOR); //检测放置台右侧物料颜色
  
  ServoSet(servo_num,73, 112, 31, 113, 90); //机械臂复位
    delay(1000);
      String serialData = SendCmd(COLOR); //获取颜色检测结果
      LcdDisplayer(serialData);        //颜色检测结果
 //      delay(1000);
   // LcdDisplayer(serialData);        //颜色检测结果
    OrderSet(serialData);          //排序结果
    delay(1000);
  
}

//机械臂放置
void Putt(){
  //根据抓取顺序进行物料抓取  0：left ；1：right
 // for(int i=0;i<2;i++)
  //{
    switch(order[0])
    {
      case 0:
        //get 0    left
        ServoSet(servo_num, 73,74,42,82,90);  //抓取1
        ServoSet(servo_num, 43,74,42,82,90);  //抓取2
        ServoSet(servo_num, 48,67,40,82,100);  //抓取3
           ServoSet(servo_num, 48,67,39,30,100);  //抓取4
              ServoSet(servo_num, 48,67,5,40,100);  //抓取5
                 ServoSet(servo_num, 48,55,20,40,100);  //抓取6
                    ServoSet(servo_num, 48,40,42,48,100);  //抓取7
        ServoSet(servo_num,48,40,42,48,72);    //抓紧1
        ServoSet(servo_num, 48,64,61,80,72);    //抓紧1中间
        ServoSet(servo_num,125,64,61,80,72);    //抓紧2中间
         moveTo(-0.5, 0, false, MOVE_LEFT); //右平移
         
        ServoSet(servo_num, 125,64,61,80,100);  //松开1
        
            moveTo(0.5, 0, false, MOVE_LEFT); // left平移
      break;
      case 1:  
        //get 1     right
         ServoSet(servo_num, 73,74,42,82,90);  //抓取1
        ServoSet(servo_num, 100,74,42,82,90);  //抓取2
        ServoSet(servo_num, 100,67,40,82,100);  //抓取3
           ServoSet(servo_num, 100,67,39,30,100);  //抓取4
              ServoSet(servo_num, 100,67,5,40,100);  //抓取5
                 ServoSet(servo_num, 100,55,20,40,100);  //抓取6
                    ServoSet(servo_num, 100,40,42,48,100);  //抓取7
      ServoSet(servo_num,100,40,42,48,72);    //抓紧1
        ServoSet(servo_num,100,64,61,80,72);    //抓紧1中间
        ServoSet(servo_num,125,64,61,80,72);    //抓紧2中间
         moveTo(-0.5, 0, false, MOVE_LEFT); //右平移
         delay(2000);
        ServoSet(servo_num, 125,64,61,80,100);  //松开1
        delay(2000);
            moveTo(0.5, 0, false, MOVE_LEFT); //left平移
    /*    
        ServoSet(servo_num, 13, 80, 42, 90, 105);
        ServoSet(servo_num, 13, 90, 38, 90, 105);
        ServoSet(servo_num, 13, 130, 71, 78, 105);
        ServoSet(servo_num, 13, 130, 71, 78, 53);
        ServoSet(servo_num, 13, 80, 42, 90, 53);
        ServoSet(servo_num, 93, 80, 42, 90, 53);
        */
      break;
    }
     ServoSet(servo_num, 73,61,71,85,83); //2.机械臂复位
}

//==================================-二维码检测-======================================

/******************************串口读取及数据处理部分********************************/
/*
 * 串口读取函数
 * 返回值类型：字符串
 */
String SerialRead(){
  String incomingStr = "";
  while(true){
    while(Serial.available()){
      incomingStr = Serial.readStringUntil('\n');
    }
    if(incomingStr != "")
      break;
  }
  return incomingStr;
}

/*

//串口发送二维码识别及颜色检测命令
//mode:检测模式
//mode=QR:检测二维码
//mode=COLOR:检测颜色
 */
String SendCmd(int mode){
  switch(mode){
    case QR: Serial.print("qr\n"); break;
    case COLOR: Serial.print("get\n"); break;
  }
  String cmd = SerialRead();
  if(cmd != "error"){
    
   // LcdDisplay(cmd);
    return cmd;
  }
  else{
    //此处可以通过控制机械臂来调整摄像头，使其能够检测到目标
    SendCmd(mode);
  }
}

//=======================================+抓取及放置函数+=================================
void OrderSet(String str){
  int n = str.toInt();
  order[0] = n/10;
  order[1] = n%10;
}
//==================================+二维码检测+=====================================
/*
  二维码识别函数
  机械臂控制摄像头对准二维码
  然后发送检测命令至上位机
  等待接收上位机返回的数据(抓取顺序, 即 '12'的随机组合)
  将数据在OLED上显示
*/
void QRDetect(){
  ServoSet(servo_num, 73, 30, 64, 2, 82); //1.移动摄像头至二维码屏幕
  data = SendCmd(QR); //获取上位机二维码识别结果mode=QR:检测二维码
   LcdDisplayer(data);
   //  LcdDisplay("hello" );
   delay(1000);
   ServoSet(servo_num, 73, 112, 31, 113, 90); //2.机械臂复位
    delay(1000);
   
}

