#include <avr/io.h>
#include <avr/interrupt.h>

//===============================+PIN definitions+===============================
//enable pin
#define EN_PIN 8
//direction pins
#define DIR_PIN_0 5
#define DIR_PIN_1 6
#define DIR_PIN_2 7
#define DIR_PIN_3 13
//directions
#define MFORWARD_0 LOW
#define MFORWARD_1 HIGH
#define MFORWARD_2 LOW
#define MFORWARD_3 HIGH
//pulse pins
#define PUL_PIN_0 2
#define PUL_PIN_1 3
#define PUL_PIN_2 4
#define PUL_PIN_3 12
//===============================-PIN definitions-===============================

//===============================+imer definitions+=============================
#define TIMER1 1
#define TIMER3 3
#define TIMER4 4
#define TIMER5 5

#define SYSTEM_CORE_CLOCK 16000000.0
//===============================-timer definitions-===============================

//===============================+robot config+===============================
#define D_WHEEL 0.06;
const double CWheel = M_PI * D_WHEEL;

#define MAIN_STEP 200
#define MICRO_STEP 8
#define TOTAL_STEP (MAIN_STEP * MICRO_STEP)

const double VRatio = (double)(2 * TOTAL_STEP) / CWheel; // 100 = TIMER_LIMIT / 1000000(ns per s)
const double VYRatio = -1.25;   //修改此参数调节平移速度
const double WRatio = 0.88 * 2000.0 * 0.04 / M_PI * 180.0; //0.88 修改此参数调节旋转速度
// const double WRatio = 0.88 * 2000000.0 * 0.04 / M_PI * 180.0;

#define MAX_VELOCITY 0.4  //limit:0.4
#define MAX_OMEGA 1.0

inline void run0(double v0);
inline void run1(double v1);
inline void run2(double v2);
inline void run3(double v3);

struct XY_LAST_STATUS
{
  unsigned long t_millis;
  double vx;
  double vy;
  double w;
}lastStat;
//===============================-robot config-===============================

//init method
void initMotors(){
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);
  
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
  lastStat.t_millis = 0;
  lastStat.vx = 0;
  lastStat.vy = 0;
  lastStat.w = 0;
  
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  
  pinMode(PUL_PIN_0, OUTPUT);
  pinMode(PUL_PIN_1, OUTPUT);
  pinMode(PUL_PIN_2, OUTPUT);
  pinMode(PUL_PIN_3, OUTPUT);
  
  digitalWrite(DIR_PIN_0, MFORWARD_0);
  digitalWrite(DIR_PIN_1, MFORWARD_1);
  digitalWrite(DIR_PIN_2, MFORWARD_2);
  digitalWrite(DIR_PIN_3, MFORWARD_3);
  
  digitalWrite(PUL_PIN_0, LOW);
  digitalWrite(PUL_PIN_1, LOW);
  digitalWrite(PUL_PIN_2, LOW);
  digitalWrite(PUL_PIN_3, LOW);
}

//===============================+position API+===============================
void updatePos() 
{ 
  double dt = millis() - lastStat.t_millis;
  dt /= 1000.0;
  lastStat.t_millis = millis();

  double lastTheta = pos.theta;
  pos.theta += dt * lastStat.w;
  pos.theta -= double(floor(pos.theta / 2.0 / M_PI)) * 2.0 * M_PI;

  if(lastStat.w == 0) {
    pos.x += dt * (lastStat.vx * cos(lastTheta) - lastStat.vy * sin(lastTheta));
    pos.y += dt * (lastStat.vy * cos(lastTheta) + lastStat.vx * sin(lastTheta));
  }
  else {
    double v = sqrt(lastStat.vx * lastStat.vx + lastStat.vy * lastStat.vy);
    double r = v / lastStat.w;
  
    pos.x += r * (sin(pos.theta) - sin(lastTheta));
    pos.y += r * (-cos(pos.theta) + cos(lastTheta));
  }
  
  pos.x = 0;
  pos.y = 0;
  pos.theta = 0;
  lastStat.t_millis = 0;
  lastStat.vx = 0;
  lastStat.vy = 0;
  lastStat.w = 0;
}
//===============================-position API-===============================

//===============================+robot control+===============================
//vx: m/s; vy: m/s; w: rad/s
void XYRun(double vx, double vy, double w, int dir)
{
  double v = sqrt(vx*vx+vy*vy);
  if(v > MAX_VELOCITY || w > MAX_OMEGA) return;
  double vy_t = vy * VYRatio;
  double v0 = VRatio * (vx+vy_t);
  double v1 = VRatio * (vx-vy_t);
  double w_t = WRatio * w;
  
  updatePos();
  lastStat.vx = vx;
  lastStat.vy = vy;
  lastStat.w = w;
  
  double v_lf = (v0 - w_t); //左前
  double v_rf = (v1 + w_t); //右前
  double v_lb = (v1 - w_t); //左后
  double v_rb = (v0 + w_t); //右后
  
  switch(dir){
    case MOVE_LEFT: break;
    case MOVE_RIGHT: break;
    case TURN_LEFT: v_rb = 0; v_rf *= 0.2; break;
    case TURN_RIGHT: v_lf = 0; v_rf *= 0.2; break;
  }
 
  run0(v_lf); run1(v_rf); run2(v_lb); run3(v_rb);
}

//v: m/s; dir: rad; w: rad/s
inline void XYRunVDW(double v, double dir, double w) {XYRun(v*cos(dir),v*sin(dir),w,0);}

inline void XYStop() {XYRun(0,0,0,0);}

//步进电机使能开启
inline void StepperEnable(){digitalWrite(EN_PIN, LOW);};

//步进电机使能关闭
inline void StepperDisable(){digitalWrite(EN_PIN, HIGH);};
//===============================-robot control-===============================

//===============================-velocity settings-===============================
inline void run0(double v0) {
  if(v0 < 0) {
    digitalWrite(DIR_PIN_0, MFORWARD_0 ^ 1);
    TimerStart(TIMER1, -v0);
  } else if(v0 == 0) {
    TimerStop(TIMER1);
  } else {
    digitalWrite(DIR_PIN_0, MFORWARD_0);
    TimerStart(TIMER1, v0);
  }
}
inline void run1(double v1) {
  if(v1 < 0) {
    digitalWrite(DIR_PIN_1, MFORWARD_1 ^ 1);
    TimerStart(TIMER3, -v1);
  } else if(v1 == 0) {
    TimerStop(TIMER3); 
  } else {
    digitalWrite(DIR_PIN_1, MFORWARD_1);
    TimerStart(TIMER3, v1);
  }
}
inline void run2(double v2) {
  if(v2 < 0) {
    digitalWrite(DIR_PIN_2, MFORWARD_2 ^ 1);
    TimerStart(TIMER4, -v2);
  } else if(v2 == 0) { 
    TimerStop(TIMER4);
  } else {
    digitalWrite(DIR_PIN_2, MFORWARD_2);
    TimerStart(TIMER4, v2);
  }
}
inline void run3(double v3) {
  if(v3 < 0) {
    digitalWrite(DIR_PIN_3, MFORWARD_3 ^ 1);
    TimerStart(TIMER5, -v3);
  } else if(v3 == 0) {
    TimerStop(TIMER5);
  } else {
    digitalWrite(DIR_PIN_3, MFORWARD_3);
    TimerStart(TIMER5, v3);
  }
}
//===============================-velocity settings-===============================

//====================================+timer+======================================

void TimerStart(int which, double frequency){
  double _RC = SYSTEM_CORE_CLOCK / 8.0 / frequency - 1;
  cli();
  switch(which){
    case TIMER1: 
      TCCR1A = 0; TCCR1B = 0; 
      OCR1A = _RC;
      //TCCR1B = _BV(WGM12) | _BV(CS10) | _BV(CS11); 
      TCCR1B = _BV(WGM12) | _BV(CS11); 
      TIMSK1 |= (1 << OCIE1A); 
      break;
    case TIMER3: 
      TCCR3A = 0; TCCR3B = 0; 
      OCR3A = _RC;
      //TCCR3B = _BV(WGM12) | _BV(CS10) | _BV(CS11); 
      TCCR3B = _BV(WGM12) | _BV(CS11); 
      TIMSK3 |= (1 << OCIE3A); 
      break;
    case TIMER4: 
      TCCR4A = 0; TCCR4B = 0; 
      OCR4A = _RC;
      //TCCR4B = _BV(WGM12) | _BV(CS10) | _BV(CS11); 
      TCCR4B = _BV(WGM12) | _BV(CS11); 
      TIMSK4 |= (1 << OCIE4A); 
      break;
    case TIMER5: 
      TCCR5A = 0; TCCR5B = 0; 
      OCR5A = _RC;
      //TCCR5B = _BV(WGM12) | _BV(CS10) | _BV(CS11); 
      TCCR5B = _BV(WGM12) | _BV(CS11); 
      TIMSK5 |= (1 << OCIE5A); 
      break;
  }
  sei();
}

void TimerStop(int which){
  switch(which){
    case TIMER1: TCCR1B = 0; break;
    case TIMER3: TCCR3B = 0; break;
    case TIMER4: TCCR4B = 0; break;
    case TIMER5: TCCR5B = 0; break;
  }
}

ISR(TIMER1_COMPA_vect){
  static byte _state = 0;
  static byte _pin = PUL_PIN_0;
  digitalWrite(_pin, _state ^= 1);
}
ISR(TIMER3_COMPA_vect){
  static byte _state = 0;
  static byte _pin = PUL_PIN_1;
  digitalWrite(_pin, _state ^= 1);
}
ISR(TIMER4_COMPA_vect){
  static byte _state = 0;
  static byte _pin = PUL_PIN_2;
  digitalWrite(_pin, _state ^= 1);
}
ISR(TIMER5_COMPA_vect){
  static byte _state = 0;
  static byte _pin = PUL_PIN_3;
  digitalWrite(_pin, _state ^= 1);
}
//====================================-timer-======================================

//====================================+move+======================================
/*
  车移动函数
  参数:
    x: 车在x轴方向上移动的距离
    y: 车在y轴方向上移动的距离
    _loop: true:以一定速度一直转动 ; false: 转动指定距离
    dir: 车移动方向
*/
void moveTo(double x, double y, boolean _loop, int dir)
{
  updatePos();
  //logPosition();
  double dx = x - pos.x;
  double dy = y - pos.y;
  //if(!dx&&!dy)return;
  double vx = dx > 0 ? 0.1 : -0.1;
  double vy = dy > 0 ? 0.1 : -0.1; 
  double tx = dx / vx;
  double ty = dy / vy;
  double t_double = tx > ty ? tx : ty;
  unsigned long t = ceil(t_double*1000.0);
  if(t==0) return;
  vx = dx / (double)t * 1000.0;
  vy = dy / (double)t * 1000.0;
  XYRun(vx, vy, 0, dir);
  
  if(!_loop){
    delay(t);
    XYStop();
  }
}
//====================================-move-======================================
