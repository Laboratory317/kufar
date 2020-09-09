#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <QueueArray.h>
#include<Servo.h>

// FIFO for calculation delta in measurement
QueueArray<float> queue_lastAngles;
QueueArray<long>  queue_lastTimes;
#define QUEUE_OPERATION_SIZE      10 

Servo servos[3]; 
#define FB_D   A0 // feedback door 
#define FB_L   A1 // feedback lock
#define DOOR_LEVEL_CLOSED 0
#define DOOR_LEVEL_OPEN   130
#define buzzer 13
#define OPEN  1
#define CLOSE 0 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin #4 (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool SCREEN_ALLOCATION = false; 

bool EN_SECURITY = true; // enabled security , status
byte code[7] = {1, 0, 1, 0, 0, 1, 1 }; // unclock code 

void setup() {
  pinMode( FB_D,    INPUT );
  pinMode( FB_L,    INPUT );
  pinMode( buzzer, OUTPUT );
  Serial.begin(9600);
  
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally 
  SCREEN_ALLOCATION = display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32 
  if( !SCREEN_ALLOCATION ) {
     Serial.println(F("SSD1306 oled display allocation failed"));
  }
     
  tone( buzzer, ((SCREEN_ALLOCATION)? 1320 : 740), 50);
  delay(100);
}

void loop() { 
  if( SCREEN_ALLOCATION ) 
  {
    disp_ownerInfo();
    intrpt_delay(3000);
  
    for( int i = 0; i < 3 ; i++ ){
        disp_securityStatus();
        intrpt_delay(1000);
    }
  
     disp_time();
     intrpt_delay(1500);
  }
  else /* only check */ intrpt_delay(10); 
}

void intrpt_delay( int _milliseconds ){
  long feature_time =  millis() + _milliseconds;
  while( millis() < feature_time ){
    ( EN_SECURITY )? check_RF() : check_motion();
  }
}

void check_RF(){
  /* listening for RF unlock signal
   * unlock, validate unlock and open */
  if( Serial.available() > 0 ){
    // read data from RF module 
    char c = Serial.read();
    if( c == 'a' ){ // if valid code 
      tone( buzzer, 1047 /* C6 */ , 100);
      // GO SERVO UNLOCK AND OPEN
      go_servo(OPEN, 0);
    }
  }
}

/*
 * ==============================================================
 * * * Motion detector 
 * ==============================================================
 *    + read feedback from servo 
 *    + calc delta angle and time changes 
 *    + check activation angle */ int AC_ANGLE = 110 ; /*
 *    + calc velocity  ( F v-> )
 *    + call go_servo( CLOSE, v0 );
 */
void check_motion(){
  float angle_now = map( analogRead( FB_D ), 82, 440, 0, 130);
  long time_now   = millis();
 
  if( queue_lastAngles.count() >= QUEUE_OPERATION_SIZE ){
      float last_angle = queue_lastAngles.dequeue();
      long last_time  = queue_lastTime.dequeue();

      float delta_angle = ( angle_now - last_angle );
      long  delta_time  = ( time_now  - last_time  );
      if( angle_now < AC_ANGLE ){
        double v0 = ( (delta_angle)/delta_time ); // velocity (degree per miliseconds)
        Serial.println( v0);
        go_servo( CLOSE, v0 );
      }
  }
  
}

/*
 * ===============================================================
 * * * Servo controller 
 * ===============================================================
     + Exponencial motion
     + feedback read
 * position lock mechanisum:
     unlock  s[1] - 180; s[2] - 0;   
     lock    s[1] - 90;  s[2] - 90;    
 */
void go_servo( byte activ, double v0 ){ 
    /*
     * REGULATOR VAR's
     * coefficient's( a1, a2, c1, c2 ) regulate curve 
     * ========================================================== */
      double delta_t;  /* Δtime */
      
      /* accel and decel slope */    /* half point of slope   */
      double a1 = 6,  a2 = 3,          c1 = 1.8,   c2 = 8.2  ;
      double y1, y2, y ;   /* var for function result  */
      int n = 1000;        /* assign No. of point plot */
     /* ==========================================================*/
  
  // ENABLE servo's
  servos[0].attach(3);
  servos[1].attach(4);
  servos[2].attach(5);

  
  if( activ == OPEN )
  {
    // procedure unlock  
    servos[1].write(180);
    servos[2].write(0);
    delay(200);
    EN_SECURITY = false;

    // procedure open
    for( int i = 0; i < n ; i++ ){
      double f = delta_t + ( delta_t / n ) * i;
      {
        y1 = 1/(1 + exp( -a1*(f-c1) ));
        //y2 = 1/(1 + exp( -a2*(f-c2) ));
        y = y1 * DOOR_LEVEL_OPEN; /* scale up factor *130 */
        Serial.println(y);
        servos[0].write( y );
        delay(10);
      }
    }
  }
  else
  {
    // procedure close 
    
    // procedure lock
    servos[1].write(90);
    servos[2].write(90);
    delay(200);
  }

  // DISABLE servo's
  for( int i = 0; i < sizeof(servos); i++ ){
    servos[i].detach();
  }
  
}







/* ================================================================
 * * * DISPLAY FRAME's * * 
 * create function disp_< frame_name > for anyone frame
 * ================================================================
 * disp_ownerInfo();
 * disp_securityStatus();
 * disp_time();
 * disp_RFinit();
 * disp_RFvalid();
 * ================================================================
 */
 
void disp_ownerInfo(){
  display.clearDisplay();
  // draw logo 
  display.fillRect(0, 0, 7, 32, SSD1306_WHITE); // първи вертикален
  display.fillRect(7, 0, 14, 7, SSD1306_WHITE); // висок хоризонтален до триъгълника
  display.fillRect(7, 25, 25, 7, SSD1306_WHITE); // долен хоризонтален
  display.fillRect(12, 12, 8, 8, SSD1306_WHITE); // квадрата в средата 10x10
  display.fillRect(25, 15, 7, 16, SSD1306_WHITE); // вертикален отдясно 
  display.fillTriangle(20, 0,  20, 6,  26, 6, SSD1306_WHITE); // триъгълника
  
  display.setTextSize(1);             
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(40,2 );             
  display.print(F("Georgi"));
  display.setCursor(40,11 );             
  display.print(F("Chakarov"));
  display.setCursor(40,24 );             
  display.print(F("0/879-689-408"));
  
  display.display();
}

void disp_securityStatus(){
  display.clearDisplay();
  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor( (EN_SECURITY)?(random(0, 52)):(random(0, 30)) ,random(0, 18) ); 
  display.print((EN_SECURITY)?F("LOCKED"):F("UNSECURE"));
  display.display();
}

void disp_time(){
  display.clearDisplay();
  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0,0);             
  display.println(F("17:02"));
  display.setTextSize(1);
  display.println(F("UTC+2 BG located"));
  display.display();
}

void disp_RFinit(){
  display.clearDisplay();

  for(int16_t i=0; i< 14; i+=2) {
    display.drawRoundRect(i, i, 64-2*i, 32-2*i, 8, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(75,6);             
  display.println(F("RF"));
  display.display();
}

void disp_RFvalid(){
  display.clearDisplay();

  for(int16_t i=0; i<14; i+=2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, 64-2*i, 32-2*i, 8, SSD1306_INVERSE);
    display.display();
    delay(1);
  }
  display.setTextSize(2);             
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(75,6);             
  display.println(F("RF"));
  display.setTextSize(1);             
  display.setCursor(75,25);             
  display.println(F("validate"));
  display.display();
  
}


