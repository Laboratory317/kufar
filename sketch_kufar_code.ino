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
#define UNSECURE  1
#define SECURE    0 

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin #4 (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool SCREEN_ALLOCATION = false; 

bool EN_SECURITY = true; // enabled security , status
byte code[7] = {1, 0, 1, 0, 0, 1, 1 }; // unclock code 

// Variables for setting the time / agusment delta = set - show_now
int delta_hours   = 22;
int delta_minutes = 25;

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
      go_servo(UNSECURE, 0);
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
      long last_time  = queue_lastTimes.dequeue();

      float delta_angle = ( angle_now - last_angle );
      long  delta_time  = ( time_now  - last_time  );
      if( angle_now < AC_ANGLE ){
        double v0 = ( (delta_angle)/delta_time ); // velocity (degree per miliseconds)
        Serial.println( v0);
        go_servo( SECURE, v0 );
      }
  }

  // UPDATE - put[LAST]
  queue_lastAngles.push( angle_now );
  queue_lastTimes.push( millis() );
  
}

/*
 * ===============================================================
 * * * Servo controller 
 * ===============================================================
     + Exponencial motion
     + feedback read
     + map( analogRead(A1), 57, 590, 0, 180 )
 * position lock mechanisum:
     unlock  s[1] - 180; s[2] - 0;   
     lock    s[1] - 90;  s[2] - 90;    
 */
void go_servo( byte activ, double v0 ){ 
    /*
     * REGULATOR VAR's
     * coefficient's( a1, a2, c1, c2 ) regulate curve 
     * ========================================================== */
      double delta_t = 10;  /* Δtime */
      //int delay_time = 5;
      
      double a = 2,      /* accel open slope */     
             c = 8.2  ;  /* half point of slope   */
      double y1, y2, y ;   /* var for function result  */
      int n = 1000;        /* assign No. of point plot */  
      
      /* * * SAVE VARIABLES * * */
      int timeout_counter = 2000; // milliseconds
     /* ==========================================================*/

    // ENABLE servos
    go_enable_servos();

    
    if( activ == UNSECURE )
    {
        // procedure unlock  
        servos[0].write(5);         // + LOAD FORCE ASSISTANT
        servos[1].write(180);       // 90 -> 180 ; unlock 
        servos[2].write(0);         // 90 -> 0   

        while( map(analogRead(FB_L), 57, 590, 0, 180) > 175 ){ // servos[1] feedback read
          if( timeout_counter-- < 0 ){
            // error beep ...
            go_disable_servos();
            return; // exit from function
          }
          delay(1);
        }

        // change status 
        EN_SECURITY = false;

        // procedure open 
        go_exp_motion_servo( 1 );
    }
    
    else
    {
      // procedure close 
      go_exp_motion_servo( 0 );

      while( map(analogRead(FB_D), 57, 590, 0, 180) > 10 ) {
        if( timeout_counter-- < 0 ){
            // error beep ...
            go_disable_servos();
            return; // exit from function
        }
        delay(1);
      }

      //procedure lock 
      servos[1].write(90);         // 180 -> 90 ; lock 
      servos[2].write(90);         // 0   -> 90 
      int _timeout_counter = 2000;
      while( map(analogRead(FB_L), 57, 590, 0, 180) < 98 ){
        if( _timeout_counter-- < 0 ){
            // error beep ...
            go_disable_servos();
            return; // exit from function
        }
      }

      // change status 
      EN_SECURITY = true;
      
  
    }

    go_disable_servos();
    delay(100);
}




void go_exp_motion_servo( int _direction ){ // 1 - open, 0 - close
  /*
     * REGULATOR VAR's
     * coefficient's( a1, a2, c1, c2 ) regulate curve 
     * ========================================================== */
      double delta_t = 10;  /* Δtime */
      int    scale_up_factor = 130;
      
      double a = 2,      /* accel open slope */     
             c = 8.2  ;  /* half point of slope   */
      double y1, y2, y ;   /* var for function result  */
      int n = 1000;        /* assign No. of point plot */  
     /* ==========================================================*/

    // change direction 
    if( _direction == 1 ){
      a *= -1;
      scale_up_factor = 90; // тук е за доизмисляне от къде да тръгва като кривата пак почва от нула
    }
    
    // procedure motion
    for( int i = 0; i < n ; i++ )
    {
      double f = ( delta_t / n ) * i;
      {
        y = 1/(1 + exp( a*(f-c) )); // calc curve motion
        y *= scale_up_factor;       // scale up factor *130 
        
        // DEBUG: Serial.println(y);
        servos[0].write( y );
        delay(delta_t);
      }
    }         
}


// ENABLE servo's
void go_enable_servos(){
  // set pin attach 
  servos[0].attach(3);
  servos[1].attach(4);
  servos[2].attach(5);
}

// DISABLE servo's
void go_disable_servos(){
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
     
  unsigned long Now = millis()/1000;
  int seconds = Now % 60;
  int minutes = ( Now / 60 ) % 60;
  int hours   = ( Now / 3600 ) % 24;

  // сверяваща щампа
  hours   += delta_hours;
  minutes += delta_minutes;

  print_dec_clock(hours);
  display.print(":");
  print_dec_clock(minutes);
  display.print(":");
  print_dec_clock(seconds);
  display.println();

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


void print_dec_clock(int n) {
  if (n<10) display.print('0');
  display.print(n);
}
