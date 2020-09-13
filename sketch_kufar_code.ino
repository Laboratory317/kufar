#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include<Servo.h>

Servo servos[3];
#define FB_D   A0            // pin feedback door - A0
#define FB_L   A1            // pin feedback lock - A1
#define buzzer 7             // pin buzzer 
#define SERVO_PWR_ENABLE A3  // pin MOSFET enable power servo 
#define TILT_AMPULA      6   // pin tilt ampula input

#define DOOR_LEVEL_CLOSED 5     // min position servo door closed
#define DOOR_LEVEL_OPEN   146   // max 
#define UNSECURE  1
#define SECURE    0


// DEBUG DEFINES
//#define DEBUG_SERVOS_P
//#define DEBUG_EXP
//#define DEBUG_MOTION


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin #4 (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool SCREEN_ALLOCATION = false;

bool EN_SECURITY = true; // enabled security , status
byte code[7] = {1, 0, 1, 0, 0, 1, 1 }; // unclock code

// Variables for setting the time / agusment delta = set - show_now
long delta_stampTimeSet = 83460 ;

void setup() {
  pinMode( FB_D,    INPUT );
  pinMode( FB_L,    INPUT );
  pinMode( buzzer, OUTPUT );
  pinMode( TILT_AMPULA, INPUT_PULLUP );
  pinMode( SERVO_PWR_ENABLE, OUTPUT ); // connect internal mos to GND when is logic 0 ( not open colector )!
  digitalWrite( SERVO_PWR_ENABLE, 1 );     // enable mosfet servo power
  go_disable_servos();

  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  SCREEN_ALLOCATION = display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  if ( !SCREEN_ALLOCATION ) {
    Serial.println(F("SSD1306 oled display allocation failed"));
  }

  tone( buzzer, ((SCREEN_ALLOCATION) ? 1320 : 740), 50);
  delay(100);
}


/**
   ===============================================================================
   ==== LOOP =====
   + set show frames
   + check intrerupt call
   ===============================================================================
*/
void loop() {
  if ( SCREEN_ALLOCATION )
  {
    disp_ownerInfo();
    intrpt_delay(3000);

    for ( int i = 0; i < 3 ; i++ ) {
      disp_securityStatus();
      intrpt_delay(1000);
    }

    disp_time();
    intrpt_delay(1500);
  }
  else /* only check */ intrpt_delay(10);
}

/* ==============================================================================*/



void intrpt_delay( int _milliseconds ) {
  long feature_time =  millis() + _milliseconds;
  while ( millis() < feature_time ) {
    check_serial_comport_pc();
    ( EN_SECURITY ) ? check_RF() : check_motion();
  }
}

void check_RF() {

  /* listening for RF unlock signal
     unlock, validate unlock and open */
  if ( Serial.available() > 0 ) {
    if ( SCREEN_ALLOCATION ) {
      disp_RFinit();
      delay(100);
    }

    // read data from RF module
    String code = Serial.readString();
    if ( code == "a" ) { // if valid code
      
      if( digitalRead( TILT_AMPULA ) == 1 ){ // not leveled
        error_message( F("Hold at the correct level!") );
        return;
      }
      
      tone( buzzer, 1047 /* C6 */ , 100);
      
      if ( SCREEN_ALLOCATION ) disp_RFvalid();
      
      go_servo(UNSECURE, 0); // GO SERVO UNLOCK AND OPEN
    }
  }
}

void check_serial_comport_pc() {
  // read serial data
  // delta_stampTimeSet = new_data
}




/*
   ==============================================================
 * * * Motion detector
   ==============================================================
      + read feedback from servo
      + calc delta angle and time changes
      + check activation angle */ int AC_ANGLE = 138 ; /*
      + calc velocity  ( F v-> )
      + call go_servo( CLOSE, v0 );
*/
void check_motion() {
  digitalWrite( SERVO_PWR_ENABLE, 1 ); // enable power to servo for measuring pullup
  delay(1); // await register shift and read from ADC
  int angle_now = map( analogRead(FB_D), 152, 336, 0, 180 );
  digitalWrite( SERVO_PWR_ENABLE, 0 );

#ifdef DEBUG_MOTION
  Serial.print("a[");
  Serial.print( angle_now);
  Serial.print("];");
  Serial.println();
#endif

  if ( angle_now < AC_ANGLE ) {
    go_servo( SECURE, 0 );
    delay(2000);
  }

}

/*
   ===============================================================
 * * * Servo controller
   ===============================================================
     + Exponencial motion
     + feedback read
     + map( analogRead(A1), 57,  590, 0, 180 )
     + map( analogRead(A0), 152, 336, 0, 180 )
   position lock mechanisum:
     unlock  s[1] - 180; s[2] - 0;
     lock    s[1] - 90;  s[2] - 90;
*/
void go_servo( byte activ, double v0 ) {

  int timeout_counter = 2000;
  // ENABLE servos
  go_enable_servos();


  if ( activ == UNSECURE )
  {
    // procedure unlock
    servos[0].write(0);         // + LOAD FORCE ASSISTANT
    servos[1].write(180);       // 90 -> 180 ; unlock
    servos[2].write(0);         // 90 -> 0

    while ( map( analogRead(FB_L), 57,  590, 0, 180 ) < 175 ) { // servos[1] feedback read
      if ( timeout_counter-- < 0 ) {
        go_disable_servos();
        error_message(F("timeout unlock"));
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

    while ( map( analogRead(FB_D), 152, 336, 0, 180 ) > 10 ) {
      if ( timeout_counter-- < 0 ) {
        go_disable_servos();
        error_message(F("timeout close"));
        return; // exit from function
      }
      delay(1);
    }

    //procedure lock
    servos[1].write(90);         // 180 -> 90 ; lock
    servos[2].write(90);         // 0   -> 90
    int _timeout_counter = 2000;
    while (  map( analogRead(FB_L), 57,  590, 0, 180 ) > 94 ) {
      if ( _timeout_counter-- < 0 ) {
        go_disable_servos();
        error_message(F("timeout lock"));
        return; // exit from function
      }
      delay(1);
    }

    // change status
    EN_SECURITY = true;


  }

  go_disable_servos();
  delay(1000);
}




void go_exp_motion_servo( int _direction ) { // 1 - open, 0 - close
  /*
       REGULATOR VAR's
       coefficient's( a1, a2, c1, c2 ) regulate curve
       ========================================================== */
  double delta_t = 10;  /* Δtime */
  int    scale_up_factor = ( AC_ANGLE - 20 ) - DOOR_LEVEL_CLOSED ;

  double a = 7,        /* accel open slope */
         c = 9  ;    /* half point of slope   */
  double y ;         /* var for function result  */
  int n = 1000;      /* assign No. of point plot */

  // points(i) in interval [ n_0 - n_x1000 ]
  int n_0 = 859;
  int n_x = 1000;
  /* ==========================================================*/

  // change direction
  if ( _direction == 1 ) { // if open
    a = -2;
    scale_up_factor = DOOR_LEVEL_OPEN ; // тук е за доизмисляне от къде да тръгва като кривата пак почва от нула
    n_0 = 763;
    c = 8.2;


    // procedure motion
    for ( int i = n_0; i < n_x ; i++ ) // foreach points(i) in interval [ 0 - 1000 ]
    {

      double f = ( delta_t / n ) * i;
      {
        y = 1 / (1 + exp( a * (f - c) )); // calc curve motion
        y *= scale_up_factor;       // scale up factor *130
        y += DOOR_LEVEL_CLOSED;
        servos[0].write( y );
        delay(delta_t);
      }

        #ifdef DEBUG_EXP
              Serial.print("i[");
              Serial.print(i);
              Serial.print("]  y[");
              Serial.print(y);
              Serial.println();
        #endif
    }
  }
  else {

    for ( int i = AC_ANGLE - 10; i > 7; i-- ) {
      servos[0].write( i );
      delay(47);
    }
  }
}


// ENABLE servo's
void go_enable_servos() {
  // set pin attach
  servos[0].attach(3);
  servos[1].attach(4);
  servos[2].attach(5);
  digitalWrite( SERVO_PWR_ENABLE, 1);

#ifdef DEBUG_SERVOS_P
  Serial.println("ENABLED SERVOS");
#endif
}

// DISABLE servo's
void go_disable_servos() {
  digitalWrite( SERVO_PWR_ENABLE, 0 ); // interupt servo power- reset automaticly position hold

  for ( int i = 0; i < sizeof(servos); i++ ) {
    servos[i].detach();
  }
  delay(10);

#ifdef DEBUG_SERVOS_P
  Serial.println("DISABLED SERVOS");
#endif

}









/* ================================================================
 * * * DISPLAY FRAME's * *
   create function disp_< frame_name > for anyone frame
   ================================================================
   disp_ownerInfo();
   disp_securityStatus();
   disp_time();
   disp_RFinit();
   disp_RFvalid();
   ================================================================
*/

void disp_ownerInfo() {
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
  display.setCursor(40, 2 );
  display.print(F("Georgi"));
  display.setCursor(40, 11 );
  display.print(F("Chakarov"));
  display.setCursor(40, 24 );
  display.print(F("0/879-689-408"));

  display.display();
}

void disp_securityStatus() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor( (EN_SECURITY) ? (random(0, 52)) : (random(0, 30)) , random(0, 18) );
  display.print((EN_SECURITY) ? F("LOCKED") : F("UNSECURE"));
  display.display();
}

void disp_time() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  unsigned long Now = (millis() / 1000) + delta_stampTimeSet;
  int seconds = Now % 60;
  int minutes = ( Now / 60 ) % 60;
  int hours   = ( Now / 3600 ) % 24;

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

void disp_RFinit() {
  display.clearDisplay();

  for (int16_t i = 0; i < 14; i += 2) {
    display.drawRoundRect(i, i, 64 - 2 * i, 32 - 2 * i, 8, SSD1306_WHITE);
    display.display();
    delay(1);
  }
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(75, 6);
  display.println(F("RF"));
  display.display();
}

void disp_RFvalid() {
  display.clearDisplay();

  for (int16_t i = 0; i < 14; i += 2) {
    // The INVERSE color is used so round-rects alternate white/black
    display.fillRoundRect(i, i, 64 - 2 * i, 32 - 2 * i, 8, SSD1306_INVERSE);
    display.display();
    delay(1);
  }
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(75, 6);
  display.println(F("RF"));
  display.setTextSize(1);
  display.setCursor(75, 25);
  display.println(F("validate"));
  display.display();

}


/**
   =================================================================
   ANOTHER FUNCTION
   + display show
   =================================================================
*/

void error_message( String _exeption ) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Exeption:"));
  display.setTextSize(1);
  display.println( _exeption );
  display.display();

  // error beep times
  for ( int i = 0; i < 8 ; i++ ) {
    tone( buzzer, 988, 100 );
    delay(200);
  }
  delay(4000);
}

void print_dec_clock(int n) {
  if (n < 10) display.print('0');
  display.print(n);
}
