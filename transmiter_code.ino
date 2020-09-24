#include<SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6]  = {{'R', 'x', 'G', 'Y', 'A'},  {'T', 'x', 'G', 'E', 'o'} };

int counter = 0;
const char ping[]   = "b06a5f1ab41c7bd0705b0b85843a";     // ping code (public key randum)
const char code_s[] = "b067becac7f8a1059356ccbf278e"; // max size 32bytes
const char code_r[] = "0ceab8e71268fe7d44f36cc93d10";
char receive_buf[32] = "";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);

  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(100);
  radio.disableDynamicPayloads();
  radio.setAutoAck(false);

  delay(1);

  // WRITE [ping]
  radio.stopListening();
  int stat = radio.write(&ping, sizeof(ping)); // ping code
  Serial.print("ping code is send = "); Serial.println(stat);
  delay(1);

  // LISTENING
  radio.startListening();
  int timeout = 200; // 200ms.
  Serial.println("Listening..");
  while ( timeout > 0 ) { // wait responce
    timeout--;
    if ( radio.available() ) {
      radio.stopListening();

      // READ [code_s]
      clear_receivebuf();
      radio.read(&receive_buf, sizeof(receive_buf));
      Serial.println( receive_buf );

      // IF is code_s
      if ( strcmp(receive_buf, code_s) == 0) {
        Serial.println("code_s valid");

        // WRITE [code_r]
        radio.stopListening();
        int stat = radio.write(&code_r, sizeof(code_r)); // ping code
        Serial.print("ping code_r is send = "); Serial.println(stat);
        delay(1);

      }
      Serial.println("END. clearFIFO");
      clear_availableFIFO();
      return;
    }
    delay(1);
  }
  Serial.println("timeout responce");

}
void loop() {}

void clear_receivebuf() {
  receive_buf[32] = "";
}

void clear_availableFIFO() {
  radio.stopListening();
  char receive_buf[32] = "";
  int stack_n = 0;
  while (radio.available()) {
    radio.read(&receive_buf, sizeof(receive_buf));// read and clear
    Serial.print("clear buf/");
    Serial.print(stack_n);
    Serial.print("/");
    Serial.println(receive_buf);
    stack_n++;
  }
}
