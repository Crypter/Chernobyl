#include <PS2X.h>
#include <Servo.h>
#include "esp-rclink.h"

/*
PSB_SELECT       0x0001
PSB_L3           0x0002
PSB_R3           0x0004
PSB_START        0x0008
PSB_PAD_UP       0x0010
PSB_PAD_RIGHT    0x0020
PSB_PAD_DOWN     0x0040
PSB_PAD_LEFT     0x0080
PSB_L2           0x0100
PSB_R2           0x0200
PSB_L1           0x0400
PSB_R1           0x0800
PSB_GREEN        0x1000
PSB_RED          0x2000
PSB_BLUE         0x4000
PSB_PINK         0x8000
PSB_TRIANGLE     0x1000
PSB_CIRCLE       0x2000
PSB_CROSS        0x4000
PSB_SQUARE       0x8000
*/

int16_t off_rx=0, off_ry=0, off_lx=0, off_ly=0;

PS2X joystick;
Servo servos[9];

uint8_t oldModes[9];

uint8_t pinMappings[9] = {D0, D1, D2, D3, D4, D5, D6, D7, D8};

void dataIn(){
  if (ESPRCLink.deviceMode() == ESPRC_SLAVE){
    for (int i=0; i<9; i++){
      if(oldModes[i] != ESPRCLink.masterState.mode[i]){
        oldModes[i] = ESPRCLink.masterState.mode[i];
        if (ESPRCLink.masterState.mode[i]==ESPRC_OFF){
          if (servos[i].attached()) servos[i].detach();
          pinMode(pinMappings[i], INPUT);
        } else if (ESPRCLink.masterState.mode[i]==ESPRC_SERVO) {
          servos[i].attach(pinMappings[i]);
        } else if (ESPRCLink.masterState.mode[i]>ESPRC_SERVO) {
          if (servos[i].attached()) servos[i].detach();
          pinMode(pinMappings[i], OUTPUT);
        }
      }
      if (ESPRCLink.masterState.mode[i] == ESPRC_SERVO){
        servos[i].write(ESPRCLink.masterState.data[i]);
      } else if (ESPRCLink.masterState.mode[i] == ESPRC_PWM){
        analogWrite(pinMappings[i], ESPRCLink.masterState.data[i]);
      } else if (ESPRCLink.masterState.mode[i] > ESPRC_PWM){
        digitalWrite(pinMappings[i], ESPRCLink.masterState.data[i]);
      }
    }
  }
}

void setup() {
  analogWriteRange(255);
  Serial.begin(115200);
  joystick.ConfigGamepad(D5, D6, D7, D8);
  
  joystick.ReadGamepad();
  delay(100);
  joystick.ReadGamepad();

  off_rx = joystick.Analog(PSS_RX);
  off_ry = joystick.Analog(PSS_RY);
  off_lx = joystick.Analog(PSS_LX);
  off_ly = joystick.Analog(PSS_LY);
  
//  ESPRCLink.init("BE:DD:C2:82:7E:EB", ESPRC_SLAVE);
  ESPRCLink.init("5E:CF:7F:B2:E9:25", ESPRC_MASTER);
  ESPRCLink.on_receive = dataIn;
  Serial.println();
  Serial.println("Hi, I'm "+ESPRCLink.getMac());
  ESPRCLink.setMode(0, ESPRC_PWM);
  ESPRCLink.setMode(1, ESPRC_OFF);
  ESPRCLink.setMode(2, ESPRC_OFF);
  ESPRCLink.setMode(3, ESPRC_OFF);
  ESPRCLink.setMode(4, ESPRC_SERVO);
  ESPRCLink.setMode(5, ESPRC_OFF);
  ESPRCLink.setMode(6, ESPRC_OFF);
  ESPRCLink.setMode(7, ESPRC_OFF);
  ESPRCLink.setMode(8, ESPRC_OFF);
}

void loop() {
  if (ESPRCLink.deviceMode() == ESPRC_SLAVE){
    
  }
  else {
//    Serial.println("LOOP");
    joystick.ReadGamepad();
    if (joystick._buttons == 0xf0ff) {
      off_rx = joystick.Analog(PSS_RX);
      off_ry = joystick.Analog(PSS_RY);
      off_lx = joystick.Analog(PSS_LX);
      off_ly = joystick.Analog(PSS_LY);
    }
//    ESPRCLink.write(0, joystick.Analog(PSS_RY));
    ESPRCLink.write(0, joystick.Analog(PSS_LY));
    ESPRCLink.write(2, joystick.Button(PSB_CROSS));
    ESPRCLink.write(3, joystick.Button(PSB_CIRCLE));
//    ESPRCLink.write(4, joystick.Button(PSB_TRIANGLE));
    ESPRCLink.write(4, 90+(joystick.Analog(PSS_RY)-off_ly)/1.4);
    ESPRCLink.write(5, joystick.Button(PSB_SQUARE));
  }


  /*
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  if (joystick._buttons >> 15 == 0 ) Serial.print(0);
  Serial.println(joystick._buttons, BIN);
  Serial.print(" U : "); Serial.println(joystick.Button(PSB_PAD_UP));
  Serial.print(" D : "); Serial.println(joystick.Button(PSB_PAD_DOWN));
  Serial.print(" L : "); Serial.println(joystick.Button(PSB_PAD_LEFT));
  Serial.print(" R : "); Serial.println(joystick.Button(PSB_PAD_RIGHT));
  Serial.print("/_\\: "); Serial.println(joystick.Button(PSB_TRIANGLE));
  Serial.print(" O : "); Serial.println(joystick.Button(PSB_CIRCLE));
  Serial.print(" X : "); Serial.println(joystick.Button(PSB_CROSS));
  Serial.print("[ ]: "); Serial.println(+joystick.Button(PSB_SQUARE));
  Serial.println(((joystick.Analog(PSS_LX) - off_lx)*(joystick.Analog(PSS_LX) - off_lx))/128);
  Serial.println(((joystick.Analog(PSS_LY) - off_ly)*(joystick.Analog(PSS_LY) - off_ly))/128);
  Serial.println(joystick.Analog(PSS_RX) - off_rx);
  Serial.println(joystick.Analog(PSS_RY) - off_ry);
  */
  delay(10);
}
