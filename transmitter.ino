#define RF_CE_PIN 4
#define RF_CSN_PIN 3

#define LED_PIN 9

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "symatx.h"
#include "DualShock2.h"

RF24 radio(RF_CE_PIN, RF_CSN_PIN); //CE, CSN
SymaTX tx;
byte packet[11];
byte addr[] = {0x80, 0x80, 0x80, 0x80, 0xa2};
int8_t temp;
byte temp2;

DualShock2 controller(2);

uint8_t controller_data[21];

bool select_btn, start_btn;
bool square_btn, o_btn, x_btn, triangle_btn;
bool R1, R2, R3, L1, L2, L3;
bool up, down, left, right;
uint8_t Lx, Ly, Rx, Ry;

uint8_t elevator_trim = 31;
uint8_t rudder_trim = 31;
uint8_t aileron_trim = 31;

struct DataStruct{
  byte throttle;
  byte elevator;
  byte rudder;
  byte aileron;
  byte data4;
  byte data5;
  byte data6;
  byte data7;
  byte data8;
//  byte data9;
};

DataStruct rf_data;

String inputString = "";
bool stringComplete = false;

class ToggleButton{
  public:
    bool state = false;
    bool change_state_flag = false;
    void update(bool button_state){
      if(state){
        if(change_state_flag){
          if(button_state){
            change_state_flag = false;
          }
        } else{
          if(!button_state){
            state = false;
            change_state_flag = true;
          }
        }
      } else{
        if(change_state_flag){
          if(button_state){
            change_state_flag = false;
          }
        } else{
          if(!button_state){
            state = true;
            change_state_flag = true;
          }
        }
      }
    }
};

ToggleButton video_on_off_toggle;
ToggleButton acrobatic_mode_toggle;

class PressButton{
  public:
    bool state = false;
    bool change_state_flag = false;
    void update(bool button_state){
      if(state){
        state = false;
        if(button_state){
          change_state_flag = false;
        }
      } else{
        if(change_state_flag){
          if(button_state){
            change_state_flag = false;
          }
        } else{
          if(!button_state){
            state = true;
            change_state_flag = true;
          }
        }
      }
    }
};

PressButton R1_press;
PressButton L1_press;
PressButton up_press;
PressButton down_press;
PressButton left_press;
PressButton right_press;

void setup() {
  Serial.begin(115200);
  printf_begin();
  
  SPI.begin();
  pinMode(12, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);

  pinMode(RF_CE_PIN, OUTPUT);
  pinMode(RF_CSN_PIN, OUTPUT);
  
  digitalWrite(RF_CSN_PIN, HIGH);
  controller.update();
  controller.configurate(true, false, false);
  digitalWrite(RF_CSN_PIN, LOW);
  
  if (!radio.begin()) {
    Serial.println(F("radio hardware not responding!"));
    while (1) {} // hold program in infinite loop to prevent subsequent errors
  }
  Serial.println("Radio hardware initialized");
  radio.setAutoAck(false);
  radio.setAddressWidth(5);
  radio.setRetries(15, 15);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setPayloadSize(10);
  delay(12);

  Serial.println("Printing radio details");
  radio.printDetails();
  Serial.println("Done");

  Serial.println("Press Start to bind");
}

void loop() {
  digitalWrite(RF_CSN_PIN, HIGH);
  controller.update();
  digitalWrite(RF_CSN_PIN, LOW);
  if(controller.isConnected()){
    controller.getData(controller_data);
    parse_controller_data();
//    print_controller_data();
  } else{
    Serial.println("Controller disconnected");
    select_btn=false; start_btn=false;
    square_btn=false; o_btn=false; x_btn=false; triangle_btn=false;
    R1=false; R2=false; R3=false; L1=false; L2=false; L3=false;
    up = false; down = false; left = false; right = false;
    Lx = 0; Ly = 0; Rx = 0; Ry = 0;
  }
  
  if (stringComplete) {
    Serial.println("Parsing input");
    inputString = "";
    stringComplete = false;
  }

  
  if (tx.bound) {
//      for (int i=0; i<4; i++) {
//        tx.direction[i] = packet[i];        
//      }
//      tx.build_packet();
    write_packet();
    print_packet();
    tx.transmit((byte*)&rf_data, radio);
  } else{
    if(!start_btn){
      Serial.println("Binding");
      tx.init(addr, radio);
      Serial.println("Binded");
    }
  }
}

void write_packet(){
  video_on_off_toggle.update(o_btn);
  acrobatic_mode_toggle.update(R2);
  
  acrobatic_mode_toggle.state ? digitalWrite(LED_PIN, HIGH) : digitalWrite(LED_PIN, LOW);
  
  rf_data.data4 = 0x00;
  rf_data.data4 |= video_on_off_toggle.state ? 0x80 : 0x00;
  rf_data.data4 |= !triangle_btn ? 0x40 : 0x00;

  uint8_t d;
  
  rf_data.data5 = 0x00;
  rf_data.data5 |= acrobatic_mode_toggle.state ? 0x80 : 0x00;
  d = elevator_trim;
  rf_data.data5 |= d < 32 ? (0x20 | (31 - d)) : (d - 32);
  
  rf_data.data6 = 0x00;
  rf_data.data6 |= !square_btn ? 0x40 : 0x00;
  d = rudder_trim;
  rf_data.data6 |= d < 32 ? (0x20 | (31 - d)) : (d - 32);

  rf_data.data7 = 0x00;
  d = aileron_trim;
  rf_data.data7 |= d < 32 ? (0x20 | (31 - d)) : (d - 32);

  rf_data.data8 = 0x00;
  
//  rf_data.data9 = get_checksum();
}

void print_controller_data(){
  if(select_btn) Serial.print("SEL ");
  if(L3) Serial.print("L3 ");
  if(R3) Serial.print("R3 ");
  if(start_btn) Serial.print("START ");
  if(up) Serial.print("UP ");
  if(right) Serial.print("RIGHT ");
  if(down) Serial.print("DOWN ");
  if(left) Serial.print("LEFT ");
  
  if(L2) Serial.print("L2 ");
  if(R2) Serial.print("R2 ");
  if(L1) Serial.print("L1 ");
  if(R1) Serial.print("R1 ");
  if(triangle_btn) Serial.print("TRI ");
  if(o_btn) Serial.print("O ");
  if(x_btn) Serial.print("X ");
  if(square_btn) Serial.print("SQU ");

  Serial.print(Lx);
  Serial.print(" ");
  Serial.print(Ly);
  Serial.print(" ");
  Serial.print(Rx);
  Serial.print(" ");
  Serial.print(Ry);
  Serial.println();
}

void parse_controller_data(){
  select_btn = bitRead(controller_data[3], 0);
  L3 = bitRead(controller_data[3], 1);
  R3 = bitRead(controller_data[3], 2);
  start_btn = bitRead(controller_data[3], 3);
  up = bitRead(controller_data[3], 4);
  right = bitRead(controller_data[3], 5);
  down = bitRead(controller_data[3], 6);
  left = bitRead(controller_data[3], 7);
  
  L2 = bitRead(controller_data[4], 0);
  R2 = bitRead(controller_data[4], 1);
  L1 = bitRead(controller_data[4], 2);
  R1 = bitRead(controller_data[4], 3);
  triangle_btn = bitRead(controller_data[4], 4);
  o_btn = bitRead(controller_data[4], 5);
  x_btn = bitRead(controller_data[4], 6);
  square_btn = bitRead(controller_data[4], 7);
  
  Lx = controller_data[5];
  Ly = controller_data[6];
  Rx = controller_data[7];
  Ry = controller_data[8];

  R1_press.update(R1);
  L1_press.update(L1);
  up_press.update(up);
  down_press.update(down);
  left_press.update(left);
  right_press.update(right);

  if(Ry < 64){
    if(rf_data.throttle < 255){
      rf_data.throttle++;
    }
  } else if(Ry > 192){
    if(rf_data.throttle > 0){
      rf_data.throttle--;
    }
  }
  if(up_press.state && !down_press.state){
    if(elevator_trim < 63){
      elevator_trim++;
    }
  } else if(down_press.state && !up_press.state){
    if(elevator_trim > 0){
      elevator_trim--;
    }
  }
  if(left_press.state && !right_press.state){
    if(rudder_trim < 63){
      rudder_trim++;
    }
  } else if(right_press.state && !left_press.state){
    if(rudder_trim > 0){
      rudder_trim--;
    }
  }
  if(L1_press.state && !R1_press.state){
    if(aileron_trim < 63){
      aileron_trim++;
    }
  } else if(R1_press.state && !L1_press.state){
    if(aileron_trim > 0){
      aileron_trim--;
    }
  }
  rf_data.elevator = Ly > 127 ? (0x80 | (Ly - 128)) : (127 - Ly);
  rf_data.aileron = Lx > 127 ? (0x80 | (Lx - 128)) : (127 - Lx);
  rf_data.rudder = Rx > 127 ? (0x80 | (Rx - 128)) : (127 - Rx);

  if(!x_btn){
    rf_data.throttle = 0;
  }
}

void print_packet(){
  Serial.print("Tr ");
  Serial.print(rf_data.throttle);
  Serial.print(" El ");
  Serial.print(rf_data.elevator);
  Serial.print(" Ru ");
  Serial.print(rf_data.rudder);
  Serial.print(" Ai ");
  Serial.print(rf_data.aileron);
  Serial.print(" ElTrim ");
  Serial.print(rf_data.data5&0x1f);
  Serial.print(" RuTrim ");
  Serial.print(rf_data.data6&0x1f);
  Serial.print(" AiTrim ");
  Serial.print(rf_data.data7&0x1f);
  Serial.print(" ");
  bitRead(rf_data.data5, 7) ? Serial.print("Acrobatic ") : 1;
  bitRead(rf_data.data6, 6) ? Serial.print("Rotate ") : 1;
  bitRead(rf_data.data4, 7) ? Serial.print("Video On ") : Serial.print("Video Off ");
  bitRead(rf_data.data4, 6) ? Serial.print("Take picture ") : 1;
  Serial.println();
}

void serialEvent(){
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
