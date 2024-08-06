#ifndef DualShock2_h
#define DualShock2_h

#include "arduino.h"
#include <SPI.h>

#define PAD_SELECT 0x0100
#define PAD_L3 0x0200
#define PAD_R3 0x0400
#define PAD_START 0x0800
#define PAD_UP 0x1000
#define PAD_RIGHT 0x2000
#define PAD_DOWN 0x4000
#define PAD_LEFT 0x8000
#define PAD_L2 0x0001
#define PAD_R2 0x0002
#define PAD_L1 0x0004
#define PAD_R1 0x0008
#define PAD_TRIANGLE 0x0010
#define PAD_O 0x0020
#define PAD_X 0x0040
#define PAD_SQUARE 0x0080

#define JOYSTICK_Lx 0xff000000
#define JOYSTICK_Ly 0x00ff0000
#define JOYSTICK_Rx 0x0000ff00
#define JOYSTICK_Ry 0x000000ff

#define PRESSURE_ENABLE true
#define PRESSURE_DISABLE false
#define ANALOG_ENABLE true
#define ANALOG_DISABLE false

#define DualShock2_CLK_CONTROL 100

class DualShock2{
  public:
    DualShock2(int _CS);
    void configurate(bool ANALOG_MODE, bool ANALOG_LOCKED, bool PRESSURE_MODE);
    void update();
    bool isConnected();
    bool getButtonState(uint16_t button);
    uint8_t getButtonPressure(uint16_t button);
    uint8_t getJoystick(uint32_t joystick);
    void setLargeMotor(uint8_t LMotorValue);
    void setSmallMotor(uint8_t SMotorValue);
    bool isAnalog();
    bool isDigital();
    void getData(uint8_t *buf);
    void buttonsInPolling();
    void mainPolling();
    void setConfigMode(bool CONFIG_MODE);
    void setAnalogMode(bool ANALOG_MODE, bool ANALOG_LOCKED);
    void getMoreStatusInfo();
    void mapMotors(bool motor_1, bool motor_2);
    void setAnalogResponses(uint8_t b1, uint8_t b2, uint8_t b3);
  private:
    void readControllerHeader();
    void readControllerData();
    void transfer();
    void fillCommandData(uint16_t fillingByte);
    
    int CS;

    uint8_t dataHeader[3];
    uint8_t commandHeader[3] = {0x01, 0x42, 0x00};
    uint16_t dataMainTransfer[9];
    uint16_t commandMainTransfer[9];
    uint8_t mainTransferLength = 0;
    
    uint8_t controllerMode;
    
    uint16_t commandMotors = 0x0000;
    bool motorMap_1 = 1;
    bool motorMap_2 = 0;
    bool _isConnected = false;
};

#endif

//Communication format
/*
 * An example exchange that from a dual shock controller when firt plugged in:
 * Controller defaults to digital mode and only transmits the on / off status of the buttons in the 4th and 5th byte.
 * No joystick data, pressure or vibration control capabilities.
 * byte         1     2     3     4     5
 * Command      0x01  0x42  0x00  0x00  0x00
 * Data         0xFF  0x41  0x5A  0xFF  0xFF
 * Explanation  ( -- header -- )  (Command / Mode Dependent Data (2 to 18 mor bytes)
 * Header:
 *  Command:
 *    byte 1 New packets aways start with 0x01 ... 0x81 for memory card?
 *    byte 2 Main command: can either poll controller or configure it.
 *    byte 3 Always 0x00
 *  Data
 *    byte 1 always 0xFF
 *    byte 2 Device mode
 *    byte 3 always 0x5A, this value appears in several non-functional places
 * Command byte 4 can be configure to control either of the motors
 * Command byte 5 can be configure to control either of the motors
 * First two bytes have the digital button states - pressed or not
 * bytes 6 through 9 are the analog joy
 * bytes 10 through 21 are button pressures (0xFF = fully pressed)
 */

//Command Listing Value
/*
 * infoCommand 0x41
 * pollingCommand 0x42
 * configCommand 0x43
 * switchModeCommand 0x44
 * getStatusInfoCommand 0x45
 * getConst1Command 0x46
 * getConst2Command 0x47
 * getConst3Command 0x4c
 * mapBytesCommand 0x4d
 * analogResponseCommand 0x4f
 */

//Button Map Byte.Bit
/*
 * Select 4.0
 * L3 4.1
 * R3 4.2
 * START 4.3
 * UP 4.4
 * RIGHT 4.5
 * DOWN 4.6
 * LEFT 4.7
 * L2 5.0
 * R2 5.1
 * L1 5.2
 * R1 5.3
 * TRIANGLE 5.4
 * O 5.5
 * X 5.6
 * SQUARE 5.7
 */
