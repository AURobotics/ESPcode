#include <Arduino.h>
#include <ArduinoJson.h>
#include <BasicStepperDriver.h>
#include <HardwareSerial.h>

// #include "BluetoothSerial.h"  // Include BluetoothSerial library for ESP32

// BluetoothSerial SerialBT;     // Create a BluetoothSerial instance

// debugging variables
int counter = 0;
unsigned long t = 0;

// ########################## DEFINES ##########################

#define RPM 120
#define MICROSTEPS 1
#define MOTOR_STEPS 200
#define START_FRAME 0xABCD  // Start frame for reliable serial communication
#define UART1_TX 17         // UART1 TX pin for HoverSerialR
#define UART1_RX 16         // UART1 RX pin for HoverSerialR
#define UART2_TX 25         // UART2 TX pin for HoverSerialL
#define UART2_RX 26         // UART2 RX pin for HoverSerialL
#define magnetPin 13        // Pin controlling the magnet

// Initialize speed variables, step counts, and magnet state
int torqueR = 0;
int torqueL = 0;
String sliderState;
String elevatorState;
String dumpState;
String actuatorData;
String motionData;
bool magnetState = false;

// Define step and direction pins for each stepper
const int stepS = 33, dirS = 32;
const int stepE = 4, dirE = 2;
const int stepK = 14, dirK = 27;

// Stepper motor instances
BasicStepperDriver sliderStepper(MOTOR_STEPS, dirS, stepS);
BasicStepperDriver elevatorStepper(MOTOR_STEPS, dirE, stepE);
BasicStepperDriver dumpStepper(MOTOR_STEPS, dirK, stepK);

// Serial communication instances
HardwareSerial HoverSerialR(1);  // right hoverboard
HardwareSerial HoverSerialL(2);  // left hoverboard
///////////////////////////////

// // struct for serial communication 
// // start frame, torque front, torque back, checksum
struct SerialCommand {
  uint16_t start;
  int16_t torqueF;
  int16_t torqueB;
  uint16_t checksum;
};

// // struct for serial communication
// start frame, cmd1, cmd2, speedR_meas, speedL_meas, batVoltage, boardTemp, cmdLed, checksum
struct SerialFeedback {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  uint16_t batVoltage;
  int16_t boardTemp;
  uint8_t cmdLed;
  uint16_t checksum;
};
void send();
void receive();

// // feedback structs 
SerialFeedback NewFeedback ;        // feedback struct for serial communication
SerialFeedback Feedback ;           // feedback struct for serial communication where if the checksum is correct, the values are copied to this struct
SerialFeedback leftBoardFeedback ;  // feedback struct for left hoverboard
SerialFeedback rightBoardFeedback ; // feedback struct for right hoverboard

// speed variables to return to pi
float leftSpeed;
float rightSpeed;

// battery voltages for each hoverboard
float leftBatteryVoltage;
float rightBatteryVoltage;

// ########################## FUNCTION DECLARATIONS ##########################

// ########################## FUNCTION TO PARSE JSON ##########################
// void parsing(const char* json) {
//     StaticJsonDocument<256> data;
//     DeserializationError error = deserializeJson(data, json);

//     if (error) {
//         Serial.print(F("deserializeJson() failed: "));
//         Serial.println(error.f_str());
//         return;
//     }
//     // Parse speed values for motors
//     torqueF = data["torques"][0];
//     torqueB = data["torques"][1];


//     // Parse actuator states for steppers and magnet
//     sliderState = data["actuators"]["slider"].as<int>();
//     elevatorState = data["actuators"]["elevator"].as<int>();
//     dumpState = data["actuators"]["kick"].as<int>();
//     magnetState = data["actuators"]["magnet"].as<bool>();
// }

// ########################## FUNCTION TO comma seperated string ##########################
/*
void parsing(const char* input) {
    // Convert the input C-string to an Arduino String object for easy manipulation
    String data = String(input);

  // Parse each value separated by commas
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);
  int fourthComma = data.indexOf(',', thirdComma + 1);
  int fifthComma = data.indexOf(',', fourthComma + 1);

  // Extract and convert each substring to the correct type

  torqueR = data.substring(0, firstComma).toInt();
  torqueL = data.substring(firstComma + 1, secondComma).toInt();
  sliderState = data.substring(secondComma + 1, thirdComma).toInt();
  elevatorState = data.substring(thirdComma + 1, fourthComma).toInt();
  dumpState = data.substring(fourthComma + 1, fifthComma).toInt();
  magnetState = (data.substring(fifthComma + 1).toInt() != 0);

  // Debug output to verify the parsing result
  SerialBT.print("Torque Right: ");
  Serial.print(torqueR);
  SerialBT.print(" Torque Left: ");
  Serial.print(torqueL);
  SerialBT.print(" Slider State: ");
  Serial.print(sliderState);
  SerialBT.print(" Elevator State: ");
  Serial.print(elevatorState);
  SerialBT.print(" Dump State: ");
  Serial.print(dumpState);
  SerialBT.print(" Magnet State: ");
  Serial.println(magnetState);
}
*/
void parsing2(const char* input) {
    
   // Convert the input C-string to an Arduino String object for easy manipulation
    String receivedData = String(input);
    // Parse the combined data
    if (receivedData.startsWith("(") && receivedData.endsWith(")")) {
      int separatorIndex = receivedData.indexOf('|');
      if (separatorIndex > 0) {
        // Extract motion and actuator data
        String motionData = receivedData.substring(1, separatorIndex - 1);
        String actuatorData = receivedData.substring(separatorIndex + 2, receivedData.length() - 1);
      }
    }


    // Parse each value separated by commas
    int torquefirstComma = motionData.indexOf(','); 
    // int secondComma = data.indexOf(',', firstComma + 1);
    
    // Extract and convert each substring to the correct type
    torqueR = motionData.substring(0, torquefirstComma).toInt();
    torqueL = motionData.substring(torquefirstComma + 1).toInt();
    
     int actfirstComma = actuatorData.indexOf(','); 
     int actsecondComma = actuatorData.indexOf(',', actfirstComma + 1);
     
    sliderState = actuatorData.substring(0, actfirstComma).toInt();
    elevatorState = actuatorData.substring(actfirstComma + 1, actsecondComma).toInt();
    dumpState = actuatorData.substring(actsecondComma + 1).toInt();
    Serial.print("Right Torque: ");
    Serial.println(torqueR);
    
    Serial.print("Left Torque: ");
    Serial.println(torqueL);
    
    Serial.print("Slider State: ");
    Serial.println(sliderState);
    
    Serial.print("Elevator State: ");
    Serial.println(elevatorState);
    
    Serial.print("Dump State: ");
    Serial.println(dumpState);
}

// ########################## SPEED CONTROL FUNCTION ##########################
void moveMotors() {
  // Send torque values to HoverSerials
  Send(-100, 100, HoverSerialR);
  Send(100, 100, HoverSerialL);
}

// ########################## ACTUATOR FUNCTION ##########################
void actuator() {
  int sliderStep = 0;
  int elevatorStep = 0;
  int disStep = 0;

  if (sliderState == "left") {
    sliderStep = 1;
    elevatorStep = 0;
    disStep = 0;

  } else if (sliderState == "right") {
    sliderStep = -1;
    elevatorStep = 0;
    disStep = 0;
  } else {
    sliderStep = -1;
    elevatorStep = 0;
    disStep = 0;
  }

  if (elevatorState == "up") {
    sliderStep = 0;
    elevatorStep = 1;
    disStep = 0;
  } else if (elevatorState == "down") {
    sliderStep = 0;
    elevatorStep = -1;
    disStep = 0;
  } else {
    sliderStep = 0;
    elevatorStep = 0;
    disStep = 0;
  }

  //  if (dumpState == "toggleK") {
  //     dumpStepper.move(MOTOR_STEPS * MICROSTEPS);  // Adjust move as required for "open" or "close"
  // }

  // dumpStepper.move(disStep*MOTOR_STEPS*MICROSTEPS);
  elevatorStepper.move(elevatorStep * MOTOR_STEPS * MICROSTEPS);
  sliderStepper.move(sliderStep * MOTOR_STEPS * MICROSTEPS);
  // }
}
// ########################## FEEDBACK FUNCTION ##########################
void feedback() {
    // Read serial feedback from hoverboards
    // left board feedback
    Receive(HoverSerialL);
    memcpy(&leftBoardFeedback, &Feedback, sizeof(SerialFeedback));
    leftSpeed = (leftBoardFeedback.speedL_meas + leftBoardFeedback.speedR_meas)/2 ;
    leftBatteryVoltage = leftBoardFeedback.batVoltage ;
    
    // right board feedback
    Receive(HoverSerialR);
    memcpy(&rightBoardFeedback, &Feedback, sizeof(SerialFeedback));
    rightSpeed = (rightBoardFeedback.speedL_meas + rightBoardFeedback.speedR_meas)/2 ;
    rightBatteryVoltage = rightBoardFeedback.batVoltage ;
    /*
    // Debug output to verify the feedback values
    SerialBT.print("Left Speed: "); Serial.print(leftSpeed);
    SerialBT.print(" Right Speed: "); Serial.print(rightSpeed);
    SerialBT.print(" Left Battery Voltage: "); Serial.print(leftBatteryVoltage);
    SerialBT.print(" Right Battery Voltage: "); Serial.println(rightBatteryVoltage);

    // Send feedback values to PI
    Serial.print("Speed Left: "); Serial.print(leftSpeed);
    Serial.print(" Speed Right: "); Serial.print(rightSpeed);
    Serial.print(" Battery Voltage Left: "); Serial.print(leftBatteryVoltage);
    Serial.print(" Battery Voltage Right: "); Serial.println(rightBatteryVoltage);
    */

}

// ########################## METAL GRIPPING FUNCTION ##########################
void metalGripping() {
    // Control the magnet
    digitalWrite(magnetPin, magnetState ? HIGH : LOW);
    // Debug output to verify the magnet state
    //SerialBT.print("Magnet State: ");
    //Serial.println(magnetState);
}

// ########################## COMMAND SEND FUNCTION ##########################

void Send(int16_t utorqueF, int16_t utorqueB, HardwareSerial &HoverSerial) {
  SerialCommand Command;
  Command.start = START_FRAME;
  Command.torqueF = utorqueF;
  Command.torqueB = utorqueB;
  Command.checksum = Command.start ^ Command.torqueF ^ Command.torqueB;

  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE FUNCTION ##########################

void Receive(HardwareSerial &HoverSerial) {
  static uint8_t idx = 0;
  static byte *p;
  byte incomingByte, incomingBytePrev;
  uint16_t bufStartFrame;
  SerialFeedback NewFeedback;  // Declare NewFeedback as SerialFeedback structure

  if (HoverSerial.available()) {
    incomingByte = HoverSerial.read();
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;
  } else return;

  if (bufStartFrame == START_FRAME) {
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
    *p++ = incomingByte;
    idx++;
  }

  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum = NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;

    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
    }
    idx = 0;
  }
  incomingBytePrev = incomingByte;
}


// ########################## SETUP FUNCTION ##########################
void setup() {
    // Start the serial communication with PI
    Serial.begin(115200);               
    // Start the serial communication with Hoverboards
    HoverSerialR.begin(115200, SERIAL_8N1, UART1_TX, UART1_RX);
    HoverSerialL.begin(115200, SERIAL_8N1, UART2_TX, UART2_RX);
    // Start the Bluetooth communication for debugging purposes
    //SerialBT.begin("ESP32_Bluetooth");  

  pinMode(magnetPin, OUTPUT);  // Set the magnet pin as an output

  sliderStepper.begin(RPM, MICROSTEPS);
  elevatorStepper.begin(RPM, MICROSTEPS);
  dumpStepper.begin(RPM, MICROSTEPS);

    Serial.println("Setup complete."); 
    delay(1000);
    //SerialBT.println(" I am working from esp ");

    Serial.setTimeout(80);
}

// ########################## LOOP FUNCTION ##########################
void loop() {
  // sending data to hoverboards and receiving feedback
  if (millis() > t + 100) {
    moveMotors();
    feedback();
    t = millis();
  }
  // parsing data from pi
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parsing2(input.c_str());
  }
  // metal gripping and actuators
}
