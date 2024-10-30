#include <ArduinoJson.h>
#include <BasicStepperDriver.h>
#include <HardwareSerial.h>

// ########################## DEFINES ##########################

#define RPM 120
#define MICROSTEPS 1
#define MOTOR_STEPS 200
#define START_FRAME 0xABCD        // Start frame for reliable serial communication
#define UART1_TX 17               // UART1 TX pin for HoverSerialR
#define UART1_RX 16               // UART1 RX pin for HoverSerialR
#define UART2_TX 27               // UART2 TX pin for HoverSerialL
#define UART2_RX 26               // UART2 RX pin for HoverSerialL
#define UART3_TX 37               // UART3 RX pin for piSerial
#define UART3_RX 36               // UART3 RX pin for piSerial
#define magnetPin 13              // Pin controlling the magnet

// Initialize speed variables, step counts, and magnet state
int torqueF = 0;
int torqueB = 0;
String sliderState;
String elevatorState;
String dumpState;
bool magnetState = false;

// Define step and direction pins for each stepper
const int stepS = 3, dirS = 4;
const int stepE = 5, dirE = 6;
const int stepK = 7, dirK = 8;

// Stepper motor instances
BasicStepperDriver sliderStepper(MOTOR_STEPS, dirS, stepS);
BasicStepperDriver elevatorStepper(MOTOR_STEPS, dirE, stepE);
BasicStepperDriver dumpStepper(MOTOR_STEPS, dirK, stepK);

// Serial communication instances
HardwareSerial HoverSerialR(1);
HardwareSerial HoverSerialL(2);
///////////////////////////////

struct SerialCommand {
    uint16_t start;
    int16_t torqueF;
    int16_t torqueB;
    uint16_t checksum;
};

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

// feedback structs 
SerialFeedback NewFeedback ;
SerialFeedback leftBoardFeedback ;
SerialFeedback rightBoardFeedback   ;

// speed variables 
float leftSpeed ;
float rightSpeed ;
// ########################## FUNCTION TO PARSE JSON ##########################
void parsing(const char* json) {
    StaticJsonDocument<256> data;
    DeserializationError error = deserializeJson(data, json);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
    }
    // Parse speed values for motors
    torqueF = data["torques"][0];
    torqueB = data["torques"][1];


    // Parse actuator states for steppers and magnet
    sliderState = data["actuators"]["slider"].as<int>();
    elevatorState = data["actuators"]["elevator"].as<int>();
    dumpState = data["actuators"]["kick"].as<int>();
    magnetState = data["actuators"]["magnet"].as<bool>();
}

// ########################## FUNCTION TO comma seperated string ##########################
// void parsing(const char* input) {
//     // Convert the input C-string to an Arduino String object for easy manipulation
//     String data = String(input);

//     // Parse each value separated by commas
//     int firstComma = data.indexOf(','); 
//     int secondComma = data.indexOf(',', firstComma + 1);
//     int thirdComma = data.indexOf(',', secondComma + 1);

//     // Extract and convert each substring to the correct type
//     sliderState = data.substring(0, firstComma).toInt();
//     elevatorState = data.substring(firstComma + 1, secondComma).toInt();
//     dumpState = data.substring(secondComma + 1, thirdComma).toInt();
//     magnetState = (data.substring(thirdComma + 1).toInt() != 0);

//     // Debug output to verify the parsing result
//     Serial.print("Slider State: "); Serial.println(sliderState);
//     Serial.print("Elevator State: "); Serial.println(elevatorState);
//     Serial.print("Dump State: "); Serial.println(dumpState);
//     Serial.print("Magnet State: "); Serial.println(magnetState);
// }

// ########################## SPEED CONTROL FUNCTION ##########################
void moveMotors() {
    // Send torque values to HoverSerials
    Send(torqueF, torqueB, HoverSerialR);
    Send(torqueF, torqueB, HoverSerialL);
}

// ########################## ACTUATOR FUNCTION ##########################
void actuator() {
    sliderStepper.rotate(360);
    elevatorStepper.rotate(360);
    dumpStepper.rotate(360);

    if (sliderState == "left") {
        sliderStepper.move(-MOTOR_STEPS * MICROSTEPS);
    } else if (sliderState == "right") {
        sliderStepper.move(MOTOR_STEPS * MICROSTEPS);
    } else {
        sliderStepper.stop();  // "static" or any undefined state stops the slider
    }

    if (elevatorState == "up") {
        elevatorStepper.move(MOTOR_STEPS * MICROSTEPS);
    } else if (elevatorState == "down") {
        elevatorStepper.move(-MOTOR_STEPS * MICROSTEPS);
    } else {
        elevatorStepper.stop();  // "static" or any undefined state stops the elevator
    }

     if (dumpState == "toggleK") {
        dumpStepper.move(MOTOR_STEPS * MICROSTEPS);  // Adjust move as required for "open" or "close"
    }
}

// ########################## FEEDBACK FUNCTION ##########################
void feedback() {
    // Read serial feedback from hoverboards
    // left board feedback
    Receive(HoverSerialL);
    memcpy(&leftBoardFeedback, &NewFeedback, sizeof(SerialFeedback));
    leftSpeed = (leftBoardFeedback.speedL_meas + leftBoardFeedback.speedR_meas)/2 ;
    
    // right board feedback
    Receive(HoverSerialR);
    memcpy(&rightBoardFeedback, &NewFeedback, sizeof(SerialFeedback));
    rightSpeed = (rightBoardFeedback.speedL_meas + rightBoardFeedback.speedR_meas)/2 ;


}

// ########################## METAL GRIPPING FUNCTION ##########################
void metalGripping() {
    digitalWrite(magnetPin, magnetState ? HIGH : LOW);
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
    
    if (HoverSerial.available()) {s
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
        uint16_t checksum = NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ 
                            NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ 
                            NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;
        
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            Serial.print("Speed Left: "); Serial.print(NewFeedback.speedL_meas);
            Serial.print(" Speed Right: "); Serial.println(NewFeedback.speedR_meas);
        }
        idx = 0;
    }
    incomingBytePrev = incomingByte;
}


// ########################## SETUP FUNCTION ##########################
void setup() {
    Serial.begin(115200);
    HoverSerialR.begin(115200, SERIAL_8N1, UART1_TX, UART1_RX);
    HoverSerialL.begin(115200, SERIAL_8N1, UART2_TX, UART2_RX);
    // piSerial.begin(115200, SERIAL_8N1, UART3_TX, UART3_RX);

    pinMode(magnetPin, OUTPUT);

    sliderStepper.begin(RPM, MICROSTEPS);
    elevatorStepper.begin(RPM, MICROSTEPS);
    dumpStepper.begin(RPM, MICROSTEPS);

    Serial.println("Setup complete.");
}

// ########################## LOOP FUNCTION ##########################
void loop() {
    if (Serial.available() > 0) {
        String json = Serial.readString();
        parsing(json.c_str());
        moveMotors();
        actuator();
        feedback();
        metalGripping();
    }
}
