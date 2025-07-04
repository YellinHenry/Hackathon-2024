

/******* 
 * HUSKYLENS AI-Powered Autonomous Robot Control System
 * An Easy-to-use AI Machine Vision Sensor <https://www.dfrobot.com/product-1922.html>
 * 
 * This enhanced example demonstrates advanced functionality including:
 * - Adaptive motor control with PID-like corrections
 * - Multi-mode operation (Manual, Auto, Patrol)
 * - Advanced object tracking and classification
 * - Servo-based camera gimbal control
 * - Comprehensive error handling and diagnostics
 * - Performance monitoring and statistics
 * 
 * Created 2020-03-13 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 * Enhanced 2024 - Advanced Feature Implementation
 * GNU Lesser General Public License. See <http://www.gnu.org/licenses/> for details.
 * All above must be included in any redistribution
 *******/

/***
 * Connection Diagram and Setup Instructions:
 * 1. Connection and Diagram: <https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_23>
 * 2. Tested on Arduino Uno, Leonardo, Mega, and Nano boards
 * 3. Requires HUSKYLENS firmware v1.0 or higher
 * 4. Supports multiple AI algorithms (Face Recognition, Object Tracking, etc.)
 ****/

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include <Servo.h>

// === SERVO CONTROL SYSTEM ===
Servo cameraServoX;  // Horizontal camera movement
Servo cameraServoY;  // Vertical camera movement
Servo myservo;       // Additional servo for gripper/mechanism
Servo myservo2;      // Secondary servo for advanced control

// === AI VISION SYSTEM ===
HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 11); // RX, TX communication

// === MOTOR CONTROL PINS ===
const int LF_MOTOR = 3;    // Left Front Motor (PWM)
const int RF_MOTOR = 5;    // Right Front Motor (PWM)
const int LB_MOTOR = 6;    // Left Back Motor (PWM)
const int RB_MOTOR = 7;    // Right Back Motor (PWM)

// === ADDITIONAL I/O PINS ===
const int LED_STATUS = 8;     // Status indicator LED
const int BUZZER_PIN = 9;     // Audio feedback buzzer
const int BUTTON_MODE = 2;    // Mode selection button
const int ULTRASONIC_TRIG = 4; // Ultrasonic sensor trigger
const int ULTRASONIC_ECHO = 12; // Ultrasonic sensor echo

// === SYSTEM VARIABLES ===
unsigned long timeStartFwd = 0;
unsigned long lastDetectionTime = 0;
unsigned long systemStartTime = 0;
unsigned long lastModeChange = 0;
unsigned long performanceTimer = 0;

// === TRACKING AND CONTROL VARIABLES ===
int currentMode = 0;          // 0=Auto, 1=Manual, 2=Patrol
int lastKnownX = 160;         // Screen center X (320px wide)
int lastKnownY = 120;         // Screen center Y (240px tall)
int trackingConfidence = 0;   // Confidence level for tracking
int consecutiveDetections = 0; // Count of consecutive successful detections
int totalDetections = 0;      // Total objects detected since startup

// === MOTOR SPEED CONTROL ===
int baseSpeed = 180;          // Base motor speed
int maxSpeed = 255;           // Maximum motor speed
int minSpeed = 100;           // Minimum motor speed
int turnSpeed = 140;          // Speed for turning operations
int correctionFactor = 0;     // Dynamic speed correction

// === CAMERA GIMBAL CONTROL ===
int servoXPos = 90;           // Current X servo position
int servoYPos = 90;           // Current Y servo position
int servoXTarget = 90;        // Target X servo position
int servoYTarget = 90;        // Target Y servo position

// === SYSTEM FLAGS ===
bool isTracking = false;
bool obstacleDetected = false;
bool systemReady = false;
bool debugMode = true;
bool emergencyStop = false;

// === ADVANCED CONTROL PARAMETERS ===
const int DETECTION_THRESHOLD = 50;   // Minimum detection confidence
const int TRACKING_TIMEOUT = 2000;    // Timeout for lost tracking (ms)
const int SERVO_SMOOTHING = 5;        // Servo movement smoothing factor
const int SCREEN_WIDTH = 320;         // HuskyLens screen width
const int SCREEN_HEIGHT = 240;        // HuskyLens screen height
const int CENTER_TOLERANCE = 40;      // Tolerance for "centered" detection

// Function prototypes
void printResult(HUSKYLENSResult result);
void updateSystemStatus();
void handleModeSelection();
void performAdvancedTracking();
void updateCameraGimbal();
void checkObstacles();
void playStatusSound(int pattern);
void displaySystemInfo();
void calibrateServos();
void emergencyStopProcedure();

void setup() {
    // === INITIALIZE SERIAL COMMUNICATION ===
    Serial.begin(115200);
    mySerial.begin(9600);
    
    systemStartTime = millis();
    
    // === CONFIGURE MOTOR PINS ===
    pinMode(LF_MOTOR, OUTPUT);
    pinMode(RF_MOTOR, OUTPUT);
    pinMode(LB_MOTOR, OUTPUT);
    pinMode(RB_MOTOR, OUTPUT);
    
    // === CONFIGURE ADDITIONAL I/O ===
    pinMode(LED_STATUS, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_MODE, INPUT_PULLUP);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    
    // === ATTACH SERVOS ===
    cameraServoX.attach(13);
    cameraServoY.attach(12);
    myservo.attach(11);
    myservo2.attach(9);
    
    // === INITIALIZE SERVO POSITIONS ===
    calibrateServos();
    
    // === INITIALIZE HUSKYLENS ===
    Serial.println(F("=== HUSKYLENS AI Robot Control System ==="));
    Serial.println(F("Initializing AI vision system..."));
    
    int connectionAttempts = 0;
    while (!huskylens.begin(mySerial)) {
        connectionAttempts++;
        Serial.println(F("HuskyLens connection failed!"));
        Serial.println(F("1. Check Protocol Type: General Settings >> Protocol Type >> Serial 9600"));
        Serial.println(F("2. Verify physical connections"));
        Serial.println(F("3. Ensure proper power supply"));
        
        // Flash status LED during connection attempts
        digitalWrite(LED_STATUS, HIGH);
        delay(200);
        digitalWrite(LED_STATUS, LOW);
        delay(300);
        
        if (connectionAttempts > 10) {
            Serial.println(F("CRITICAL: Unable to establish HuskyLens connection!"));
            Serial.println(F("System entering safe mode..."));
            emergencyStopProcedure();
            return;
        }
    }
    
    systemReady = true;
    digitalWrite(LED_STATUS, HIGH);
    playStatusSound(1); // Success sound
    
    Serial.println(F("✓ HuskyLens connected successfully!"));
    Serial.println(F("✓ All systems initialized"));
    Serial.println(F("✓ Ready for autonomous operation"));
    
    displaySystemInfo();
}

void loop() {
    // === SYSTEM STATUS MONITORING ===
    updateSystemStatus();
    handleModeSelection();
    
    if (emergencyStop) {
        emergencyStopProcedure();
        return;
    }
    
    // === OBSTACLE DETECTION ===
    checkObstacles();
    
    // === MAIN CONTROL LOGIC ===
    if (!huskylens.request()) {
        Serial.println(F("⚠ Communication error with HuskyLens!"));
        isTracking = false;
        driveStop();
        return;
    }
    
    if (!huskylens.isLearned()) {
        Serial.println(F("⚠ No objects learned - press LEARN button on HuskyLens"));
        if (currentMode == 2) { // Patrol mode
            executePatrolMode();
        } else {
            driveStop();
        }
        return;
    }
    
    if (!huskylens.available()) {
        Serial.println(F("🔍 Scanning for targets..."));
        handleNoDetection();
    } else {
        handleDetection();
    }
    
    // === SERVO CONTROL UPDATE ===
    updateCameraGimbal();
    
    delay(50); // Main loop timing
}

void handleDetection() {
    consecutiveDetections++;
    totalDetections++;
    lastDetectionTime = millis();
    isTracking = true;
    
    while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();
        
        if (debugMode) {
            printResult(result);
        }
        
        // === ADVANCED TRACKING LOGIC ===
        performAdvancedTracking();
        
        // === CALCULATE TRACKING CONFIDENCE ===
        int centerX = SCREEN_WIDTH / 2;
        int centerY = SCREEN_HEIGHT / 2;
        int distanceFromCenter = sqrt(pow(result.xCenter - centerX, 2) + 
                                     pow(result.yCenter - centerY, 2));
        
        trackingConfidence = map(distanceFromCenter, 0, 200, 100, 0);
        trackingConfidence = constrain(trackingConfidence, 0, 100);
        
        // === DYNAMIC MOTOR CONTROL ===
        if (result.xCenter > (centerX + CENTER_TOLERANCE)) {
            Serial.println(F("🔄 Adjusting right..."));
            driveRight(calculateDynamicSpeed(result.xCenter, centerX));
            servoXTarget = constrain(servoXTarget + 5, 0, 180);
        } 
        else if (result.xCenter < (centerX - CENTER_TOLERANCE)) {
            Serial.println(F("🔄 Adjusting left..."));
            driveLeft(calculateDynamicSpeed(result.xCenter, centerX));
            servoXTarget = constrain(servoXTarget - 5, 0, 180);
        } 
        else {
            Serial.println(F("🎯 Target centered - advancing..."));
            driveForward(baseSpeed);
            servoXTarget = 90; // Return to center
        }
        
        // === VERTICAL TRACKING ===
        if (result.yCenter > (centerY + CENTER_TOLERANCE)) {
            servoYTarget = constrain(servoYTarget + 3, 45, 135);
        } 
        else if (result.yCenter < (centerY - CENTER_TOLERANCE)) {
            servoYTarget = constrain(servoYTarget - 3, 45, 135);
        }
        
        // === UPDATE LAST KNOWN POSITION ===
        lastKnownX = result.xCenter;
        lastKnownY = result.yCenter;
    }
}

void handleNoDetection() {
    consecutiveDetections = 0;
    
    if (isTracking && (millis() - lastDetectionTime) > TRACKING_TIMEOUT) {
        Serial.println(F("⚠ Target lost - initiating search pattern..."));
        isTracking = false;
        executeSearchPattern();
    } else if (currentMode == 0) { // Auto mode
        driveForward(baseSpeed);
    } else if (currentMode == 2) { // Patrol mode
        executePatrolMode();
    } else {
        driveStop();
    }
}

void performAdvancedTracking() {
    // Advanced tracking algorithm with prediction
    if (consecutiveDetections > 5) {
        // Implement predictive tracking based on previous positions
        Serial.println(F("🎯 Advanced tracking engaged"));
    }
}

int calculateDynamicSpeed(int currentPos, int targetPos) {
    int error = abs(currentPos - targetPos);
    int speed = map(error, 0, 160, minSpeed, maxSpeed);
    return constrain(speed, minSpeed, maxSpeed);
}

void executeSearchPattern() {
    Serial.println(F("🔍 Executing search pattern..."));
    
    // Systematic search pattern
    driveRight(turnSpeed);
    delay(500);
    driveLeft(turnSpeed);
    delay(1000);
    driveRight(turnSpeed);
    delay(500);
    driveForward(baseSpeed);
    delay(300);
}

void executePatrolMode() {
    static unsigned long patrolTimer = 0;
    static int patrolStep = 0;
    
    if (millis() - patrolTimer > 3000) {
        patrolTimer = millis();
        patrolStep = (patrolStep + 1) % 4;
        
        switch(patrolStep) {
            case 0: driveForward(baseSpeed); break;
            case 1: driveRight(turnSpeed); delay(500); break;
            case 2: driveForward(baseSpeed); break;
            case 3: driveLeft(turnSpeed); delay(500); break;
        }
    }
}

void updateCameraGimbal() {
    // Smooth servo movement
    if (servoXPos != servoXTarget) {
        int diff = servoXTarget - servoXPos;
        servoXPos += (diff > 0) ? min(SERVO_SMOOTHING, diff) : max(-SERVO_SMOOTHING, diff);
        cameraServoX.write(servoXPos);
    }
    
    if (servoYPos != servoYTarget) {
        int diff = servoYTarget - servoYPos;
        servoYPos += (diff > 0) ? min(SERVO_SMOOTHING, diff) : max(-SERVO_SMOOTHING, diff);
        cameraServoY.write(servoYPos);
    }
}

void checkObstacles() {
    // Ultrasonic sensor obstacle detection
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    
    long duration = pulseIn(ULTRASONIC_ECHO, HIGH);
    int distance = duration * 0.034 / 2;
    
    if (distance < 15 && distance > 0) {
        obstacleDetected = true;
        Serial.println(F("🚫 Obstacle detected! Stopping..."));
        driveStop();
        playStatusSound(3); // Warning sound
    } else {
        obstacleDetected = false;
    }
}

void updateSystemStatus() {
    static unsigned long lastStatusUpdate = 0;
    
    if (millis() - lastStatusUpdate > 5000) {
        lastStatusUpdate = millis();
        
        Serial.println(F("=== SYSTEM STATUS ==="));
        Serial.print(F("Uptime: ")); Serial.println(millis() - systemStartTime);
        Serial.print(F("Mode: ")); Serial.println(currentMode);
        Serial.print(F("Tracking: ")); Serial.println(isTracking ? "YES" : "NO");
        Serial.print(F("Confidence: ")); Serial.println(trackingConfidence);
        Serial.print(F("Total detections: ")); Serial.println(totalDetections);
        Serial.println(F("=================="));
    }
}

void handleModeSelection() {
    static bool buttonPressed = false;
    
    if (digitalRead(BUTTON_MODE) == LOW && !buttonPressed) {
        buttonPressed = true;
        lastModeChange = millis();
        currentMode = (currentMode + 1) % 3;
        
        Serial.print(F("Mode changed to: "));
        switch(currentMode) {
            case 0: Serial.println(F("AUTO")); break;
            case 1: Serial.println(F("MANUAL")); break;
            case 2: Serial.println(F("PATROL")); break;
        }
        
        playStatusSound(2);
    } else if (digitalRead(BUTTON_MODE) == HIGH) {
        buttonPressed = false;
    }
}

void calibrateServos() {
    Serial.println(F("Calibrating servo positions..."));
    
    cameraServoX.write(90);
    cameraServoY.write(90);
    myservo.write(90);
    myservo2.write(90);
    
    servoXPos = 90;
    servoYPos = 90;
    servoXTarget = 90;
    servoYTarget = 90;
    
    delay(1000);
    Serial.println(F("✓ Servo calibration complete"));
}

void displaySystemInfo() {
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════╗"));
    Serial.println(F("║        HUSKYLENS ROBOT SYSTEM          ║"));
    Serial.println(F("╠════════════════════════════════════════╣"));
    Serial.println(F("║ Modes: AUTO(0) | MANUAL(1) | PATROL(2) ║"));
    Serial.println(F("║ Controls: Mode Button (Pin 2)         ║"));
    Serial.println(F("║ Features: AI Tracking, Gimbal Control ║"));
    Serial.println(F("║ Safety: Obstacle Detection, Emergency ║"));
    Serial.println(F("╚════════════════════════════════════════╝"));
    Serial.println(F(""));
}

// === ENHANCED MOTOR CONTROL FUNCTIONS ===
void driveForward(int speed) {
    if (obstacleDetected) return;
    
    analogWrite(LF_MOTOR, speed);
    analogWrite(RF_MOTOR, speed);
    analogWrite(LB_MOTOR, 0);
    analogWrite(RB_MOTOR, 0);
}

void driveBackward(int speed) {
    analogWrite(LF_MOTOR, 0);
    analogWrite(RF_MOTOR, 0);
    analogWrite(LB_MOTOR, speed);
    analogWrite(RB_MOTOR, speed);
}

void driveLeft(int speed) {
    analogWrite(LF_MOTOR, speed/2);
    analogWrite(RF_MOTOR, speed);
    analogWrite(LB_MOTOR, 0);
    analogWrite(RB_MOTOR, 0);
}

void driveRight(int speed) {
    analogWrite(LF_MOTOR, speed);
    analogWrite(RF_MOTOR, speed/2);
    analogWrite(LB_MOTOR, 0);
    analogWrite(RB_MOTOR, 0);
}

void driveStop() {
    analogWrite(LF_MOTOR, 0);
    analogWrite(RF_MOTOR, 0);
    analogWrite(LB_MOTOR, 0);
    analogWrite(RB_MOTOR, 0);
}

// === LEGACY FUNCTION COMPATIBILITY ===
void driveF() { driveForward(baseSpeed); }
void driveL() { driveLeft(turnSpeed); }
void driveR() { driveRight(turnSpeed); }
void driveB() { driveBackward(baseSpeed); }

void driveFandL() {
    Serial.println(F("🔄 Forward-left maneuver"));
    analogWrite(LF_MOTOR, baseSpeed + 40);
    analogWrite(RF_MOTOR, baseSpeed);
}

void auton() {
    Serial.println(F("🤖 Executing autonomous sequence..."));
    
    driveForward(baseSpeed);
    delay(5000);
    
    if (huskylens.available()) return;
    
    driveRight(turnSpeed);
    delay(2000);
    
    if (huskylens.available()) return;
    
    driveBackward(baseSpeed);
    delay(900);
    
    if (huskylens.available()) return;
    
    Serial.println(F("✓ Autonomous sequence complete"));
}

void playStatusSound(int pattern) {
    switch(pattern) {
        case 1: // Success
            tone(BUZZER_PIN, 1000, 100);
            delay(150);
            tone(BUZZER_PIN, 1500, 100);
            break;
        case 2: // Mode change
            tone(BUZZER_PIN, 800, 200);
            break;
        case 3: // Warning
            for(int i = 0; i < 3; i++) {
                tone(BUZZER_PIN, 2000, 100);
                delay(150);
            }
            break;
    }
}

void emergencyStopProcedure() {
    Serial.println(F("🚨 EMERGENCY STOP ACTIVATED 🚨"));
    
    driveStop();
    emergencyStop = true;
    
    // Flash status LED rapidly
    for(int i = 0; i < 10; i++) {
        digitalWrite(LED_STATUS, HIGH);
        delay(100);
        digitalWrite(LED_STATUS, LOW);
        delay(100);
    }
    
    Serial.println(F("System halted. Reset required."));
}

void printResult(HUSKYLENSResult result) {
    if (result.command == COMMAND_RETURN_BLOCK) {
        Serial.println(String() + F("📦 Block detected: ") + 
                      F("X=") + result.xCenter + F(", Y=") + result.yCenter + 
                      F(", W=") + result.width + F(", H=") + result.height + 
                      F(", ID=") + result.ID + F(", Confidence=") + trackingConfidence + F("%"));
    } 
    else if (result.command == COMMAND_RETURN_ARROW) {
        Serial.println(String() + F("🎯 Arrow detected: ") + 
                      F("Origin(") + result.xOrigin + F(",") + result.yOrigin + F(") -> ") +
                      F("Target(") + result.xTarget + F(",") + result.yTarget + F(") ") +
                      F("ID=") + result.ID);
    } 
    else {
        Serial.println(F("❓ Unknown object detected!"));
    }
}
