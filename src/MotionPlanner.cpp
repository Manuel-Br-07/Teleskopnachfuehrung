#include "MotionPlanner.h"
#include <Arduino.h>
#include "FastAccelStepper.h"
#include <TMCStepper.h>


// PAN-Motor: STEP / DIR / ENABLE:
constexpr int PAN_STEP = 14;
constexpr int PAN_DIR  = 27;
constexpr int PAN_EN   = 26;

// TILT-Motor: STEP / DIR / ENABLE:
constexpr int TILT_STEP = 25;
constexpr int TILT_DIR  = 33;
constexpr int TILT_EN   = 32;

// UART zum TMC2209:
constexpr int UART_RX = 16;
constexpr int UART_TX = 17;

// Treiberadressen:
constexpr float R_SENSE = 0.11f; // Beispielwert: 0,11 Ohm
constexpr uint8_t PAN_DRIVER_ADDRESS = 0b00;
constexpr uint8_t TILT_DRIVER_ADDRESS = 0b01;

// Strom in mA RMS
constexpr uint16_t MOTOR_CURRENT = 600;

// Treiberinitialisierung:
TMC2209Stepper PAN_driver(&Serial2, R_SENSE, PAN_DRIVER_ADDRESS);
TMC2209Stepper TILT_driver(&Serial2, R_SENSE, TILT_DRIVER_ADDRESS);



// Initialisierung der Motoren, Sensoren und internen Zustände.
void MotionPlanner::begin() {
    initializeDrivers();
}

// Nicht blockierende Bewegungsplanung; regelmäßig aus loop() aufrufen.
void MotionPlanner::update() {
    // TODO: Bewegungszustand fortschreiben und Motoren aktualisieren.
}

void MotionPlanner::gotoPosition(float azimuth, float altitude) {
    // TODO: Zielposition planen.
}

void MotionPlanner::setFollow(float rate, bool enabled) {
    // TODO: Nachführung ein- oder ausschalten und Rate setzen.
}

void MotionPlanner::startHome() {
    // TODO: Referenzfahrt starten.
}

void MotionPlanner::setNull(float referenceAzimuth, float referenceAltitude) {
    // TODO: aktuelle Teleskopausrichtung auf Referenzwerte setzen.
}

void MotionPlanner::moveUp(float speed) {
    // TODO: manuelle Bewegung nach oben starten.
}

void MotionPlanner::moveDown(float speed) {
    // TODO: manuelle Bewegung nach unten starten.
}

void MotionPlanner::moveLeft(float speed) {
    // TODO: manuelle Bewegung nach links starten.
}

void MotionPlanner::moveRight(float speed) {
    // TODO: manuelle Bewegung nach rechts starten.
}

void MotionPlanner::stopMotion() {
    // TODO: normale Bewegung kontrolliert anhalten.
}

void MotionPlanner::emergencyStop() {
    // TODO: Motoren sofort sicher anhalten.

    digitalWrite(PAN_EN, HIGH);
    digitalWrite(TILT_EN, HIGH);
}

void MotionPlanner::setSpeed(float speed) {
    // TODO: globale Geschwindigkeit setzen.
}

bool MotionPlanner::isBusy() const {
    // TODO: tatsächlichen Bewegungszustand zurückgeben.
    return false;
}

bool MotionPlanner::isHomed() const {
    // TODO: tatsächlichen Referenzstatus zurückgeben.
    return false;
}

void MotionPlanner::getPosition(float &azimuth, float &altitude) const {
    // TODO: gemessene oder berechnete Position zurückgeben.
    azimuth = 0.0f;
    altitude = 0.0f;
}




// Konfiguriert die gemeinsamen UART-Einstellungen und beide TMC2209-Treiber.
void MotionPlanner::initializeDrivers() {
    pinMode(PAN_STEP, OUTPUT);
    pinMode(PAN_DIR, OUTPUT);
    pinMode(PAN_EN, OUTPUT);
    digitalWrite(PAN_STEP, LOW);
    digitalWrite(PAN_DIR, LOW);
    digitalWrite(PAN_EN, HIGH); // Treiber zunächst deaktiviert

    pinMode(TILT_STEP, OUTPUT);
    pinMode(TILT_DIR, OUTPUT);
    pinMode(TILT_EN, OUTPUT);
    digitalWrite(TILT_STEP, LOW);
    digitalWrite(TILT_DIR, LOW);
    digitalWrite(TILT_EN, HIGH); // Treiber zunächst deaktiviert

    // Beide Treiber teilen sich UART2.
    Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
    delay(500);

    // Für beide Treiber identische TMC2209-Konfiguration.
    PAN_driver.begin();
    PAN_driver.rms_current(MOTOR_CURRENT, 0.5);
    PAN_driver.microsteps(256);
    PAN_driver.intpol(false);
    PAN_driver.en_spreadCycle(false);
    PAN_driver.pwm_autoscale(true);
    PAN_driver.toff(4);
    PAN_driver.blank_time(24);

    TILT_driver.begin();
    TILT_driver.rms_current(MOTOR_CURRENT, 0.5);
    TILT_driver.microsteps(256);
    TILT_driver.intpol(false);
    TILT_driver.en_spreadCycle(false);
    TILT_driver.pwm_autoscale(true);
    TILT_driver.toff(4);
    TILT_driver.blank_time(24);

    digitalWrite(PAN_EN, LOW);
    digitalWrite(TILT_EN, LOW);
}