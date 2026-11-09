#include "MotionPlanner.h"
#include <Arduino.h>
#include "FastAccelStepper.h"
#include <TMCStepper.h>

// PAN-Motor: STEP / DIR / ENABLE:
constexpr int PAN_STEP = 14;
constexpr int PAN_DIR = 27;
constexpr int PAN_EN = 26;

// TILT-Motor: STEP / DIR / ENABLE:
constexpr int TILT_STEP = 25;
constexpr int TILT_DIR = 33;
constexpr int TILT_EN = 32;

// Endstop-Sensoren: PAN / TILT:
constexpr int PAN_endstop1 = 18;
constexpr int PAN_endstop2 = 19;
constexpr int TILT_endstop = 21;

// UART zum TMC2209:
constexpr int UART_RX = 16;
constexpr int UART_TX = 17;

// Treiberadressen:
constexpr float R_SENSE = 0.11f; // Beispielwert: 0,11 Ohm
constexpr uint8_t PAN_DRIVER_ADDRESS = 0b00;
constexpr uint8_t TILT_DRIVER_ADDRESS = 0b01;

// Strom in mA RMS
constexpr uint16_t MOTOR_CURRENT = 600;
constexpr int MICROSTEPS = 256; // Microsteps pro Vollschritt (z.B. 256 für TMC2209)

// Treiberinitialisierung:
TMC2209Stepper PAN_driver(&Serial2, R_SENSE, PAN_DRIVER_ADDRESS);
TMC2209Stepper TILT_driver(&Serial2, R_SENSE, TILT_DRIVER_ADDRESS);

FastAccelStepperEngine stepperEngine;
FastAccelStepper *panStepper = nullptr;
FastAccelStepper *tiltStepper = nullptr;

constexpr int MAX_SPEED = 1000;                                            // Maximalgeschwindigkeit in Schritten pro Sekunde
constexpr int ACCELERATION = 500;                                          // Beschleunigung in Schritten pro Sekunde^2
constexpr float STEP_PER_DEGREE = (200 * MICROSTEPS * 6.6666666667) / 360; // anzahl microsteps(265) * 1,8° * Untersetzung(6,66666) / 360° = Schritte pro Grad
constexpr float MAX_POSITION_PAN = 600.0;                                  // Maximaler Pan-Winkel in Grad
constexpr float MAX_POSITION_TILT = 180.0;                                 // Maximaler Tilt-Winkel in Grad

bool Homed = true;   // Flag, das angibt, ob das Teleskop bereits referenziert wurde (Homing durchgeführt).
int HomingState = 0; // Zustand der Referenzfahrt: 0 = nicht aktiv,1 = aktiv, 11 = schnell nach Endstop 1PAN u. 2PAN, 12 = langsam nach Endstop 1PAN u. 2PAN, 13 = zurück, 21 = schnell nach Endstop TILT, 22 = langsam nach Endstop TILT 23 = zurück
bool Nulled = false; // Flag, das angibt, ob das Teleskop bereits auf Null gesetzt wurde (Referenzposition definiert).

uint16_t currentPosPan = 0;  // Aktuelle Position in Grad
uint16_t currentPosTilt = 0; // Aktuelle Position in Grad

uint16_t nullPosPan = 0;  // Nullposition in Grad
uint16_t nullPosTilt = 0; // Nullposition in Grad

// Initialisierung der Motoren, Sensoren und internen Zustände.
void MotionPlanner::begin()
{
    initializeDrivers();
    initializeSteppers();
}

// Nicht blockierende Bewegungsplanung; regelmäßig aus loop() aufrufen.
void MotionPlanner::update()
{
    // TODO: Bewegungszustand fortschreiben und Motoren aktualisieren.
    homingLogic(); // Referenzfahrt-Logik aufrufen
}

void MotionPlanner::gotoPosition(float azimuth, float altitude)
{
    // TODO: Zielposition planen.
    if (Homed)
    {
    }
}

void MotionPlanner::setFollow(float rate, bool enabled)
{
    // TODO: Nachführung ein- oder ausschalten und Rate setzen.
    if (Homed)
    {
    }
}

void MotionPlanner::startHome()
{
    // TODO: Referenzfahrt starten.
    ::HomingState = 1; // Referenzfahrt starten
}

void MotionPlanner::setNull(float referenceAzimuth, float referenceAltitude)
{
    // TODO: aktuelle Teleskopausrichtung auf Referenzwerte setzen.
    if (Homed)
    {
    }
}

void MotionPlanner::moveUp(float speed)
{
    if (Homed)
    {
        tiltStepper->moveTo(MAX_POSITION_TILT * STEP_PER_DEGREE);
    }
}

void MotionPlanner::moveDown(float speed)
{
    if (Homed)
    {
        tiltStepper->moveTo(0);
    }
}

void MotionPlanner::moveLeft(float speed)
{
    if (Homed)
    {
        panStepper->moveTo(0);
    }
}

void MotionPlanner::moveRight(float speed)
{
    if (Homed)
    {
        panStepper->moveTo(MAX_POSITION_PAN * STEP_PER_DEGREE);
    }
}

void MotionPlanner::stopMotion()
{
    tiltStepper->stopMove();
    panStepper->stopMove();

    currentPosPan = panStepper->getCurrentPosition() / STEP_PER_DEGREE;
    currentPosTilt = tiltStepper->getCurrentPosition() / STEP_PER_DEGREE;
}

void MotionPlanner::emergencyStop()
{
    // TODO: Motoren sofort sicher anhalten.
    tiltStepper->forceStop();
    panStepper->forceStop();

    digitalWrite(PAN_EN, HIGH);
    digitalWrite(TILT_EN, HIGH);

    ::Homed = false;
    ::Nulled = false;
    ::HomingState = 0;
}

void MotionPlanner::setSpeed(float speed)
{
    // TODO: globale Geschwindigkeit setzen.
    tiltStepper->setSpeedInHz(speed * STEP_PER_DEGREE);
    panStepper->setSpeedInHz(speed * STEP_PER_DEGREE);
}

bool MotionPlanner::isBusy() const
{
    // TODO: tatsächlichen Bewegungszustand zurückgeben.
    return false;
}

bool MotionPlanner::isHomed() const
{
    // TODO: tatsächlichen Referenzstatus zurückgeben.
    return ::Homed;
}

bool MotionPlanner::isNulled() const
{
    return ::Nulled;
}

void MotionPlanner::getPosition(float &azimuth, float &altitude) const
{
    // TODO: gemessene oder berechnete Position zurückgeben.
    azimuth = panStepper->getCurrentPosition() / STEP_PER_DEGREE;
    altitude = tiltStepper->getCurrentPosition() / STEP_PER_DEGREE;
}

// Konfiguriert die gemeinsamen UART-Einstellungen und beide TMC2209-Treiber.
void MotionPlanner::initializeDrivers()
{
    pinMode(PAN_STEP, OUTPUT);
    pinMode(PAN_DIR, OUTPUT);
    pinMode(PAN_EN, OUTPUT);
    pinMode(PAN_endstop1, INPUT_PULLUP);
    pinMode(PAN_endstop2, INPUT_PULLUP);
    digitalWrite(PAN_STEP, LOW);
    digitalWrite(PAN_DIR, LOW);
    digitalWrite(PAN_EN, HIGH); // Treiber zunächst deaktiviert

    pinMode(TILT_STEP, OUTPUT);
    pinMode(TILT_DIR, OUTPUT);
    pinMode(TILT_EN, OUTPUT);
    pinMode(TILT_endstop, INPUT_PULLUP);
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

// Initialisiert FastAccelStepper für PAN und TILT.
void MotionPlanner::initializeSteppers()
{
    stepperEngine.init();

    panStepper = stepperEngine.stepperConnectToPin(PAN_STEP);
    if (panStepper != nullptr)
    {
        panStepper->setDirectionPin(PAN_DIR);
        // panStepper->setEnablePin(PAN_EN);
        // panStepper->setAutoEnable(true);
    }

    tiltStepper = stepperEngine.stepperConnectToPin(TILT_STEP);
    if (tiltStepper != nullptr)
    {
        tiltStepper->setDirectionPin(TILT_DIR);
        // tiltStepper->setEnablePin(TILT_EN);
        // tiltStepper->setAutoEnable(true);
    }

    MotionPlanner::setSpeed(5);
    tiltStepper->setAcceleration(10000);
    panStepper->setAcceleration(10000);
}

uint16_t MotionPlanner::getCurrentPosPan()
{
    return panStepper->getCurrentPosition() / STEP_PER_DEGREE;
}

uint16_t MotionPlanner::getCurrentPosTilt()
{
    return tiltStepper->getCurrentPosition() / STEP_PER_DEGREE;
}

void MotionPlanner::homingLogic()
{
    if (::HomingState != 0)
    {
        // Referenzfahrt-Logik implementieren
        switch (::HomingState)
        {
        case 1:
            // Referenzfahrt aktiv, aber noch nicht gestartet
            Serial.println(HomingState);
            if (panStepper->isRunning() || tiltStepper->isRunning())
            {
                // Warten, bis beide Motoren gestoppt sind
                break;
            }

            tiltStepper->setSpeedInHz(5 * STEP_PER_DEGREE);
            panStepper->setSpeedInHz(5 * STEP_PER_DEGREE);


            if (digitalRead(PAN_endstop1) == LOW || digitalRead(PAN_endstop2) == LOW)
            {
                ::HomingState = 13;                        // Von endstop 1PAN u. 2PAN wegfahren
                panStepper->move((30) * STEP_PER_DEGREE); // 30 Grad zurückfahren
            }

            if (digitalRead(TILT_endstop) == LOW)
            {
                ::HomingState = 23;                        // Von endstop TILT wegfahren
                tiltStepper->move(-(30) * STEP_PER_DEGREE); // 30 Grad zurückfahren
            }
            ::HomingState = 11; // Referenzfahrt starten

            break;
        case 11:
            // Schnell zu Endstop PAN fahren
            Serial.println(HomingState);

            break;
        case 12:
            // Langsam zu Endstop PAN fahren
            Serial.println(HomingState);

            break;
        case 13:
            // Von Endstop PAN wegfahren
            Serial.println(HomingState);
            if (digitalRead(PAN_endstop1) == HIGH && digitalRead(PAN_endstop2)   == HIGH)
            {
                ::HomingState = 1;                        // 2. Achse Prüfen
                panStepper->move((2 * STEP_PER_DEGREE)); // 2 Grad weiterfahren
            }

            break;
        case 21:
            // Schnell zu Endstop TILT fahren
            Serial.println(HomingState);

            break;
        case 22:
            // Langsam zu Endstop TILT fahren
            Serial.println(HomingState);

            break;
        case 23:
            // Von Endstop TILT wegfahren
            Serial.println(HomingState);
            if (digitalRead(TILT_endstop)    == HIGH)
            {
                ::HomingState = 1;                      // 2. Achse Prüfen
                tiltStepper->move(-2 * STEP_PER_DEGREE); // 2 Grad weiterfahren
            }
            break;
        }
    }
}