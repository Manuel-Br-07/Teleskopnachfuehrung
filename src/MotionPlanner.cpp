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
constexpr float PAN_DRIVER_ADDRESS = 0b00;
constexpr float TILT_DRIVER_ADDRESS = 0b01;

// Strom in mA RMS
constexpr float MOTOR_CURRENT = 600;
constexpr int MICROSTEPS = 256; // Microsteps pro Vollschritt (z.B. 256 für TMC2209)

// Treiberinitialisierung:
TMC2209Stepper PAN_driver(&Serial2, R_SENSE, PAN_DRIVER_ADDRESS);
TMC2209Stepper TILT_driver(&Serial2, R_SENSE, TILT_DRIVER_ADDRESS);

FastAccelStepperEngine stepperEngine;
FastAccelStepper *panStepper = nullptr;
FastAccelStepper *tiltStepper = nullptr;

constexpr int MAX_SPEED = 12;                                              // Maximalgeschwindigkeit in Grad pro Sekunde
constexpr float STEP_PER_DEGREE = (200 * MICROSTEPS * 6.6666666667) / 360; // anzahl microsteps(265) * 1,8° * Untersetzung(6,66666) / 360° = Schritte pro Grad
constexpr float MIN_POSITION_PAN = -256.33;                                // Minimaler Pan-Winkel in Grad
constexpr float MAX_POSITION_PAN = 659.23 + MIN_POSITION_PAN;              // Maximaler Pan-Winkel in Grad
constexpr float MAX_POSITION_TILT = 228.97;                                // Maximaler Tilt-Winkel in Grad
constexpr float MIN_POSITION_TILT = -(280.16 - MAX_POSITION_TILT);         // Minimaler Tilt-Winkel in Grad

bool Homed = false;   // Flag, das angibt, ob das Teleskop bereits referenziert wurde (Homing durchgeführt).
int HomingState = 0; // Zustand der Referenzfahrt: 0 = Homing abgeschlossen; 1 = Initialisierung; 2 = PAN-Endstop freifahren, warten; 3 = TILT-Endstop freifahren, warten; 10–14 = PAN-Referenzfahrt; 20–24 = TILT-Referenzfahrt; 30 = Referenzfahrt abgeschlossen, Position auf Null setzen.
bool Nulled = false; // Flag, das angibt, ob das Teleskop bereits auf Null gesetzt wurde (Referenzposition definiert).
bool GotoActive = false; // Flag, das angibt, ob eine gotoPosition()-Bewegung aktiv ist.
bool FollowActive = false; // Flag, das angibt, ob die Nachführung aktiv ist.

float nullPosPan = 0;  // Nullposition in Grad
float nullPosTilt = 0; // Nullposition in Grad

float speed = 5;        // Geschwindigkeit in Grad pro Sekunde
float acceleration = 5; // Beschleunigung in Grad pro Sekunde^2

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
    // Serial.println("PAN: " + String(digitalRead(PAN_endstop1)) + " " + String(digitalRead(PAN_endstop2)) + " TILT: " + String(digitalRead(TILT_endstop)));
    gotoLogic(); // GOTO-Logik aufrufen (kontrolliert die Bewegung und resettet die Flags, wenn die Bewegung abgeschlossen ist)
    Serial.println("PAN: " + String(getCurrentPosPan()) + " TILT: " + String(getCurrentPosTilt()));
}

void MotionPlanner::gotoPosition(float azimuth, float altitude)
{
    // TODO: Zielposition planen.
    if (Homed)
    {
        // ==========================================
    // 1. Voraussetzungen prüfen
    // ==========================================

    if (GotoActive ||
        panStepper->isRunning() ||
        tiltStepper->isRunning())
    {
        return;
    }

    if (speed <= 0 || acceleration <= 0)
    {
        Serial.println("GOTO: Ungueltige Geschwindigkeit!");
        return;
    }

    // ==========================================
    // 2. Zielpositionen überprüfen
    // ==========================================

    if (azimuth < MIN_POSITION_PAN ||
        azimuth > MAX_POSITION_PAN ||
        altitude < MIN_POSITION_TILT ||
        altitude > MAX_POSITION_TILT)
    {
        Serial.println("GOTO: Zielposition ausserhalb der Grenzen!");
        return;
    }

    // ==========================================
    // 3. Aktuelle und Zielpositionen in Steps
    // ==========================================

    int32_t currentPAN = panStepper->getCurrentPosition();
    int32_t currentTILT = tiltStepper->getCurrentPosition();

    int32_t targetPAN = lroundf(azimuth * STEP_PER_DEGREE);
    int32_t targetTILT = lroundf(altitude * STEP_PER_DEGREE);

    // ==========================================
    // 4. Bewegungsstrecken berechnen
    // ==========================================

    int32_t distancePAN = abs(targetPAN - currentPAN);
    int32_t distanceTILT = abs(targetTILT - currentTILT);

    int32_t maxDistance = max(distancePAN, distanceTILT);

    if (maxDistance == 0)
        return;

    // ==========================================
    // 5. Verhältnis der Bewegungsstrecken
    // ==========================================

    float factorPAN = (float)distancePAN / maxDistance;
    float factorTILT = (float)distanceTILT / maxDistance;

    // ==========================================
    // 6. Geschwindigkeit und Beschleunigung
    // ==========================================

    float gotoSpeed = min(speed, (float)MAX_SPEED);

    uint32_t speedPAN = max(1L, lroundf(
        gotoSpeed * STEP_PER_DEGREE * factorPAN
    ));

    uint32_t speedTILT = max(1L, lroundf(
        gotoSpeed * STEP_PER_DEGREE * factorTILT
    ));

    uint32_t accelPAN = max(1L, lroundf(
        acceleration * STEP_PER_DEGREE * factorPAN
    ));

    uint32_t accelTILT = max(1L, lroundf(
        acceleration * STEP_PER_DEGREE * factorTILT
    ));

    // ==========================================
    // 7. Stepper konfigurieren
    // ==========================================

    if (distancePAN > 0)
    {
        panStepper->setAcceleration(accelPAN);
        panStepper->setSpeedInHz(speedPAN);
    }

    if (distanceTILT > 0)
    {
        tiltStepper->setAcceleration(accelTILT);
        tiltStepper->setSpeedInHz(speedTILT);
    }

    // ==========================================
    // 8. Bewegung starten
    // ==========================================

    GotoActive = true;

    if (distancePAN > 0)
        panStepper->moveTo(targetPAN);

    if (distanceTILT > 0)
        tiltStepper->moveTo(targetTILT);

    return;
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
        tiltStepper->moveTo(MIN_POSITION_TILT * STEP_PER_DEGREE);
    }
}

void MotionPlanner::moveLeft(float speed)
{
    if (Homed)
    {
        panStepper->moveTo(MIN_POSITION_PAN * STEP_PER_DEGREE);
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
    if (speed < MAX_SPEED)
    {
        tiltStepper->setSpeedInHz(speed * STEP_PER_DEGREE);
        panStepper->setSpeedInHz(speed * STEP_PER_DEGREE);
        ::speed = speed;
    }
}

bool MotionPlanner::isBusy() const
{
    // TODO: tatsächlichen Bewegungszustand zurückgeben.
    return panStepper->isRunning() || tiltStepper->isRunning();
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

bool MotionPlanner::isFollowing() const
{
    return false; // TODO: tatsächlichen Nachführstatus zurückgeben.
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
    tiltStepper->setAcceleration(acceleration * STEP_PER_DEGREE);
    panStepper->setAcceleration(acceleration * STEP_PER_DEGREE);
}

float MotionPlanner::getCurrentPosPan()
{
    return panStepper->getCurrentPosition() / STEP_PER_DEGREE;
}

float MotionPlanner::getCurrentPosTilt()
{
    return tiltStepper->getCurrentPosition() / STEP_PER_DEGREE;
}

void MotionPlanner::homingLogic()
{
    float freifahrwinkel = 10; // in Grad
    float langsam = 2;         // in Grad/s
    float schnell_pan = 5;     // in Grad/s
    float schnell_tilt = 10;   // in Grad/s

    if (::HomingState == 0)
        return;

    switch (::HomingState)
    {
        // ==========================================
        // INITIALISIERUNG
        // ==========================================

    case 1:

        ::Homed = false;

        digitalWrite(PAN_EN, LOW);
        digitalWrite(TILT_EN, LOW);

        panStepper->setSpeedInHz(5 * STEP_PER_DEGREE);
        tiltStepper->setSpeedInHz(5 * STEP_PER_DEGREE);

        // PAN-Endstop bereits gedrückt?
        if (digitalRead(PAN_endstop2) == LOW)
        {
            panStepper->move(freifahrwinkel * STEP_PER_DEGREE);
            ::HomingState = 2;
            break;
        }

        // TILT-Endstop bereits gedrückt?
        if (digitalRead(TILT_endstop) == LOW)
        {
            tiltStepper->move(-freifahrwinkel * STEP_PER_DEGREE);
            ::HomingState = 3;
            break;
        }

        // PAN-Referenzfahrt starten
        ::HomingState = 10;
        panStepper->setSpeedInHz(schnell_pan * STEP_PER_DEGREE);
        panStepper->move(-(MAX_POSITION_PAN + 10) * STEP_PER_DEGREE);

        break;

        // ==========================================
        // INITIAL: PAN-ENDSTOP FREIFAHREN
        // ==========================================

    case 2:

        if (!panStepper->isRunning())
        {
            // Anschließend TILT-Endstop prüfen
            if (digitalRead(TILT_endstop) == LOW)
            {
                tiltStepper->move(-10 * STEP_PER_DEGREE);
                ::HomingState = 3;
            }
            else
            {
                ::HomingState = 10;
                panStepper->setSpeedInHz(schnell_pan * STEP_PER_DEGREE);
                panStepper->move(-(MAX_POSITION_PAN + 10) * STEP_PER_DEGREE);
            }
        }

        break;

        // ==========================================
        // INITIAL: TILT-ENDSTOP FREIFAHREN
        // ==========================================

    case 3:

        if (!tiltStepper->isRunning())
        {
            ::HomingState = 10;
            panStepper->move(-(MAX_POSITION_PAN + 10) * STEP_PER_DEGREE);
        }

        break;

        // ==========================================
        // PAN: SCHNELLE REFERENZFAHRT
        // ==========================================

    case 10:

        if (digitalRead(PAN_endstop1) == LOW &&
            digitalRead(PAN_endstop2) == LOW)
        {
            panStepper->forceStop();
            ::HomingState = 11;
        }

        break;

    // Warten, bis PAN tatsächlich steht
    case 11:

        if (!panStepper->isRunning())
        {
            panStepper->setSpeedInHz(langsam * STEP_PER_DEGREE);
            panStepper->move(2 * STEP_PER_DEGREE);
            ::HomingState = 12;
        }

        break;

    // PAN: 2 Grad zurückfahren
    case 12:

        if (!panStepper->isRunning())
        {
            panStepper->move(-10 * STEP_PER_DEGREE);
            ::HomingState = 13;
        }

        break;

    // PAN: Langsame Referenzfahrt
    case 13:

        if (digitalRead(PAN_endstop1) == LOW &&
            digitalRead(PAN_endstop2) == LOW)
        {
            panStepper->forceStop();
            ::HomingState = 14;
        }

        break;

    // PAN: Stillstand abwarten und Nullpunkt setzen
    case 14:

        if (!panStepper->isRunning())
        {
            panStepper->setSpeedInHz(schnell_pan * STEP_PER_DEGREE);
            panStepper->setCurrentPosition(MIN_POSITION_PAN * STEP_PER_DEGREE);
            panStepper->moveTo(0);

            // TILT-Referenzfahrt starten
            tiltStepper->setSpeedInHz(schnell_tilt * STEP_PER_DEGREE);

            tiltStepper->move(
                (MAX_POSITION_TILT + 10) * STEP_PER_DEGREE);

            ::HomingState = 20;
        }

        break;

        // ==========================================
        // TILT: SCHNELLE REFERENZFAHRT
        // ==========================================

    case 20:

        if (digitalRead(TILT_endstop) == LOW)
        {
            tiltStepper->forceStop();
            ::HomingState = 21;
        }

        break;

    // Warten, bis TILT tatsächlich steht
    case 21:

        if (!tiltStepper->isRunning())
        {
            tiltStepper->setSpeedInHz(langsam * STEP_PER_DEGREE);
            tiltStepper->move(-2 * STEP_PER_DEGREE);
            ::HomingState = 22;
        }

        break;

    // TILT: 2 Grad zurückfahren
    case 22:

        if (!tiltStepper->isRunning())
        {
            tiltStepper->move(10 * STEP_PER_DEGREE);
            ::HomingState = 23;
        }

        break;

    // TILT: Langsame Referenzfahrt
    case 23:

        if (digitalRead(TILT_endstop) == LOW)
        {
            tiltStepper->forceStop();
            ::HomingState = 24;
        }

        break;

    // TILT: Stillstand abwarten und Nullpunkt setzen
    case 24:

        if (!tiltStepper->isRunning())
        {
            tiltStepper->setSpeedInHz(schnell_tilt * STEP_PER_DEGREE);
            tiltStepper->setCurrentPosition(MAX_POSITION_TILT * STEP_PER_DEGREE);
            tiltStepper->moveTo(0);

            ::HomingState = 30;

            break;
        }

        // ==========================================
        // HOMING ABGESCHLOSSEN
        // ==========================================

    case 30:

        if (!(tiltStepper->isRunning() || panStepper->isRunning()))
        {
            ::HomingState = 0;
            ::Homed = true;

            panStepper->setSpeedInHz(::speed * STEP_PER_DEGREE);
            tiltStepper->setSpeedInHz(::speed * STEP_PER_DEGREE);

            Serial.println("Homing abgeschlossen!");
        }
    }
}



void MotionPlanner::gotoLogic()
{
    if (!GotoActive)
        return;

    if (!panStepper->isRunning() &&
        !tiltStepper->isRunning())
    {
        GotoActive = false;

        Serial.println("GOTO abgeschlossen!");
    }
}
