#pragma once

#include <stdint.h>

/**
 * Schnittstelle f?r die Bewegungsplanung des Teleskops.
 *
 * Die Implementierung der Methoden und die Anbindung an Motoren/Sensoren
 * k?nnen sp?ter erg?nzt werden. Alle Winkel sind in Grad angegeben;
 * die Einheit der Geschwindigkeiten legt die Implementierung fest.
 */
class MotionPlanner {
public:
    void begin();
    void update(); // Nicht blockierende Bewegungslogik regelmäßig aufrufen.

    void gotoPosition(float azimuth, float altitude);
    void setFollow(float rate, bool enabled);
    void startHome();
    void setNull(float referenceAzimuth, float referenceAltitude);

    void moveUp(float speed);
    void moveDown(float speed);
    void moveLeft(float speed);
    void moveRight(float speed);

    void stopMotion();
    void emergencyStop();
    void setSpeed(float speed);

    bool isBusy() const;
    bool isHomed()const;
    bool isNulled()const;
    bool isFollowing() const;
    void getPosition(float &azimuth, float &altitude) const;

    void initializeDrivers(); // Initialisierung der Motoren und Sensoren.
    void initializeSteppers(); // Initialisierung der Schrittmotoren und der Bewegungssteuerung.

    float getCurrentPosPan() ;
    float getCurrentPosTilt() ;

    void homingLogic();
    void gotoLogic();
};
