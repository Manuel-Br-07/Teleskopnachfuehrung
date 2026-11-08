#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "MotionPlanner.h"

/*
 * Nicht blockierender Serial-Interpreter für das Teleskop-Protokoll.
 *
 * Format: genau ein Befehl pro Zeile, abgeschlossen mit LF oder CR/LF.
 * Beispiel: Goto 180.0 45.5\n
 * loop() liest nur gerade verfügbare Bytes. Es gibt keine Warte-Schleifen,
 * delay()-Aufrufe oder String-Leseoperationen mit Timeout.
 *
 * Antwortverhalten:
 * - Bewegungs-/Einstellbefehle: +OK nach erfolgreicher Prüfung.
 * - Goto/Home/Null: spätere Fertigmeldung muss von der Bewegungslogik mit
 *   Serial.println("+DONE"), Serial.println("+HOMED") bzw. "+NULLED"
 *   gesendet werden.
 * - Pos und Status liefern momentan Beispielwerte, die später durch echte
 *   Zustandswerte ersetzt werden sollen.
 * - Jede gültige Anfrage wird zusätzlich als Testausgabe ausgegeben.
 */

namespace {
constexpr size_t LINE_CAPACITY = 96; // einschließlich abschließendem '\0'
MotionPlanner motionPlanner;
char lineBuffer[LINE_CAPACITY];
size_t lineLength = 0;
bool lineOverflow = false;
// Vergleicht ASCII-Text ohne Beachtung von Groß-/Kleinschreibung.
bool equalsIgnoreCase(const char *left, const char *right) {
    while (*left != '\0' && *right != '\0') {
        char a = *left++;
        char b = *right++;
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
        if (a != b) return false;
    }
    return *left == '\0' && *right == '\0';
}

// Überspringt Leerzeichen und Tabulatoren innerhalb eines Befehls.
char *nextToken(char *&cursor) {
    while (*cursor == ' ' || *cursor == '\t') ++cursor;
    if (*cursor == '\0') return nullptr;
    char *token = cursor;
    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t') ++cursor;
    if (*cursor != '\0') *cursor++ = '\0';
    return token;
}

// Liest eine endliche Zahl vollständig ein; ungültige/trailing Zeichen werden abgelehnt.
bool parseNumber(const char *text, float &value) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    value = strtof(text, &end);
    return end != text && *end == '\0' && isfinite(value);
}

bool parseBoolean(const char *text, bool &value) {
    if (text == nullptr) return false;
    if (equalsIgnoreCase(text, "true") || equalsIgnoreCase(text, "1")) {
        value = true;
        return true;
    }
    if (equalsIgnoreCase(text, "false") || equalsIgnoreCase(text, "0")) {
        value = false;
        return true;
    }
    return false;
}

// Lehnt fehlende oder überzählige Argumente ab.
bool hasExactArguments(char *&cursor, size_t expected) {
    for (size_t i = 0; i < expected; ++i) {
        if (nextToken(cursor) == nullptr) return false;
    }
    return nextToken(cursor) == nullptr;
}

void printBadCommand() {
    Serial.println("-ERR");
}

void processLine(char *line) {
    char *cursor = line;
    char *command = nextToken(cursor);
    if (command == nullptr) return; // leere Zeilen ignorieren

    // Goto <Azimut> <Altitude>
    if (equalsIgnoreCase(command, "Goto")) {
        char *azText = nextToken(cursor);
        char *altText = nextToken(cursor);
        float azimuth, altitude;
        if (azText == nullptr || altText == nullptr || nextToken(cursor) != nullptr ||
            !parseNumber(azText, azimuth) || !parseNumber(altText, altitude)) {
            printBadCommand(); return;
        }
        Serial.print("TEST Goto "); Serial.print(azimuth); Serial.print(' ');
        Serial.println(altitude);
        motionPlanner.gotoPosition(azimuth, altitude);
        Serial.println("+OK");
        return;
    }

    // Follow <Rate> <Enabled>
    if (equalsIgnoreCase(command, "Follow")) {
        char *rateText = nextToken(cursor);
        char *enabledText = nextToken(cursor);
        float rate; bool enabled;
        if (rateText == nullptr || enabledText == nullptr || nextToken(cursor) != nullptr ||
            !parseNumber(rateText, rate) || !parseBoolean(enabledText, enabled)) {
            printBadCommand(); return;
        }
        Serial.print("TEST Follow "); Serial.print(rate); Serial.print(' ');
        Serial.println(enabled ? "true" : "false");
        motionPlanner.setFollow(rate, enabled);
        Serial.println("+OK");
        return;
    }

    // Home hat eine asynchrone Abschlussmeldung +HOMED.
    if (equalsIgnoreCase(command, "Home")) {
        if (!hasExactArguments(cursor, 0)) { printBadCommand(); return; }
        Serial.println("TEST Home");
        motionPlanner.startHome();
        Serial.println("+OK");
        return;
    }

    // Null <Azimut Nordstern> <Altitude Nordstern>
    if (equalsIgnoreCase(command, "Null")) {
        char *azText = nextToken(cursor);
        char *altText = nextToken(cursor);
        float azimuth, altitude;
        if (azText == nullptr || altText == nullptr || nextToken(cursor) != nullptr ||
            !parseNumber(azText, azimuth) || !parseNumber(altText, altitude)) {
            printBadCommand(); return;
        }
        Serial.print("TEST Null "); Serial.print(azimuth); Serial.print(' ');
        Serial.println(altitude);
        motionPlanner.setNull(azimuth, altitude);
        Serial.println("+OK");
        return;
    }

    // Manuelle Einzelachsen-Befehle: <Befehl> <Geschwindigkeit>
    const char *manualName = nullptr;
    if (equalsIgnoreCase(command, "Auf")) manualName = "Auf";
    else if (equalsIgnoreCase(command, "Ab")) manualName = "Ab";
    else if (equalsIgnoreCase(command, "Links")) manualName = "Links";
    else if (equalsIgnoreCase(command, "Rechts")) manualName = "Rechts";
    if (manualName != nullptr) {
        char *speedText = nextToken(cursor);
        float speed;
        if (speedText == nullptr || nextToken(cursor) != nullptr || !parseNumber(speedText, speed)) {
            printBadCommand(); return;
        }
        Serial.print("TEST "); Serial.print(manualName); Serial.print(' ');
        Serial.println(speed);
        if (equalsIgnoreCase(manualName, "Auf")) motionPlanner.moveUp(speed);
        else if (equalsIgnoreCase(manualName, "Ab")) motionPlanner.moveDown(speed);
        else if (equalsIgnoreCase(manualName, "Links")) motionPlanner.moveLeft(speed);
        else motionPlanner.moveRight(speed);
        Serial.println("+OK");
        return;
    }

    // Stop und E-Stop haben keine Argumente.
    if (equalsIgnoreCase(command, "Stop") || equalsIgnoreCase(command, "E-Stop")) {
        if (!hasExactArguments(cursor, 0)) { printBadCommand(); return; }
        Serial.print("TEST "); Serial.println(command);
        if (equalsIgnoreCase(command, "E-Stop")) motionPlanner.emergencyStop();
        else motionPlanner.stopMotion();
        Serial.println("+OK");
        return;
    }

    // Pos und Status sind Abfragen mit jeweils einer Antwortzeile.
    if (equalsIgnoreCase(command, "Pos")) {
        if (!hasExactArguments(cursor, 0)) { printBadCommand(); return; }
        float azimuth, altitude;
        motionPlanner.getPosition(azimuth, altitude);
        Serial.println("TEST Pos");
        Serial.print("+POS "); Serial.print(azimuth); Serial.print(' '); Serial.println(altitude);
        return;
    }
    if (equalsIgnoreCase(command, "Status")) {
        if (!hasExactArguments(cursor, 0)) { printBadCommand(); return; }
        Serial.println("TEST Status");
        Serial.println(motionPlanner.isBusy() ? "+STATUS BUSY" : "+STATUS IDLE");
        return;
    }

    // Speed <Geschwindigkeit>
    if (equalsIgnoreCase(command, "Speed")) {
        char *speedText = nextToken(cursor);
        float speed;
        if (speedText == nullptr || nextToken(cursor) != nullptr || !parseNumber(speedText, speed)) {
            printBadCommand(); return;
        }
        Serial.print("TEST Speed "); Serial.println(speed);
        motionPlanner.setSpeed(speed);
        Serial.println("+OK");
        return;
    }

    printBadCommand(); // unbekannter Befehl
}

void receiveSerial() {
    // Pro loop()-Durchlauf höchstens 64 Zeichen verarbeiten, damit andere
    // Aufgaben regelmäßig CPU-Zeit erhalten, auch bei dauerhaftem Datenstrom.
    uint8_t processed = 0;
    while (Serial.available() > 0 && processed < 64) {
        const char incoming = static_cast<char>(Serial.read());
        ++processed;

        if (incoming == '\n') {
            if (lineOverflow) {
                printBadCommand();
            } else {
                lineBuffer[lineLength] = '\0';
                // CR von CR/LF entfernen
                if (lineLength > 0 && lineBuffer[lineLength - 1] == '\r')
                    lineBuffer[--lineLength] = '\0';
                processLine(lineBuffer);
            }
            lineLength = 0;
            lineOverflow = false;
        } else if (!lineOverflow) {
            if (lineLength + 1 < LINE_CAPACITY) {
                lineBuffer[lineLength++] = incoming;
            } else {
                // Rest der überlangen Zeile bis zum LF verwerfen.
                lineOverflow = true;
            }
        }
    }
}
} // namespace

void setup() {
    Serial.begin(115200);
    motionPlanner.begin();
}

void loop() {
    receiveSerial(); // regelmäßig aufrufen; keine blockierende Eingabe verwenden
    motionPlanner.update();
}






