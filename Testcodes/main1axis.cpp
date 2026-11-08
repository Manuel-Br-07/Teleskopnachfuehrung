
#include <TMCStepper.h>

// Code zum bewegen einer Achse mit einem TMC2209 Steppertreiber über UART auf einem ESP32.

// PAN-Motor: STEP / DIR / ENABLE
// constexpr int PAN_STEP = 14;
// constexpr int PAN_DIR  = 27;
// constexpr int PAN_EN   = 26;

// TILT-Motor: STEP / DIR / ENABLE
constexpr int PAN_STEP = 25;
constexpr int PAN_DIR  = 33;
constexpr int PAN_EN   = 32;

// UART zum TMC2209
constexpr int UART_RX = 16;
constexpr int UART_TX = 17;

// An den konkreten Treiber anpassen!
constexpr float R_SENSE = 0.11f; // Beispielwert: 0,11 Ohm
constexpr uint8_t DRIVER_ADDRESS = 0b01;

// Strom in mA RMS
constexpr uint16_t MOTOR_CURRENT = 600;

// Bewegung
constexpr int STEPS = 51200;
constexpr int STEP_DELAY_US = 100; //281025;

TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);

void setup() {
  Serial.begin(115200);

  pinMode(PAN_STEP, OUTPUT);
  pinMode(PAN_DIR, OUTPUT);
  pinMode(PAN_EN, OUTPUT);

  digitalWrite(PAN_STEP, LOW);
  digitalWrite(PAN_DIR, LOW);
  digitalWrite(PAN_EN, HIGH); // Treiber zunächst deaktiviert

  // ESP32 UART2: RX = GPIO16, TX = GPIO17
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  delay(500);

    driver.begin();

  // Motorstrom
  driver.rms_current(MOTOR_CURRENT, 0.5);

  // Microstepping
  driver.microsteps(256);
  driver.intpol(false);

  // Betriebsart: SpreadCycle
  driver.en_spreadCycle(false);
  driver.pwm_autoscale(true);

  // Chopper-Konfiguration
  driver.toff(4);
  driver.blank_time(24);


 // 1. Verbindungstest
  bool connected = driver.test_connection();
  Serial.printf("test_connection: %s\n", connected ? "OK" : "FEHLER");

  // 2. Version lesen (muss 0x21 = 33 sein für TMC2209)
  uint8_t ver = driver.version();
  Serial.printf("Version: 0x%02X\n", ver);

  // 3. IFCNT lesen – muss sich bei jedem Befehl erhöhen
  uint32_t ifc1 = driver.IFCNT();
  driver.rms_current(500);
  uint32_t ifc2 = driver.IFCNT();
  Serial.printf("IFCNT: %u → %u (Δ=%d)\n", ifc1, ifc2, (int)(ifc2 - ifc1));

  // 4. MRES lesen

  uint8_t mres = driver.microsteps();
  Serial.printf("MRES nach Write: %u\n", mres);


  digitalWrite(PAN_EN, LOW);         // ENABLE: LOW = aktiv

  Serial.println("PAN-Treiber konfiguriert.");
  Serial.print("Motorstrom RMS: ");
  Serial.print(MOTOR_CURRENT);
  Serial.println(" mA");
}

void moveMotor(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(PAN_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PAN_STEP, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void loop() {
  digitalWrite(PAN_DIR, HIGH);
  moveMotor(STEPS);
  delay(1000);

  digitalWrite(PAN_DIR, LOW);
  moveMotor(STEPS);
  delay(2000);
}