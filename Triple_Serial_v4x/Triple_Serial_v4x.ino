#include <Streaming.h>

#define SerialGPS1 Serial7              // Main postion receiver (GGA & VTG)
#define SerialGPS2 Serial2              // Dual heading receiver  (relposNED)
const int32_t baudGPS = 460800;

bool USB1DTR = false;               // to track bridge mode state
bool USB2DTR = false;
uint32_t GPS1BAUD;                  // to track baud changes for bridge mode
uint32_t GPS2BAUD;

#define STEER_PIN        32
#define WORK_PIN         34
#define KICKOUT_D_PIN    37     // REMOTE input


void setup()
{
  pinMode(STEER_PIN, INPUT_PULLUP);
  pinMode(WORK_PIN, INPUT_PULLUP);
  pinMode(KICKOUT_D_PIN, INPUT_PULLUP);

  SerialGPS1.begin(baudGPS);
  GPS1BAUD = baudGPS;
  SerialGPS2.begin(baudGPS);
  GPS2BAUD = baudGPS;
  Serial.print("\r\n\n*** Setup complete ***");
} 

void loop()
{
  checkUSBMode();
  processGPS1Port();
  processGPS2Port();
  checkInputBtns();
}

void checkInputBtns()
{
  if (!digitalRead(STEER_PIN))
  {
    Serial.print("\r\nSteer Btn");
    while (!digitalRead(STEER_PIN));
    Serial.print("\r\n - configuring UM982 on GPS1");
    configUM982(&SerialGPS1);
  }
}

void configUM982(HardwareSerial *SERIAL_PORT)
{
  SERIAL_PORT->clear();
  SERIAL_PORT->write("FRESET\r\n");   // send RESET command at 460800 baud
  delay(6000);                        // wait while um982 reboots (measured 5.6s)

  SERIAL_PORT->begin(115200);         // change to factory default 115200
  SERIAL_PORT->write("FRESET\r\n");   // send RESET command again in case um982 was at factory default 115200 baud
  delay(6000);

  // start configuration process
  SERIAL_PORT->write("UNLOGALL\r\n");
  delay(100);
  SERIAL_PORT->clear();

  // Set UM982 operating mode
  Serial.println("\r\nSetting mode");
  SERIAL_PORT->write("MODE ROVER SURVEY\r\n");
  delay(100);

  // Set heading to tractor
  Serial.println("Setting heading to variablelength");
  SERIAL_PORT->write("CONFIG HEADING VARIABLELENGTH\r\n");
  delay(100);

  // Set heading reliability
  Serial.println("Setting heading reliability");
  SERIAL_PORT->write("CONFIG HEADING RELIABILITY 3\r\n");
  delay(100);

  // Set heading smoothing
  Serial.println("Setting heading smoothing");
  SERIAL_PORT->write("CONFIG SMOOTH HEADING 10\r\n");
  delay(100);

  // Set rtk reliability
  Serial.println("Setting RTK reliability");
  SERIAL_PORT->write("CONFIG RTK RELIABILITY 3 1\r\n");
  delay(100);

  // Set rtk height smoothing
  Serial.println("Setting RTK height smoothing");
  SERIAL_PORT->write("CONFIG SMOOTH RTKHEIGHT 10\r\n");
  delay(100);

  // Set GGA message and rate
  Serial.println("Setting GGA");
  SERIAL_PORT->write("GPGGA COM1 0.1\r\n");
  delay(100);

  // Set VTG message and rate
  Serial.println("Setting VTG");
  SERIAL_PORT->write("GPVTG COM1 0.1\r\n");
  delay(100);

  // Set HPR message and rate
  Serial.println("Setting HPR");
  SERIAL_PORT->write("GPHPR COM1 0.1\r\n");
  delay(100);

  // Setting the flag to signal UM982 is configured for AOG
  //Serial.println("Setting UM982 configured flag");
  //SERIAL_PORT->write("CONFIG ANTENNADELTAHEN 0.0099 0.0099 0.0099\r\n");
  //delay(100);

  // Set COM1 to 460800
  Serial.println("Setting COM1<->Teensy to 460800 bps");
  SERIAL_PORT->write("CONFIG COM1 460800\r\n");
  delay(100);
  SERIAL_PORT->begin(460800);         // change to 460800 takes effect immediately so we need to also change baud on Teensy

  // Set COM2 to 460800
  //Serial.println("Setting COM2 to 460800 bps");
  //SERIAL_PORT->write("CONFIG COM2 460800\r\n");
  //delay(100);

  // Set COM3 to 460800
  //Serial.println("Setting COM3 to 115200 bps");
  //SERIAL_PORT->write("CONFIG COM3 115200\r\n");
  //delay(100);

  // Saving the configuration in the UM982
  Serial.println("Saving the configuration");
  SERIAL_PORT->write("SAVECONFIG\r\n");
  Serial.println();
  SERIAL_PORT->clear();
  delay(100);

}

void processGPS1Port()
{
  // process USB1/GPS1 serial port
  if (!USB1DTR) // no USB1 connection, just read port to clear rx buffer
  {
    if (SerialGPS1.available()) SerialGPS1.read();
  }
  else         // in SerialUSB1<->SerialGPS1 bridge mode, for connecting via u-center/UPrecise
  {
    if (SerialGPS1.available()) {
      while (SerialGPS1.available()) {     // seems necessary to keep sentences/packets grouped as tight as possible
        SerialUSB1.write(SerialGPS1.read());
      }
    }
    if (SerialUSB1.available()) {           // seems necessary to ensure UBX msgs from U-Center aren't interrupted by RTCM data (xbee or ntrip)
      while (SerialUSB1.available()) {
        SerialGPS1.write(SerialUSB1.read());
      }
    }
  }
}

void processGPS2Port()
{
  // process USB2/GPS2 serial port
  if (!USB2DTR) // no USB1 connection, just read port to clear rx buffer
  {
    if (SerialGPS2.available()) SerialGPS2.read();
  }
  else         // in SerialUSB2<->SerialGPS2 bridge mode, for connecting via u-center/UPrecise
  {
    if (SerialGPS2.available()) {
      while (SerialGPS2.available()) {     // seems necessary to keep sentences/packets grouped as tight as possible
        SerialUSB2.write(SerialGPS2.read());
      }
    }
    if (SerialUSB2.available()) {           // seems necessary to ensure UBX msgs from U-Center aren't interrupted by RTCM data (xbee or ntrip)
      while (SerialUSB2.available()) {
        SerialGPS2.write(SerialUSB2.read());
      }
    }
  }
}

void checkUSBMode()
{
  // check USB1
  static bool prevUSB1DTR;
  USB1DTR = SerialUSB1.dtr(); 
  if (USB1DTR != prevUSB1DTR) {
    Serial << "\r\n**SerialUSB1 " << (USB1DTR ? "bridged with GPS1" : "disconnected");
    if (USB1DTR) {
      if (SerialUSB1.baud() == GPS1BAUD) Serial << ", baud set at " << baudGPS << " (default)";
    } else {
      if (GPS1BAUD != baudGPS){
        SerialGPS1.begin(baudGPS);
        GPS1BAUD = baudGPS;
        Serial << ", baud reverted back to default " << GPS1BAUD;
      }
    }
    prevUSB1DTR = USB1DTR;
  }

  if (USB1DTR) {
    if (SerialUSB1.baud() != GPS1BAUD) {
      SerialGPS1.begin(SerialUSB1.baud());
      GPS1BAUD = SerialUSB1.baud();
      Serial << "\r\n**GPS1 baud changed to " << GPS1BAUD;
      if (GPS1BAUD == baudGPS) Serial << " (default)";
    }
  }

  // check USB2
  static bool prevUSB2DTR;
  USB2DTR = SerialUSB2.dtr(); 
  if (USB2DTR != prevUSB2DTR) {
    Serial << "\r\n**SerialUSB2 " << (USB2DTR ? "bridged with GPS2" : "disconnected");
    if (USB2DTR) {
      if (SerialUSB2.baud() == GPS2BAUD) Serial << ", baud set at " << baudGPS << " (default)";
    } else {
      if (GPS2BAUD != baudGPS){
        SerialGPS2.begin(baudGPS);
        GPS2BAUD = baudGPS;
        Serial << ", baud reverted back to default " << GPS2BAUD;
      }
    }
    prevUSB2DTR = USB2DTR;
  }

  if (USB2DTR) {
    if (SerialUSB2.baud() != GPS2BAUD) {
      SerialGPS2.begin(SerialUSB2.baud());
      GPS2BAUD = SerialUSB2.baud();
      Serial << "\r\n**GPS2 baud changed to " << GPS2BAUD;
      if (GPS2BAUD == baudGPS) Serial << " (default)";
    }
  }
}