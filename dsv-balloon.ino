/*
  Basic tracker code taken from
  https://git.ok1kvk.cz/RAJlab/openSTRATOkit/-/blob/master/FW/openSTRATOkit_basic-tracker/openSTRATOkit_basic-tracker.ino
  Licensed under Creative Commons Attribution 4.0 International License.
*/

/*
  openSTRATOkit Basic Tracker

  @DevOK1RAJ Team, code by OK1CDJ 3/2021

  low ("space") frequency:     434.69 MHz
  frequency shift:             610 Hz
  baud rate:                   300 baud
  encoding:                    ASCII (7-bit)
  stop bits:                   2

  NOTE: RTTY frequency shift will be rounded
        to the nearest multiple of frequency step size.
        The exact value depends on the module:
        RF69 - 61 Hz steps

  MegaCoreX documentation: https://github.com/MCUdude/MegaCoreX

  Compile and upload instructions:
  - if you're using Windows, you can skip all but the last step
  and download a portable version of Arduino IDE with everything
  preinstalled: https://files.dotknisevesmiru.cz/arduino_openSTRATOkit.zip
  - In your Arduino IDE, go to File > Preferences and enter
  https://mcudude.github.io/MegaCoreX/package_MCUdude_MegaCoreX_index.json
  into the “Additional Board Manager URLs” field
  - Go to Tools > Board > Boards Manager…, search for MegaCoreX and install
    MegaCoreX by MCUdude
  - In the Tools > Board menu select MegaCoreX > ATmega4809
  - In the Tools menu, make sure you select:
    Clock: "Internal 16MHz"
    BOD: "BOD 2.6V"
    Pinout: "48 pin standard"
    Reset pin: "Reset"
    Bootloader: "Optiboot (UART3 alternative pins)"
    Programmer: "JTAG2UPDI"
  - Download the following libraries from the Arduino Library Manager
  (Project > Add library > Manage libraries...):
    Adafruit BME280 Library ba Adafruit - select "Install All" in the pop-up
  window RadioLib by Jan Gromes
  - Download and import TinyGPSPlus library from here:
  https://github.com/mikalhart/TinyGPSPlus/archive/refs/heads/master.zip
  (Project > Add library > Add .ZIP Library)
  - Connect your openSTRATOkit and select it in Tools > Port
  (make sure you use a short, quality cable, like the one provided with the kit)
*/

// libraries
#include "MPU9250.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <RadioLib.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <util/crc16.h>

// Radio Settings
#define FREQ 434.690
#define SHIFT 610
#define BAUD 300
#define ENC ASCII
#define STOPB 2

#define CALL "TTS8";

long pkt_num = 1; // packet number

File myFile;

RF69 radio = new Module(33, 9, RADIOLIB_NC);

// create RTTY client instance using the FSK module
RTTYClient rtty(&radio);

TinyGPSPlus gps;

Adafruit_BME280 bme;

MPU9250 IMU(Wire, 0x68);

void (*resetFunc)(void) = 0; // declare reset function @ address 0

void setup() {

  // init serial comm
  Serial3.pins(12, 13);
  Serial3.begin(115200);
  Serial1.pins(18, 19);
  Serial1.begin(9600);
  Serial3.println("openSTRATOtracker (modified)");
  Serial3.println();

  // init and check for BME280
  if (!bme.begin(0x76)) {
    Serial3.print("[BME280], no BME280 detected...");
  } else
    Serial3.println("[BME280] found...");

  // init radio
  Serial3.print(F("[RF69] Initializing ... "));
  SPI.pins(30, 31, 32, 33);
  int state = radio.begin();

  // check for errors
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();
    while (true)
      ;
  }

  // radio output power
  Serial3.print(F("[RF69] Setting high power module ... "));
  state = radio.setOutputPower(20, true);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();
    while (true)
      ;
  }

  // set-up rtty comm
  Serial3.print(F("[RTTY] Initializing ... "));
  state = rtty.begin(FREQ, SHIFT, BAUD, ENC, STOPB);
  if (state == ERR_NONE) {
    Serial3.println(F("success!"));
  } else {
    Serial3.print(F("failed, code "));
    Serial3.println(state);
    resetFunc();
    while (true)
      ;
  }

  // set-up GPS
  Serial3.println(F("[GPS] Set flight mode ... "));
  setGPS_DynamicModel6();
  delay(500);
  setGps_MaxPerformanceMode();
  delay(500);

  SPI.pins(30, 31, 32, 33);

  digitalWrite(33, 1);

  state = IMU.begin();
  if (state < 0) {
    Serial3.print(F("IMU initialization failed, code "));
    Serial3.println(state);
    resetFunc();
    while (true)
      ;
  }

  // init SD card
  Serial3.print("[SD] Initializing SD card...");
  if (!SD.begin(10)) {
    Serial3.println("initialization failed!");
  } else
    Serial3.println(F("success!"));
}

void loop() {

  // read GPS and send RTTY messages
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
    sendData();
  }

  // try to fix the GPS in case of a malfunction
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial3.println(F("No GPS detected."));
    resetGPS();
    resetFunc();
    while (true)
      ;
  }
}

// Hardware pin definitions
// int UVOUT = A0; //Output from the sensor
// int REF_3V3 = A1; //3.3V power on the Arduino board

// void setup()
// {
//   Serial.begin(9600);
//
//   pinMode(UVOUT, INPUT);
//   pinMode(REF_3V3, INPUT);
//
//   Serial.println("MP8511 example");
// }
//
// void loop()
// {
//   int uvLevel = averageAnalogRead(UVOUT);
//   int refLevel = averageAnalogRead(REF_3V3);
//
//   Use the 3.3V power pin as a reference to get a very accurate output value
//   from sensor float outputVoltage = 3.3 / refLevel * uvLevel;
//
//   float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
//
//   Serial.print("MP8511 output: ");
//   Serial.print(uvLevel);
//
//   Serial.print(" MP8511 voltage: ");
//   Serial.print(outputVoltage);
//
//   Serial.print(" UV Intensity (mW/cm^2): ");
//   Serial.print(uvIntensity);
//
//   Serial.println();
//
//   delay(100);
// }

// Takes an average of readings on a given pin
// Returns the average
int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

// The Arduino Map function but for floats
// From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min,
               float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// compile sensor information and transmit it
void sendData() {

  // the condition is not really relevant anymore with more data being reported
  if (true /*gps.location.isUpdated() && gps.altitude.isUpdated()*/) {

    float batt_voltage = (analogRead(A3) * 3.3 / 1024) *
                         ((15.0 + 33.0) / 33.0); // divider 15k and 33k

    String datastring;

    datastring += "$$$$";
    datastring += CALL;
    datastring += ",";
    datastring += String(pkt_num) + ","; // Packet number

    // GPS time
    if (gps.time.hour() < 10)
      datastring += "0" + String(gps.time.hour()) + ":";
    else
      datastring += String(gps.time.hour()) + ":";

    if (gps.time.minute() < 10)
      datastring += "0" + String(gps.time.minute()) + ":";
    else
      datastring += String(gps.time.minute()) + ":";

    if (gps.time.second() < 10)
      datastring += "0" + String(gps.time.second()) + ",";
    else
      datastring += String(gps.time.second()) + ",";

    datastring += String(gps.location.lat(), 6) + ",";    // lat
    datastring += String(gps.location.lng(), 6) + ",";    // long
    datastring += String(gps.altitude.meters(), 0) + ","; // altitude
    // datastring += String(gps.speed.mps()) + ",";          // speed
    // datastring += String(gps.course.deg()) + ",";         // course
    datastring += String(batt_voltage, 2) + ","; // voltage
    // datastring += String(temp.temperature, 1) + ",";      // temperature
    // internal
    datastring +=
        String(bme.readTemperature(), 1) + ",";        // temperature external
    datastring += String(bme.readPressure(), 0) + ","; // pressure
    datastring += String(gps.satellites.value());      // sats

    // checksum
    unsigned int CHECKSUM = gps_CRC16_checksum(datastring.c_str());
    char checksum_str[6];
    sprintf(checksum_str, "*%04X", CHECKSUM);
    datastring += String(checksum_str);

    int secret_start = datastring.length();

    // start of the super secret message
    datastring += "\nhehe";

    // calculate battery voltage
    // uv - shamelessly taken from
    // http://wiki.sunfounder.cc/index.php?title=GYML8511_UV_Sensor
    float uvIntensity;
    {
      // uv
      int uvLevel = averageAnalogRead(A0);
      int refLevel = averageAnalogRead(A1);

      // Use the 3.3V power pin as a reference to get a very accurate output
      // value from sensor
      float outputVoltage = 3.3 / refLevel * uvLevel;

      uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
      datastring += String(uvIntensity, 6) + ",";
    }

    // taken from https://robojax.com/learn/arduino/?vid=robojax-MPU9250
    {
      IMU.readSensor();
      datastring += String(IMU.getAccelX_mss(), 3) + ",";
      datastring += String(IMU.getAccelY_mss(), 3) + ",";
      datastring += String(IMU.getAccelZ_mss(), 3) + ",";

      datastring += String(IMU.getGyroX_rads(), 3) + ",";
      datastring += String(IMU.getGyroY_rads(), 3) + ",";
      datastring += String(IMU.getGyroZ_rads(), 3) + ",";

      datastring += String(IMU.getMagX_uT(), 3) + ",";
      datastring += String(IMU.getMagY_uT(), 3) + ",";
      datastring += String(IMU.getMagZ_uT(), 3);
    }

    unsigned int CHECKSUM2 =
        gps_CRC16_checksum(datastring.c_str() + secret_start);
    sprintf(checksum_str, "*%04X", CHECKSUM2);
    datastring += String(checksum_str);

    // transmit the data
    Serial3.println(F("[RTTY] Sending RTTY data ... "));

    // send out idle condition for 500 ms
    rtty.idle();
    delay(1000);

    Serial3.println(datastring);
    rtty.println(datastring);

    Serial3.println(F("[RTTY] Done!"));
    writeData(datastring); // write a copy to the SD card
    pkt_num++;             // advance packet number
  }
}

// write data to the SD card
void writeData(String Str) {
  myFile = SD.open("data.txt", FILE_WRITE);
  if (myFile) {
    Serial3.print("[SD] Writing to data.txt...");
    myFile.println(Str);
    myFile.close();
    Serial3.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial3.println("error opening data.txt");
  }
}

// calculate a CRC16 checksum
uint16_t gps_CRC16_checksum(char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }
  return crc;
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial1.flush();
  Serial1.write(0xFF); //
  delay(100);
  for (int i = 0; i < len; i++) {
    Serial1.write(MSG[i]);
  }
}

void resetGPS() {
  /*
  Forced (Watchdog)
  Coldstart
  */
  uint8_t set_reset[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
                         0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5};
  sendUBX(set_reset, sizeof(set_reset) / sizeof(uint8_t));
}

void setGPS_DynamicModel6() {
  /*
  CFG-NAV5

Header: 0xB5, 0x62,
ID: 0x06, 0x24,
Length 0x24, 0x00,
mask 0xFF, 0xFF,
dynModel:  0x06, (Airborne <1g)
fixMode: 0x03,
fixedAlt: 0x00, 0x00, 0x00, 0x00,
fixedAltVar: 0x10, 0x27, 0x00, 0x00,
minElev 0x05,
drLimit 0x00,
pDop 0xFA, 0x00,
tDop 0xFA, 0x00,
pAcc 0x64, 0x00,
tAcc 0x2C, 0x01,
staticHoldThresh 0x00,
dgpsTimeOut 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00,
CK_A 0x16,
CK_B 0xDC

  */
  int gps_set_sucess = 0;
  uint8_t setdm6[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
                      0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
                      0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
                      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  while (!gps_set_sucess) {
    sendUBX(setdm6, sizeof(setdm6) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setdm6);
  }
  // morse("OK");
}

void setGps_MaxPerformanceMode() {
  /*
  UBX-CFG-RMX - 0 Continuous Mode (Max Performance Mode)
  */
  // Set GPS for Max Performance Mode
  uint8_t setMax[] = {0xB5, 0x62, 0x06, 0x11, 0x02,
                      0x00, 0x08, 0x00, 0x21, 0x91}; // Setup for Max Power Mode
  sendUBX(setMax, sizeof(setMax) / sizeof(uint8_t));
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0;      // CK_A
  ackPacket[9] = 0;      // CK_B

  // Calculate the checksums
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      } else {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
}
