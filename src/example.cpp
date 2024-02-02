
// #include <Arduino.h>
// #include <Wire.h>

// #include "SparkFun_u-blox_GNSS_Arduino_Library.h"
// SFE_UBLOX_GNSS myGPS;

// void setup()
// {
//     Serial.begin(9600);
//     Serial.println("SparkFun Ublox Example");

//     Wire.begin();

//     if (myGPS.begin() == false)
//     {
//         Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
//         while (1);
//     }

//     //This will pipe all NMEA sentences to the serial port so we can see them
//     // myGPS.setNMEAOutputPort(Serial);
// }

// void loop()
// {
//     myGPS.getProtocolVersion();
//     // myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

//     delay(1000); //Don't pound too hard on the I2C bus
// }
