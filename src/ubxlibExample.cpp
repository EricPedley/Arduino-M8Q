// /******************************************************************************
// u-blox_GNSS.h
// u-blox GNSS Arduino
// Leonardo Bispo
// Mar 03, 2019
// https://github.com/ldab/u-blox_GNSS

// Distributed as-is; no warranty is given.
// ******************************************************************************/

// // Enable Serial debbug on Serial UART to see registers wrote
// #define GNSS_DEBUG Serial

// #include "ublox_GNSS.h"
// #include <Wire.h>
// float lat, lon, acc;

// fixType_t fix = NO_FIX;

// GNSS gnss(Wire);

// void setup()
// {
//     // put your setup code here, to run once:
//     Serial.begin(9600);
//     Wire.begin();

//     if (gnss.init())
//     {
//         Serial.println("\nGNSS initialized.");
//     }
//     else
//     {
//         Serial.println("\nFailed to initialize GNSS module.");
//     }
// }

// void loop()
// {
//     // Get coordinates with minimum 100m accuracy;
//     Serial.println("Get location");

//     if (gnss.getCoodinates(lon, lat, fix, acc, 100) == 0)
//     {
//         Serial.println("Failed to get coordinates, check signal, accuracy required or wiring");
//     }

//     Serial.println("\nHere you are, lon:" + String(lon, 7) + " lat:" + String(lat, 7));
//     Serial.println("calculated error: " + String(acc) + "m");

//     Serial.println("\nOr try the following link to see on google maps:");
//     Serial.println(String("https://www.google.com/maps/search/?api=1&query=") + String(lat, 7) + "," + String(lon, 7));

//     delay(50000);
// }