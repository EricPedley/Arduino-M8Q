// #include <Arduino.h>
// #include <Wire.h>
// #include "ubxPacket.h"
// #include "SparkFun_u-blox_GNSS_Arduino_Library.h"
// SFE_UBLOX_GNSS myGPS;

// void initGPS() {
//     Wire.beginTransmission(0x42);
//     Wire.write(0xB5);
//     Wire.write(0x62);
//     Wire.write(0x06);
//     Wire.write(0x00);
//     Wire.write(0x01);
//     Wire.write(0x00);
//     Wire.write(0x00);
//     Wire.write(0x07);
//     Wire.write(0x10);
//     Wire.endTransmission(true);
// }

// void setup() {
//     Serial.begin(9600);
//     Serial.println("SparkFun Ublox Example");
    
//     Wire.begin();
//     myGPS.begin();
// }

// void loop() {

//     // myGPS.getProtocolVersion();

//     Wire.beginTransmission(0x42);  
//     byte messageBuff[6] = {
//         0xB5, //magic number 1 for ublox
//         0x62, //magic number 2 for ublox
//         0x0A, //class
//         0x04, //id
//         0x0, //length
//         0x0 //length
//     };

//     uint8_t CK_A{0}, CK_B{0};

//     Wire.write(messageBuff[0]);
//     Wire.write(messageBuff[1]);
//     for(int i=2;i<6;i++) {
//         CK_A = CK_A + messageBuff[i];
//         CK_B = CK_B + CK_A;
//         Wire.write(messageBuff[i]);
//     }
//     Wire.write(CK_A);
//     Wire.write(CK_B);
//     // no payload because length 0, this is a pull request.

//     Wire.endTransmission(true);

//     // Wire.requestFrom(0x42,READ_LENGTH); // request bytes from register XY
//     Wire.beginTransmission(0x42);
//     Wire.write(0xFD);
//     Wire.endTransmission(false); // false to not release the line
//     Wire.requestFrom(0x42, 2);

//     uint8_t msb = Wire.read();
//     uint8_t lsb = Wire.read();
//     int bytesAvailable = (uint16_t)msb << 8 | lsb;

//     uint8_t bytesReturned = Wire.requestFrom(0x42, bytesAvailable);
//     Serial.println("Bytes returned:");
//     Serial.println(bytesReturned);

//     bool seenMagicNumber = false;

//     UBXPacketReader ubxPack;

//     for(int i=0;i<bytesReturned;i++) {
//         uint8_t incomingByte = Wire.read();
//         if(incomingByte == 0xB5) {
//             seenMagicNumber = true;
//             Serial.println("Found magic number 1");
//         } else if(seenMagicNumber && incomingByte == 0x62) {
//             Serial.println("Found magic number 2");
//             ubxPack.update(incomingByte);
//             for(int i=0;i<200 && !ubxPack.isComplete();i++) {
//                 ubxPack.update(Wire.read());
//             }
//             Serial.println("Packet complete");
//             Serial.print("Payload length: ");
//             Serial.println(ubxPack.getPayloadLength());
//             Serial.print("Message class: ");
//             Serial.println(ubxPack.getMessageClass(), HEX);
//             Serial.print("Message ID: ");
//             Serial.println(ubxPack.getMessageId(), HEX);
//             uint8_t* packet = ubxPack.getPayload();
//             for(int i=0;i<ubxPack.getPayloadLength();i++) {
//                 Serial.println(packet[i], HEX);
//             }
//             ubxPack.reset();
//             break;
//         } else {
//             seenMagicNumber = false;
//         }

//     }

//     // byte buff[READ_LENGTH];    
//     // size_t bytes_read = Wire.readBytes(buff, READ_LENGTH);
//     // // for (int i = 0; i < READ_LENGTH; i++) {
//     // //   Serial.println(buff[i], HEX);
//     // // }

//     // if(buff[0] != 0xB5 || buff[1] != 0x62) {
//     //   Serial.println("Wrong magic number");
//     //   return;
//     // }

//     // uint16_t length[2] = {buff[4], buff[5]};
//     // uint16_t lengthInt = (length[1] << 8) | length[0];
//     // Serial.println(bytes_read);
//     // Serial.println(lengthInt);

//     // while(Wire.available()) {
//     //   Wire.read();
//     // }
//     // char versionString[31];
//     // versionString[30] = '\0';
//     // for(int i=0;i<30;i++) {
//     //   versionString[i] = buff[i+6];
//     // }
//     // Serial.println(versionString);

//     Serial.println();
//     delay(1000);
// }