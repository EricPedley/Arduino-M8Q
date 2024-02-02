#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "ubxPacket.h"

#pragma pack(push, 1)
struct UBX_NAV_PVT_PAYLOAD {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};
#pragma pack(pop)

UBXPacketReader packet;

void sendUBX(uint8_t* message, uint16_t len) {
    Wire.beginTransmission(0x42);
    uint8_t CK_A{0}, CK_B{0};

    Wire.write(0xb5);
    Wire.write(0x62);
    for(uint16_t i=0;i<len;i++) {
        CK_A = CK_A + message[i];
        CK_B = CK_B + CK_A;
        Wire.write(message[i]);
    }
    Wire.write(CK_A);
    Wire.write(CK_B);
    Wire.endTransmission(true);
}

void setup() {
    Serial.begin(9600);
    Serial.println("SparkFun Ublox Example");
    
    Wire.begin(); // A4 is SDA, A5 is SCL

    // pinMode(A3, OUTPUT);
    // digitalWrite(A3, HIGH);    Serial.println("Requesting data from u-blox");
    // uint8_t configMessage[5] = {0x06, 0x00, 0x01, 0x00, 0x00};
    // sendUBX(configMessage, 5);
    // delay(100);
}




void loop() {
    uint8_t navMessage[4] = {0x01, 0x07, 0x00, 0x00};
    sendUBX(navMessage, 4); // trigger GPS module reset

    uint8_t len[2];
    uint16_t dataLen = 0; 
    uint8_t buffer[256];

    while(dataLen<1) {
        Wire.beginTransmission(0x42);
        Wire.write(0xfd);
        Wire.endTransmission(false);

        Wire.requestFrom(0x42, 2);

        len[0] = Wire.read();
        len[1] = Wire.read();
        dataLen = len[0] << 8 | len[1];
        if(dataLen==0) {
            delay(100);
            continue;
        }

        Wire.requestFrom(0x42, dataLen);
        size_t realBytes = 0;
        for(uint16_t i=0; i<dataLen; i++) {
            buffer[i] = Wire.read();
            Serial.print(buffer[i], HEX);
            if(buffer[i] != 0xff) {
                realBytes++;
            }
        }
        Serial.print("Real bytes: ");
        Serial.println(realBytes);
        Serial.print("Data length: ");
        Serial.println(dataLen);
        Serial.println();
    }

    for(uint16_t i = 0; i<dataLen; i++) {
        if(buffer[i] == 0xB5 && buffer[i+1] == 0x62) {
            Serial.println("Found UBX packet");
            i+=2; // skip the header
            while(i<dataLen && !packet.isComplete()) {
                UBXPacketUpdateResult res = packet.update(buffer[i]);
                if(res != UBXPacketUpdateResult::UPDATE_OK) {
                    Serial.print("Error updating packet: ");
                    Serial.println((int)res);
                }
                i++;
            }
            Serial.print("Payload length: ");
            Serial.println(packet.getPayloadLength());
            Serial.print("Message class: ");
            Serial.println(packet.getMessageClass(), HEX);
            Serial.print("Message ID: ");
            Serial.println(packet.getMessageId(), HEX);



            Serial.println("Payload:");
            Serial.flush();

            void* payload = packet.getPayload();
            UBX_NAV_PVT_PAYLOAD* pvt = (UBX_NAV_PVT_PAYLOAD*)payload;
            Serial.print("Fix Type:");
            Serial.println(pvt->fixType);
            Serial.print("Latitude:");
            Serial.println(pvt->lat);
            Serial.print("Year:");
            Serial.println(pvt->year);


            packet.reset();
            break;
        }
    }
    Serial.println();
    delay(5000);
}