#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "ubxPacket.h"

void setup() {
    Serial.begin(9600);
    Serial.println("SparkFun Ublox Example");
    
    Wire.begin(); // A4 is SDA, A5 is SCL
}

void loop() {
    Serial.println("Requesting data from u-blox");
    Wire.beginTransmission(0x42);
    uint8_t messageBuff[6] = {0xb5, 0x62, 0x0A, 0x04, 0x00, 0x00};
    uint8_t CK_A{0}, CK_B{0};

    Wire.write(messageBuff[0]);
    Wire.write(messageBuff[1]);
    for(int i=2;i<6;i++) {
        CK_A = CK_A + messageBuff[i];
        CK_B = CK_B + CK_A;
        Wire.write(messageBuff[i]);
    }
    Wire.write(CK_A);
    Wire.write(CK_B);
    uint8_t len[2];
    Wire.endTransmission(true);

    Wire.beginTransmission(0x42);
    Wire.write(0xfd);
    Wire.endTransmission(false);

    Wire.requestFrom(0x42, 2);

    Wire.readBytes(len, 2);
    uint16_t dataLen = len[0] << 8 | len[1];
    uint8_t buffer[4096];

    Wire.requestFrom(0x42, dataLen);
    Wire.readBytes(buffer, dataLen);
    Serial.print("Data length: ");
    Serial.println(dataLen);

    UBXPacketReader packet;
    if (buffer[0]==0xB5)
    {
        for(uint16_t i=2; i<dataLen; i++)
        {
            packet.update(buffer[i]);
            if(packet.isComplete())
            {
                break;
            }
        }
        Serial.print("Payload length: ");
        Serial.println(packet.getPayloadLength());
        Serial.print("Message class: ");
        Serial.println(packet.getMessageClass(), HEX);
        Serial.print("Message ID: ");
        Serial.println(packet.getMessageId(), HEX);

        // void* payload = packet.getPayload();

        // Serial.println("Payload:");
        // // char* swVersion = (char*)payload;
        // // Serial.print("Software version: ");
        // // Serial.println(swVersion);
        // // char* hwVersion = (char*)payload + 30;
        // // Serial.print("Hardware version: ");
        // // Serial.println(hwVersion);
        // for(int i=0;i<packet.getPayloadLength();i++) {
        //     Serial.print(*(char*)(payload+i));
        // }
        // Serial.println();

        packet.reset();
    }
    else
    {
        Serial.println("No ublox packet available");
    }
    delay(1000);
    Serial.println();
}