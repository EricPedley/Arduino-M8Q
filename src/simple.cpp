#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include "ubxPacket.h"

void setup() {
    Serial.begin(9600);
    Serial.println("SparkFun Ublox Example");
    
    Wire.begin();
}

void loop() {
    Wire.beginTransmission(0x42);
    uint8_t messageBuff[7] = {0xb5, 0x62, 0x06, 0x02, 0x01, 0x00, 0x00};
    uint8_t CK_A{0}, CK_B{0};

    Wire.write(messageBuff[0]);
    Wire.write(messageBuff[1]);
    for(int i=2;i<7;i++) {
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
    uint8_t buffer[1024];

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
        packet.reset();
    }
    else
    {
        Serial.println("No ublox packet available");
    }
    delay(1000);
    Serial.println();
}