#include <unity.h>
#include "ubxPacket.h"

void test_short_message_completes() {
    UBXPacketReader packet;

    uint8_t messageBuff[9] = {0xb5, 0x62, 0x06, 0x02, 0x01, 0x00, 0x00, 0x09, 0x29};

    for(int i=2; i<9; i++) {
        TEST_ASSERT_EQUAL(UBXPacketUpdateResult::UPDATE_OK, packet.update(messageBuff[i]));
    }

    TEST_ASSERT_TRUE(packet.isComplete());
    TEST_ASSERT_EQUAL(1, packet.getPayloadLength());
    TEST_ASSERT_EQUAL(0x06, packet.getMessageClass());
    TEST_ASSERT_EQUAL(0x02, packet.getMessageId());

    TEST_ASSERT_EQUAL(0x00, packet.getPayload()[0]);

    packet.reset();
    TEST_ASSERT_FALSE(packet.isComplete());
    TEST_ASSERT_EQUAL(0, packet.getPayloadLength());
}


int main( int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_short_message_completes);

    UNITY_END();
}
