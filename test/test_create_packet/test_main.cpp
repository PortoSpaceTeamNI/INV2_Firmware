/**
 * Host-side (native) unit tests for rc_create_packet() in
 * lib/rocket_core/rc_packet_utils.
 *
 * Run with:  pio test -e native
 */
#include <unity.h>
#include <string.h>
#include "rc_packet_utils.h"
#include "rc_parser.h"
#include "rc_ids.h"
#include "rc_commands.h"

void setUp(void) {}
void tearDown(void) {}

/* ------------------------------------------------------------------ */
/* Argument validation                                                 */
/* ------------------------------------------------------------------ */

static void test_null_packet_returns_minus1(void)
{
    uint8_t payload[2] = {0x01, 0x02};
    TEST_ASSERT_EQUAL_INT(-1, rc_create_packet(NULL, CORTEX_ID, HYDRA_UF_ID,
                                               CMD_STATUS, payload, 2));
}

static void test_payload_too_large_returns_minus1(void)
{
    packet_t pkt;
    uint8_t big[MAX_PAYLOAD_SIZE + 1];
    memset(big, 0xAA, sizeof(big));
    TEST_ASSERT_EQUAL_INT(-1, rc_create_packet(&pkt, CORTEX_ID, HYDRA_UF_ID,
                                               CMD_STATUS, big, MAX_PAYLOAD_SIZE + 1));
}

static void test_nonzero_size_null_payload_returns_minus1(void)
{
    packet_t pkt;
    TEST_ASSERT_EQUAL_INT(-1, rc_create_packet(&pkt, CORTEX_ID, HYDRA_LF_ID,
                                               CMD_STATUS, NULL, 3));
}

static void test_zero_size_null_payload_ok(void)
{
    packet_t pkt;
    int ret = rc_create_packet(&pkt, CORTEX_ID, BROADCAST_ID,
                               CMD_STATUS, NULL, 0);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_EQUAL_UINT8(0, pkt.payload_size);
}

/* ------------------------------------------------------------------ */
/* Correct field population                                            */
/* ------------------------------------------------------------------ */

static void test_fields_set_correctly(void)
{
    packet_t pkt;
    memset(&pkt, 0xFF, sizeof(pkt)); // pre-fill with garbage

    uint8_t payload[3] = {0x10, 0x20, 0x30};
    int ret = rc_create_packet(&pkt, CORTEX_ID, HYDRA_FS_ID, CMD_MANUAL_EXEC,
                               payload, 3);

    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_EQUAL_UINT8(CORTEX_ID,      pkt.sender_id);
    TEST_ASSERT_EQUAL_UINT8(HYDRA_FS_ID,    pkt.target_id);
    TEST_ASSERT_EQUAL_UINT8(CMD_MANUAL_EXEC, pkt.cmd);
    TEST_ASSERT_EQUAL_UINT8(3,              pkt.payload_size);
}

static void test_payload_bytes_copied(void)
{
    packet_t pkt;
    uint8_t src[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    rc_create_packet(&pkt, CORTEX_ID, HYDRA_UF_ID, CMD_ACK, src, 4);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, pkt.payload, 4);
}

static void test_remaining_payload_zeroed(void)
{
    packet_t pkt;
    memset(&pkt, 0xFF, sizeof(pkt));

    uint8_t src[2] = {0x01, 0x02};
    rc_create_packet(&pkt, CORTEX_ID, HYDRA_LF_ID, CMD_STATUS, src, 2);

    /* Bytes [2 .. MAX_PAYLOAD_SIZE-1] must be zero */
    for (int i = 2; i < MAX_PAYLOAD_SIZE; i++) {
        TEST_ASSERT_EQUAL_HEX8(0x00, pkt.payload[i]);
    }
}

static void test_crc_set_to_zero(void)
{
    packet_t pkt;
    memset(&pkt, 0xFF, sizeof(pkt));

    uint8_t src[1] = {0x42};
    rc_create_packet(&pkt, CORTEX_ID, BROADCAST_ID, CMD_ABORT, src, 1);

    TEST_ASSERT_EQUAL_UINT16(0x0000, pkt.crc);
}

static void test_max_payload_size_accepted(void)
{
    packet_t pkt;
    uint8_t big[MAX_PAYLOAD_SIZE];
    for (int i = 0; i < MAX_PAYLOAD_SIZE; i++)
        big[i] = (uint8_t)i;

    int ret = rc_create_packet(&pkt, CORTEX_ID, BROADCAST_ID, CMD_STATUS,
                               big, MAX_PAYLOAD_SIZE);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_EQUAL_UINT8(MAX_PAYLOAD_SIZE, pkt.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(big, pkt.payload, MAX_PAYLOAD_SIZE);
}

/* ------------------------------------------------------------------ */
/* Round-trip with rc_serialize + rc_parse_byte                        */
/* ------------------------------------------------------------------ */

static void test_created_packet_survives_roundtrip(void)
{
    packet_t tx;
    uint8_t payload[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    rc_create_packet(&tx, CORTEX_ID, HYDRA_UF_ID, CMD_MANUAL_EXEC, payload, 5);

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    TEST_ASSERT_NOT_EQUAL(0, n);

    packet_t rx;
    rc_parse_state_t st = RC_SYNC;
    for (size_t i = 0; i < n; i++)
        st = rc_parse_byte(buf[i], &rx, st);

    TEST_ASSERT_EQUAL_INT(RC_END, st);
    TEST_ASSERT_EQUAL_UINT8(CORTEX_ID,       rx.sender_id);
    TEST_ASSERT_EQUAL_UINT8(HYDRA_UF_ID,     rx.target_id);
    TEST_ASSERT_EQUAL_UINT8(CMD_MANUAL_EXEC, rx.cmd);
    TEST_ASSERT_EQUAL_UINT8(5,               rx.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload,   rx.payload, 5);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();

    /* Argument validation */
    RUN_TEST(test_null_packet_returns_minus1);
    RUN_TEST(test_payload_too_large_returns_minus1);
    RUN_TEST(test_nonzero_size_null_payload_returns_minus1);
    RUN_TEST(test_zero_size_null_payload_ok);

    /* Field population */
    RUN_TEST(test_fields_set_correctly);
    RUN_TEST(test_payload_bytes_copied);
    RUN_TEST(test_remaining_payload_zeroed);
    RUN_TEST(test_crc_set_to_zero);
    RUN_TEST(test_max_payload_size_accepted);

    /* Integration roundtrip */
    RUN_TEST(test_created_packet_survives_roundtrip);

    return UNITY_END();
}
