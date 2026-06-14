/**
 * Host-side (native) unit tests for lib/rocket_core.
 *
 * Run with:  pio test -e native
 */
#include <unity.h>
#include <string.h>
#include "rocket_core.h"

void setUp(void) {}
void tearDown(void) {}

/* ------------------------------------------------------------------ */
/* Canonical enum values are pinned. These guard against the original  */
/* divergence bug (CMD_ARM was 5 on OBC, 4 on slaves) ever returning.  */
/* ------------------------------------------------------------------ */
static void test_command_values_pinned(void)
{
    TEST_ASSERT_EQUAL_INT(0, CMD_NONE);
    TEST_ASSERT_EQUAL_INT(1, CMD_STATUS);
    TEST_ASSERT_EQUAL_INT(2, CMD_ABORT);
    TEST_ASSERT_EQUAL_INT(3, CMD_STOP);
    TEST_ASSERT_EQUAL_INT(4, CMD_READY);
    TEST_ASSERT_EQUAL_INT(5, CMD_ARM);
    TEST_ASSERT_EQUAL_INT(6, CMD_FIRE);
    TEST_ASSERT_EQUAL_INT(7, CMD_LAUNCH_OVERRIDE);
    TEST_ASSERT_EQUAL_INT(8, CMD_FILL_EXEC);
    TEST_ASSERT_EQUAL_INT(9, CMD_FILL_RESUME);
    TEST_ASSERT_EQUAL_INT(10, CMD_MANUAL_EXEC);
    TEST_ASSERT_EQUAL_INT(11, CMD_ACK);
    TEST_ASSERT_EQUAL_INT(12, CMD_NACK);
    TEST_ASSERT_EQUAL_INT(13, CMD_COUNT);
}

static void test_manual_command_values_pinned(void)
{
    // payload[0] sub-commands for CMD_MANUAL_EXEC. The valve sub-commands must
    // keep their values (OBC and HYDRA already agreed on these on the wire).
    TEST_ASSERT_EQUAL_INT(0, CMD_MANUAL_SD_LOG_START);
    TEST_ASSERT_EQUAL_INT(1, CMD_MANUAL_SD_LOG_STOP);
    TEST_ASSERT_EQUAL_INT(2, CMD_MANUAL_SD_STATUS);
    TEST_ASSERT_EQUAL_INT(3, CMD_MANUAL_VALVE_STATE);
    TEST_ASSERT_EQUAL_INT(4, CMD_MANUAL_VALVE_MS);
    TEST_ASSERT_EQUAL_INT(5, CMD_MANUAL_ACK);
}

static void test_board_ids_pinned(void)
{
    TEST_ASSERT_EQUAL_INT(0, GROUND_ID);
    TEST_ASSERT_EQUAL_INT(1, OBC_ID);
    TEST_ASSERT_EQUAL_INT(2, HYDRA_UF_ID);
    TEST_ASSERT_EQUAL_INT(3, HYDRA_LF_ID);
    TEST_ASSERT_EQUAL_INT(4, HYDRA_FS_ID);
    TEST_ASSERT_EQUAL_INT(5, NAVIGATOR_ID);
    TEST_ASSERT_EQUAL_INT(6, LIFT_TANK_ID);
    TEST_ASSERT_EQUAL_INT(7, LIFT_BOTTLE_ID);
    TEST_ASSERT_EQUAL_INT(8, LIFT_THRUST_ID);
    TEST_ASSERT_EQUAL_INT(0xFF, BROADCAST_ID);
}

static void test_state_values_pinned(void)
{
    TEST_ASSERT_EQUAL_INT(0, IDLE);
    TEST_ASSERT_EQUAL_INT(13, ABORT);
    TEST_ASSERT_EQUAL_INT(14, state_count);
    TEST_ASSERT_EQUAL_INT(-1, S_NONE);
}

/* ------------------------------------------------------------------ */
/* Frame parser / serializer                                           */
/* ------------------------------------------------------------------ */

/* Feed a buffer byte-by-byte; return the final parser state. */
static rc_parse_state_t feed(const uint8_t *buf, size_t len, packet_t *out)
{
    rc_parse_state_t st = RC_SYNC;
    for (size_t i = 0; i < len; i++)
        st = rc_parse_byte(buf[i], out, st);
    return st;
}

static void test_serialize_parse_roundtrip(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id = OBC_ID;
    tx.target_id = HYDRA_LF_ID;
    tx.cmd = CMD_MANUAL_EXEC;
    tx.payload_size = 3;
    tx.payload[0] = 0xDE;
    tx.payload[1] = 0xAD;
    tx.payload[2] = 0xBE;
    tx.crc = 0x1234;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT(HEADER_SIZE + 3 + CRC_SIZE, n);
    TEST_ASSERT_EQUAL_HEX8(SYNC_BYTE, buf[0]);

    packet_t rx;
    rc_parse_state_t st = feed(buf, n, &rx);

    TEST_ASSERT_EQUAL_INT(RC_END, st);
    TEST_ASSERT_EQUAL_UINT8(tx.sender_id, rx.sender_id);
    TEST_ASSERT_EQUAL_UINT8(tx.target_id, rx.target_id);
    TEST_ASSERT_EQUAL_UINT8(tx.cmd, rx.cmd);
    TEST_ASSERT_EQUAL_UINT8(tx.payload_size, rx.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tx.payload, rx.payload, tx.payload_size);
    TEST_ASSERT_EQUAL_UINT16(tx.crc, rx.crc);
}

static void test_zero_payload_roundtrip(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id = HYDRA_FS_ID;
    tx.target_id = OBC_ID;
    tx.cmd = CMD_STATUS;
    tx.payload_size = 0;
    tx.crc = 0xABCD;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT(HEADER_SIZE + CRC_SIZE, n);

    packet_t rx;
    TEST_ASSERT_EQUAL_INT(RC_END, feed(buf, n, &rx));
    TEST_ASSERT_EQUAL_UINT8(CMD_STATUS, rx.cmd);
    TEST_ASSERT_EQUAL_UINT8(0, rx.payload_size);
    TEST_ASSERT_EQUAL_UINT16(0xABCD, rx.crc);
}

/* Garbage before SYNC must be ignored, then a real frame parses cleanly. */
static void test_resync_after_garbage(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id = OBC_ID;
    tx.target_id = LIFT_TANK_ID;
    tx.cmd = CMD_STATUS;
    tx.payload_size = 1;
    tx.payload[0] = 0x42;
    tx.crc = 0x0001;

    uint8_t frame[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, frame, sizeof(frame));

    uint8_t stream[8 + MAX_FRAME_SIZE];
    size_t s = 0;
    stream[s++] = 0x00;
    stream[s++] = 0xFF;
    stream[s++] = 0x13; // junk, no SYNC byte
    memcpy(&stream[s], frame, n);
    s += n;

    packet_t rx;
    TEST_ASSERT_EQUAL_INT(RC_END, feed(stream, s, &rx));
    TEST_ASSERT_EQUAL_UINT8(LIFT_TANK_ID, rx.target_id);
    TEST_ASSERT_EQUAL_UINT8(0x42, rx.payload[0]);
}

/* A payload_size byte larger than capacity must be clamped, never overflow. */
static void test_payload_size_clamped(void)
{
    packet_t rx;
    rc_parse_state_t st = RC_SYNC;
    st = rc_parse_byte(SYNC_BYTE, &rx, st);
    st = rc_parse_byte(OBC_ID, &rx, st);
    st = rc_parse_byte(HYDRA_UF_ID, &rx, st);
    st = rc_parse_byte(CMD_STATUS, &rx, st);
    st = rc_parse_byte(0xFF, &rx, st); // claims 255 byte payload (> MAX_PAYLOAD_SIZE)

    TEST_ASSERT_EQUAL_INT(RC_PAYLOAD, st);
    TEST_ASSERT_TRUE(rx.payload_size <= MAX_PAYLOAD_SIZE);
}

static void test_serialize_rejects_small_buffer(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.payload_size = 4;
    uint8_t tiny[3];
    TEST_ASSERT_EQUAL_UINT(0, rc_serialize(&tx, tiny, sizeof(tiny)));
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_command_values_pinned);
    RUN_TEST(test_manual_command_values_pinned);
    RUN_TEST(test_board_ids_pinned);
    RUN_TEST(test_state_values_pinned);
    RUN_TEST(test_serialize_parse_roundtrip);
    RUN_TEST(test_zero_payload_roundtrip);
    RUN_TEST(test_resync_after_garbage);
    RUN_TEST(test_payload_size_clamped);
    RUN_TEST(test_serialize_rejects_small_buffer);
    return UNITY_END();
}
