/**
 * Host-side (native) unit tests for lib/rocket_core protocol layer.
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
    TEST_ASSERT_EQUAL_INT(4, CMD_ARM);
    TEST_ASSERT_EQUAL_INT(5, CMD_FIRE);
    TEST_ASSERT_EQUAL_INT(6, CMD_FILL_EXEC);
    TEST_ASSERT_EQUAL_INT(7, CMD_MANUAL_EXEC);
    TEST_ASSERT_EQUAL_INT(8, CMD_ACK);
    TEST_ASSERT_EQUAL_INT(9, CMD_NACK);
    TEST_ASSERT_EQUAL_INT(10, CMD_COUNT);
}

static void test_manual_command_values_pinned(void)
{
    TEST_ASSERT_EQUAL_INT(0, CMD_MANUAL_SD_LOG_START);
    TEST_ASSERT_EQUAL_INT(1, CMD_MANUAL_SD_LOG_STOP);
    TEST_ASSERT_EQUAL_INT(2, CMD_MANUAL_SD_STATUS);
    TEST_ASSERT_EQUAL_INT(3, CMD_MANUAL_VALVE_STATE);
    TEST_ASSERT_EQUAL_INT(4, CMD_MANUAL_VALVE_MS);
    TEST_ASSERT_EQUAL_INT(5, CMD_MANUAL_ACK);
    TEST_ASSERT_EQUAL_INT(6, manual_cmd_size);
}

static void test_board_ids_pinned(void)
{
    TEST_ASSERT_EQUAL_INT(0, MISSION_CONTROL_ID);
    TEST_ASSERT_EQUAL_INT(1, CORTEX_ID);
    TEST_ASSERT_EQUAL_INT(2, HYDRA_UF_ID);
    TEST_ASSERT_EQUAL_INT(3, HYDRA_LF_ID);
    TEST_ASSERT_EQUAL_INT(4, HYDRA_FS_ID);
    TEST_ASSERT_EQUAL_INT(5, NAVIGATOR_ID);
    TEST_ASSERT_EQUAL_INT(6, LIFT_TANK_ID);
    TEST_ASSERT_EQUAL_INT(7, LIFT_BOTTLE_ID);
    TEST_ASSERT_EQUAL_INT(8, LIFT_THRUST_ID);
    TEST_ASSERT_EQUAL_INT(0xFF, BROADCAST_ID);
}

static void test_board_id_aliases(void)
{
    TEST_ASSERT_EQUAL_INT(MISSION_CONTROL_ID, GROUND_ID);
    TEST_ASSERT_EQUAL_INT(CORTEX_ID, OBC_ID);
}

static void test_state_values_pinned(void)
{
    TEST_ASSERT_EQUAL_INT(0, IDLE);
    TEST_ASSERT_EQUAL_INT(13, ABORT);
    TEST_ASSERT_EQUAL_INT(14, state_count);
    TEST_ASSERT_EQUAL_INT(-1, S_NONE);
}

/* ------------------------------------------------------------------ */
/* Wire format constants                                               */
/* ------------------------------------------------------------------ */
static void test_frame_size_constants(void)
{
    TEST_ASSERT_EQUAL_INT(5,   HEADER_SIZE);
    TEST_ASSERT_EQUAL_INT(2,   CRC_SIZE);
    TEST_ASSERT_EQUAL_INT(150, MAX_PAYLOAD_SIZE);
    TEST_ASSERT_EQUAL_INT(HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE, MAX_FRAME_SIZE);
    TEST_ASSERT_EQUAL_INT(157, MAX_FRAME_SIZE);
}

static void test_sync_byte_value(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x55, SYNC_BYTE);
}

/* packet_t fields: sender_id(1) + target_id(1) + cmd(1) + payload_size(1)
 * + payload[150] + crc(2) + data_recv(1) = 157 bytes (packed, no sync byte). */
static void test_packet_struct_layout(void)
{
    TEST_ASSERT_EQUAL_INT(157, (int)sizeof(packet_t));
}

/* ------------------------------------------------------------------ */
/* Frame parser / serializer                                           */
/* ------------------------------------------------------------------ */

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

static void test_payload_size_clamped(void)
{
    packet_t rx;
    rc_parse_state_t st = RC_SYNC;
    st = rc_parse_byte(SYNC_BYTE, &rx, st);
    st = rc_parse_byte(OBC_ID, &rx, st);
    st = rc_parse_byte(HYDRA_UF_ID, &rx, st);
    st = rc_parse_byte(CMD_STATUS, &rx, st);
    st = rc_parse_byte(0xFF, &rx, st); // claims 255 bytes (> MAX_PAYLOAD_SIZE)

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

/* ------------------------------------------------------------------ */
/* Additional serializer edge cases                                    */
/* ------------------------------------------------------------------ */

static void test_serialize_null_buffer_returns_zero(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.payload_size = 0;
    TEST_ASSERT_EQUAL_UINT(0, rc_serialize(&tx, NULL, 100));
}

static void test_serialize_exact_fit_buffer(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id    = CORTEX_ID;
    tx.target_id    = HYDRA_UF_ID;
    tx.cmd          = CMD_STATUS;
    tx.payload_size = 2;
    tx.payload[0]   = 0xAA;
    tx.payload[1]   = 0xBB;
    tx.crc          = 0x5566;

    size_t needed = HEADER_SIZE + 2 + CRC_SIZE;
    uint8_t buf[HEADER_SIZE + 2 + CRC_SIZE];
    size_t n = rc_serialize(&tx, buf, needed);
    TEST_ASSERT_EQUAL_UINT(needed, n);

    packet_t rx;
    TEST_ASSERT_EQUAL_INT(RC_END, feed(buf, n, &rx));
    TEST_ASSERT_EQUAL_UINT8(0xAA, rx.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(0xBB, rx.payload[1]);
}

static void test_serialize_max_payload(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id    = CORTEX_ID;
    tx.target_id    = BROADCAST_ID;
    tx.cmd          = CMD_STATUS;
    tx.payload_size = MAX_PAYLOAD_SIZE;
    for (int i = 0; i < MAX_PAYLOAD_SIZE; i++)
        tx.payload[i] = (uint8_t)(i & 0xFF);
    tx.crc = 0x0000;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_UINT(MAX_FRAME_SIZE, n);

    packet_t rx;
    TEST_ASSERT_EQUAL_INT(RC_END, feed(buf, n, &rx));
    TEST_ASSERT_EQUAL_UINT8(MAX_PAYLOAD_SIZE, rx.payload_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tx.payload, rx.payload, MAX_PAYLOAD_SIZE);
}

static void test_serialize_oversized_payload_clamped(void)
{
    /* rc_serialize silently clamps payload_size to MAX_PAYLOAD_SIZE */
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.payload_size = 200; // > MAX_PAYLOAD_SIZE
    tx.crc          = 0;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    /* It should succeed, writing MAX_PAYLOAD_SIZE payload bytes */
    TEST_ASSERT_EQUAL_UINT(MAX_FRAME_SIZE, n);
    /* The wire payload_size byte must reflect the clamped size */
    TEST_ASSERT_EQUAL_UINT8(MAX_PAYLOAD_SIZE, buf[4]);
}

/* ------------------------------------------------------------------ */
/* CRC byte ordering in the wire buffer                                */
/* ------------------------------------------------------------------ */

static void test_crc_bytes_big_endian_in_wire_buffer(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id    = CORTEX_ID;
    tx.target_id    = OBC_ID;
    tx.cmd          = CMD_ACK;
    tx.payload_size = 0;
    tx.crc          = 0xBEEF;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    /* Frame: [SYNC][sender][target][cmd][size][crc_hi][crc_lo] */
    TEST_ASSERT_EQUAL_HEX8(0xBE, buf[n - 2]); // hi byte first
    TEST_ASSERT_EQUAL_HEX8(0xEF, buf[n - 1]); // lo byte second
}

static void test_crc_bytes_reassembled_correctly(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.payload_size = 0;
    tx.crc          = 0x1234;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));

    packet_t rx;
    feed(buf, n, &rx);
    TEST_ASSERT_EQUAL_UINT16(0x1234, rx.crc);
}

/* ------------------------------------------------------------------ */
/* Multi-frame streaming                                               */
/* ------------------------------------------------------------------ */

static void test_back_to_back_frames(void)
{
    packet_t tx1, tx2;
    memset(&tx1, 0, sizeof(tx1));
    memset(&tx2, 0, sizeof(tx2));

    tx1.sender_id = CORTEX_ID; tx1.target_id = HYDRA_UF_ID;
    tx1.cmd = CMD_STATUS; tx1.payload_size = 1; tx1.payload[0] = 0x11;

    tx2.sender_id = CORTEX_ID; tx2.target_id = HYDRA_LF_ID;
    tx2.cmd = CMD_ACK;    tx2.payload_size = 1; tx2.payload[0] = 0x22;

    uint8_t stream[2 * MAX_FRAME_SIZE];
    size_t n1 = rc_serialize(&tx1, stream,      sizeof(stream));
    size_t n2 = rc_serialize(&tx2, stream + n1, sizeof(stream) - n1);

    rc_parse_state_t st = RC_SYNC;
    packet_t rx;
    size_t total = n1 + n2;

    /* Feed first frame */
    for (size_t i = 0; i < n1; i++)
        st = rc_parse_byte(stream[i], &rx, st);
    TEST_ASSERT_EQUAL_INT(RC_END, st);
    TEST_ASSERT_EQUAL_UINT8(0x11, rx.payload[0]);

    /* After RC_END the caller resets to RC_SYNC before feeding next frame */
    st = RC_SYNC;
    for (size_t i = n1; i < total; i++)
        st = rc_parse_byte(stream[i], &rx, st);
    TEST_ASSERT_EQUAL_INT(RC_END, st);
    TEST_ASSERT_EQUAL_UINT8(0x22, rx.payload[0]);
}

static void test_sync_byte_inside_payload_not_restarted(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id    = CORTEX_ID;
    tx.target_id    = HYDRA_FS_ID;
    tx.cmd          = CMD_MANUAL_EXEC;
    tx.payload_size = 5;
    tx.payload[0]   = SYNC_BYTE; // 0x55 in payload — must NOT restart parser
    tx.payload[1]   = 0x01;
    tx.payload[2]   = 0x02;
    tx.payload[3]   = 0x03;
    tx.payload[4]   = 0x04;
    tx.crc          = 0x0000;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));

    packet_t rx;
    rc_parse_state_t st = feed(buf, n, &rx);
    TEST_ASSERT_EQUAL_INT(RC_END, st);
    TEST_ASSERT_EQUAL_UINT8(5, rx.payload_size);
    TEST_ASSERT_EQUAL_HEX8(SYNC_BYTE, rx.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(0x04, rx.payload[4]);
}

static void test_broadcast_target_id(void)
{
    packet_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.sender_id    = CORTEX_ID;
    tx.target_id    = BROADCAST_ID;
    tx.cmd          = CMD_STATUS;
    tx.payload_size = 0;
    tx.crc          = 0xFFFF;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));

    packet_t rx;
    TEST_ASSERT_EQUAL_INT(RC_END, feed(buf, n, &rx));
    TEST_ASSERT_EQUAL_HEX8(BROADCAST_ID, rx.target_id);
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, rx.crc);
}

static void test_rc_end_state_resets_on_next_byte(void)
{
    /* After reaching RC_END, feeding another byte that is not SYNC resets to
     * RC_SYNC (the default branch in rc_parse_byte). */
    packet_t tx, rx;
    memset(&tx, 0, sizeof(tx));
    tx.payload_size = 0;
    tx.crc          = 0;

    uint8_t buf[MAX_FRAME_SIZE];
    size_t n = rc_serialize(&tx, buf, sizeof(buf));
    rc_parse_state_t st = feed(buf, n, &rx);
    TEST_ASSERT_EQUAL_INT(RC_END, st);

    /* Extra non-SYNC byte goes through default branch → RC_SYNC */
    st = rc_parse_byte(0xAB, &rx, st);
    TEST_ASSERT_EQUAL_INT(RC_SYNC, st);
}

/* ------------------------------------------------------------------ */
/* rc_states.h (shared state enum, not CORTEX-specific RocketState)   */
/* ------------------------------------------------------------------ */
static void test_shared_state_enum_count(void)
{
    TEST_ASSERT_EQUAL_INT(14, state_count);
    TEST_ASSERT_EQUAL_INT(-1, S_NONE);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();

    /* Enum pinning */
    RUN_TEST(test_command_values_pinned);
    RUN_TEST(test_manual_command_values_pinned);
    RUN_TEST(test_board_ids_pinned);
    RUN_TEST(test_board_id_aliases);
    RUN_TEST(test_state_values_pinned);
    RUN_TEST(test_shared_state_enum_count);

    /* Wire format constants */
    RUN_TEST(test_frame_size_constants);
    RUN_TEST(test_sync_byte_value);
    RUN_TEST(test_packet_struct_layout);

    /* Core roundtrip */
    RUN_TEST(test_serialize_parse_roundtrip);
    RUN_TEST(test_zero_payload_roundtrip);
    RUN_TEST(test_resync_after_garbage);
    RUN_TEST(test_payload_size_clamped);
    RUN_TEST(test_serialize_rejects_small_buffer);

    /* Serializer edge cases */
    RUN_TEST(test_serialize_null_buffer_returns_zero);
    RUN_TEST(test_serialize_exact_fit_buffer);
    RUN_TEST(test_serialize_max_payload);
    RUN_TEST(test_serialize_oversized_payload_clamped);

    /* CRC byte ordering */
    RUN_TEST(test_crc_bytes_big_endian_in_wire_buffer);
    RUN_TEST(test_crc_bytes_reassembled_correctly);

    /* Multi-frame streaming */
    RUN_TEST(test_back_to_back_frames);
    RUN_TEST(test_sync_byte_inside_payload_not_restarted);
    RUN_TEST(test_broadcast_target_id);
    RUN_TEST(test_rc_end_state_resets_on_next_byte);

    return UNITY_END();
}
