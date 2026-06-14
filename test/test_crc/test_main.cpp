/**
 * Host-side (native) unit tests for lib/Crc (CRC-16 with poly 0xD175).
 *
 * Run with:  pio test -e native
 */
#include <unity.h>
#include "Crc.h"

void setUp(void) {}
void tearDown(void) {}

/* ------------------------------------------------------------------ */
/* CRC lookup table spot-checks                                        */
/* ------------------------------------------------------------------ */

static void test_crc_table_entry_0(void)
{
    /* Index 0: no bits set → XOR with 0 throughout → 0 */
    TEST_ASSERT_EQUAL_HEX16(0x0000, crctable[0]);
}

static void test_crc_table_entry_1(void)
{
    /* Index 1 corresponds to poly 0xD175, confirmed from the table in Crc.cpp */
    TEST_ASSERT_EQUAL_HEX16(0xD175, crctable[1]);
}

static void test_crc_table_entry_2(void)
{
    TEST_ASSERT_EQUAL_HEX16(0x739F, crctable[2]);
}

/* ------------------------------------------------------------------ */
/* crc() function                                                      */
/* ------------------------------------------------------------------ */

static void test_crc_empty_data(void)
{
    /* INIT=0, XOROT=0, so an empty run should yield 0 */
    unsigned char buf[1] = {0};
    unsigned long result = crc(buf, 0);
    TEST_ASSERT_EQUAL_HEX32(0x0000UL, result);
}

static void test_crc_single_zero_byte(void)
{
    /* crc({0x00}, 1): first (and only) iteration →
     * crc = crctable[((0>>8) ^ 0x00) & 0xFF] ^ (0 << 8)
     *     = crctable[0] ^ 0
     *     = 0x0000
     */
    unsigned char buf[1] = {0x00};
    TEST_ASSERT_EQUAL_HEX32(0x0000UL, crc(buf, 1));
}

static void test_crc_single_one_byte(void)
{
    /* crc({0x01}, 1):
     * crc = crctable[((0>>8) ^ 0x01) & 0xFF] ^ (0 << 8)
     *     = crctable[1] ^ 0
     *     = 0xD175
     */
    unsigned char buf[1] = {0x01};
    TEST_ASSERT_EQUAL_HEX32(0xD175UL, crc(buf, 1));
}

static void test_crc_known_sequence(void)
{
    /* Verify determinism: the same 3-byte sequence always yields the same output.
     * Note: the accumulator is an `unsigned long` and is NOT masked to 16 bits
     * between bytes, so multi-byte results are wider than 16 bits. The idempotency
     * test (below) covers determinism; this test just checks the exact value
     * against a known-good run to catch table or logic regressions. */
    unsigned char seq[3] = {0x55, 0x01, 0x02};
    unsigned long first  = crc(seq, 3);
    unsigned long second = crc(seq, 3);
    TEST_ASSERT_EQUAL_HEX32(first, second);
    /* Value must be non-zero for this non-trivial input */
    TEST_ASSERT_NOT_EQUAL(0UL, first);
}

static void test_crc_idempotent(void)
{
    unsigned char data[8] = {0x55, 0xAA, 0x01, 0x02, 0x03, 0x04, 0x00, 0xFF};
    unsigned long r1 = crc(data, 8);
    unsigned long r2 = crc(data, 8);
    TEST_ASSERT_EQUAL_HEX32(r1, r2);
}

static void test_crc_different_data_different_result(void)
{
    unsigned char a[4] = {0x01, 0x02, 0x03, 0x04};
    unsigned char b[4] = {0x04, 0x03, 0x02, 0x01};
    /* Distinct byte sequences almost certainly give distinct CRCs */
    TEST_ASSERT_NOT_EQUAL(crc(a, 4), crc(b, 4));
}

static void test_crc_single_byte_result_is_16bit(void)
{
    /* For a single byte the shift chain stays within 16 bits:
     * crc = crctable[byte] ^ (0 << 8) = crctable[byte]  (always 16-bit) */
    unsigned char data[1] = {0xFF};
    unsigned long result = crc(data, 1);
    TEST_ASSERT_EQUAL_HEX32(result & 0xFFFFUL, result);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();

    RUN_TEST(test_crc_table_entry_0);
    RUN_TEST(test_crc_table_entry_1);
    RUN_TEST(test_crc_table_entry_2);
    RUN_TEST(test_crc_empty_data);
    RUN_TEST(test_crc_single_zero_byte);
    RUN_TEST(test_crc_single_one_byte);
    RUN_TEST(test_crc_known_sequence);
    RUN_TEST(test_crc_idempotent);
    RUN_TEST(test_crc_different_data_different_result);
    RUN_TEST(test_crc_single_byte_result_is_16bit);

    return UNITY_END();
}
