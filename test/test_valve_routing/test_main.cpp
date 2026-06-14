/**
 * Host-side (native) unit tests for the pure valve routing table in
 * lib/rocket_core/rc_valve_routing.
 *
 * Run with:  pio test -e native
 */
#include <unity.h>
#include "rc_valve_routing.h"
#include "rc_ids.h"

void setUp(void) {}
void tearDown(void) {}

/* ------------------------------------------------------------------ */
/* Enum counts (guards against accidental reorder / insertion)         */
/* ------------------------------------------------------------------ */

static void test_valve_count_is_10(void)
{
    TEST_ASSERT_EQUAL_INT(10, valve_count);
}

static void test_h_valve_count_is_8(void)
{
    TEST_ASSERT_EQUAL_INT(8, h_valve_count);
}

/* ------------------------------------------------------------------ */
/* Error cases                                                         */
/* ------------------------------------------------------------------ */

static void test_null_route_returns_minus1(void)
{
    TEST_ASSERT_EQUAL_INT(-1, rc_get_valve_route(VALVE_ABORT, NULL));
}

static void test_invalid_valve_returns_minus1(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(-1, rc_get_valve_route((valve_t)valve_count, &r));
}

static void test_negative_valve_returns_minus1(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(-1, rc_get_valve_route((valve_t)-1, &r));
}

/* ------------------------------------------------------------------ */
/* All 10 valid logical valve routes                                   */
/* ------------------------------------------------------------------ */

static void test_valve_abort(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_ABORT, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_LF_ID,        r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_CONTROLLED_1, r.hydraValve);
}

static void test_valve_pressurizing(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_PRESSURIZING, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_LF_ID,        r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_CONTROLLED_3, r.hydraValve);
}

static void test_valve_main(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_MAIN, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_LF_ID,        r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_CONTROLLED_2, r.hydraValve);
}

static void test_valve_vent(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_VENT, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_UF_ID,        r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_CONTROLLED_1, r.hydraValve);
}

static void test_valve_n2_fill(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_N2_FILL, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_FS_ID,         r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_STEEL_BALL_2, r.hydraValve);
}

static void test_valve_n2_purge(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_N2_PURGE, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_FS_ID,         r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_STEEL_BALL_1, r.hydraValve);
}

static void test_valve_n2o_fill(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_N2O_FILL, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_FS_ID,   r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_SERVO, r.hydraValve);
}

static void test_valve_n2o_purge(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_N2O_PURGE, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_FS_ID,         r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_CONTROLLED_3, r.hydraValve);
}

static void test_valve_n2o_quick_dc(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_N2O_QUICK_DC, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_FS_ID,      r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_QUICK_DC_1, r.hydraValve);
}

static void test_valve_n2_quick_dc(void)
{
    ValveRoute r;
    TEST_ASSERT_EQUAL_INT(0, rc_get_valve_route(VALVE_N2_QUICK_DC, &r));
    TEST_ASSERT_EQUAL_INT(HYDRA_FS_ID,      r.hydraId);
    TEST_ASSERT_EQUAL_INT(H_VALVE_QUICK_DC_2, r.hydraValve);
}

/* ------------------------------------------------------------------ */
/* Sanity: LF node owns 3 valves; UF node owns 1; FS node owns 6      */
/* ------------------------------------------------------------------ */

static void test_lf_valves_are_abort_main_pressurizing(void)
{
    ValveRoute r;
    rc_get_valve_route(VALVE_ABORT,       &r); TEST_ASSERT_EQUAL_INT(HYDRA_LF_ID, r.hydraId);
    rc_get_valve_route(VALVE_MAIN,        &r); TEST_ASSERT_EQUAL_INT(HYDRA_LF_ID, r.hydraId);
    rc_get_valve_route(VALVE_PRESSURIZING,&r); TEST_ASSERT_EQUAL_INT(HYDRA_LF_ID, r.hydraId);
}

static void test_uf_valve_is_vent_only(void)
{
    /* Count valves mapping to HYDRA_UF_ID */
    int uf_count = 0;
    for (int v = 0; v < valve_count; v++) {
        ValveRoute r;
        if (rc_get_valve_route((valve_t)v, &r) == 0 && r.hydraId == HYDRA_UF_ID)
            uf_count++;
    }
    TEST_ASSERT_EQUAL_INT(1, uf_count);
}

static void test_fs_owns_six_valves(void)
{
    int fs_count = 0;
    for (int v = 0; v < valve_count; v++) {
        ValveRoute r;
        if (rc_get_valve_route((valve_t)v, &r) == 0 && r.hydraId == HYDRA_FS_ID)
            fs_count++;
    }
    TEST_ASSERT_EQUAL_INT(6, fs_count);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();

    /* Enum guards */
    RUN_TEST(test_valve_count_is_10);
    RUN_TEST(test_h_valve_count_is_8);

    /* Error cases */
    RUN_TEST(test_null_route_returns_minus1);
    RUN_TEST(test_invalid_valve_returns_minus1);
    RUN_TEST(test_negative_valve_returns_minus1);

    /* All 10 valid valves */
    RUN_TEST(test_valve_abort);
    RUN_TEST(test_valve_pressurizing);
    RUN_TEST(test_valve_main);
    RUN_TEST(test_valve_vent);
    RUN_TEST(test_valve_n2_fill);
    RUN_TEST(test_valve_n2_purge);
    RUN_TEST(test_valve_n2o_fill);
    RUN_TEST(test_valve_n2o_purge);
    RUN_TEST(test_valve_n2o_quick_dc);
    RUN_TEST(test_valve_n2_quick_dc);

    /* Node distribution sanity */
    RUN_TEST(test_lf_valves_are_abort_main_pressurizing);
    RUN_TEST(test_uf_valve_is_vent_only);
    RUN_TEST(test_fs_owns_six_valves);

    return UNITY_END();
}
