/**
 * Host-side (native) unit tests for the pure state-machine transition logic
 * in lib/rocket_core/rc_state_machine.
 *
 * Run with:  pio test -e native
 */
#include <unity.h>
#include "rc_state_machine.h"

void setUp(void) {}
void tearDown(void) {}

/* ------------------------------------------------------------------ */
/* Enum value pinning — guards against any accidental reorder          */
/* ------------------------------------------------------------------ */

static void test_state_enum_values(void)
{
    TEST_ASSERT_EQUAL_INT(0,  IDLE);
    TEST_ASSERT_EQUAL_INT(1,  ABORT);
    TEST_ASSERT_EQUAL_INT(2,  FILL_N2);
    TEST_ASSERT_EQUAL_INT(3,  PRE_PRESSURIZE);
    TEST_ASSERT_EQUAL_INT(4,  FILL_OX);
    TEST_ASSERT_EQUAL_INT(5,  POST_PRESSURIZE);
    TEST_ASSERT_EQUAL_INT(6,  ARMED);
    TEST_ASSERT_EQUAL_INT(7,  IGNITION);
    TEST_ASSERT_EQUAL_INT(8,  LAUNCH);
    TEST_ASSERT_EQUAL_INT(9,  BOOST);
    TEST_ASSERT_EQUAL_INT(10, COAST);
    TEST_ASSERT_EQUAL_INT(11, DROGUE_DESCENT);
    TEST_ASSERT_EQUAL_INT(12, MAIN_DESCENT);
    TEST_ASSERT_EQUAL_INT(13, TOUCHDOWN);
    TEST_ASSERT_EQUAL_INT(14, _STATE_COUNT);
}

static void test_event_enum_values(void)
{
    TEST_ASSERT_EQUAL_INT(0,  EV_NONE);
    TEST_ASSERT_EQUAL_INT(1,  EV_DATA_UPDATE);
    TEST_ASSERT_EQUAL_INT(2,  EV_STOP_CMD);
    TEST_ASSERT_EQUAL_INT(3,  EV_ABORT_CMD);
    TEST_ASSERT_EQUAL_INT(4,  EV_FILL_N2_CMD);
    TEST_ASSERT_EQUAL_INT(5,  EV_PRE_PRESSURIZE_CMD);
    TEST_ASSERT_EQUAL_INT(6,  EV_FILL_OX_CMD);
    TEST_ASSERT_EQUAL_INT(7,  EV_POST_PRESSURIZE_CMD);
    TEST_ASSERT_EQUAL_INT(8,  EV_ARM_CMD);
    TEST_ASSERT_EQUAL_INT(9,  EV_FIRE_CMD);
    TEST_ASSERT_EQUAL_INT(10, EV_LAUNCH);
    TEST_ASSERT_EQUAL_INT(11, EV_TAKEOFF);
    TEST_ASSERT_EQUAL_INT(12, EV_BOOST_END);
    TEST_ASSERT_EQUAL_INT(13, EV_APOGEE);
    TEST_ASSERT_EQUAL_INT(14, EV_MAIN_ALTITUDE);
    TEST_ASSERT_EQUAL_INT(15, EV_TOUCHDOWN);
    TEST_ASSERT_EQUAL_INT(16, _EVENT_COUNT);
}

/* ------------------------------------------------------------------ */
/* EV_ABORT_CMD: must reach ABORT from every non-abort state           */
/* ------------------------------------------------------------------ */

static void test_abort_from_idle(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(IDLE, EV_ABORT_CMD));
}

static void test_abort_from_fill_n2(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(FILL_N2, EV_ABORT_CMD));
}

static void test_abort_from_pre_pressurize(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(PRE_PRESSURIZE, EV_ABORT_CMD));
}

static void test_abort_from_fill_ox(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(FILL_OX, EV_ABORT_CMD));
}

static void test_abort_from_post_pressurize(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(POST_PRESSURIZE, EV_ABORT_CMD));
}

static void test_abort_from_armed(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(ARMED, EV_ABORT_CMD));
}

static void test_abort_from_ignition(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(IGNITION, EV_ABORT_CMD));
}

static void test_abort_from_launch(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(LAUNCH, EV_ABORT_CMD));
}

static void test_abort_from_boost(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(BOOST, EV_ABORT_CMD));
}

static void test_abort_from_coast(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(COAST, EV_ABORT_CMD));
}

static void test_abort_from_drogue_descent(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(DROGUE_DESCENT, EV_ABORT_CMD));
}

static void test_abort_from_touchdown(void)
{
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(TOUCHDOWN, EV_ABORT_CMD));
}

/* ------------------------------------------------------------------ */
/* EV_STOP_CMD: must reach IDLE from any state                         */
/* ------------------------------------------------------------------ */

static void test_stop_from_idle(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(IDLE, EV_STOP_CMD));
}

static void test_stop_from_fill_n2(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(FILL_N2, EV_STOP_CMD));
}

static void test_stop_from_armed(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(ARMED, EV_STOP_CMD));
}

static void test_stop_from_ignition(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(IGNITION, EV_STOP_CMD));
}

static void test_stop_from_boost(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(BOOST, EV_STOP_CMD));
}

static void test_stop_from_touchdown(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(TOUCHDOWN, EV_STOP_CMD));
}

/* ------------------------------------------------------------------ */
/* Normal (sequential) filling and flight transitions                  */
/* ------------------------------------------------------------------ */

static void test_idle_to_fill_n2(void)
{
    TEST_ASSERT_EQUAL_INT(FILL_N2, rc_update_state(IDLE, EV_FILL_N2_CMD));
}

static void test_idle_to_pre_pressurize(void)
{
    /* CORTEX allows jumping directly to PRE_PRESSURIZE from IDLE */
    TEST_ASSERT_EQUAL_INT(PRE_PRESSURIZE, rc_update_state(IDLE, EV_PRE_PRESSURIZE_CMD));
}

static void test_idle_to_fill_ox(void)
{
    TEST_ASSERT_EQUAL_INT(FILL_OX, rc_update_state(IDLE, EV_FILL_OX_CMD));
}

static void test_idle_to_post_pressurize(void)
{
    TEST_ASSERT_EQUAL_INT(POST_PRESSURIZE, rc_update_state(IDLE, EV_POST_PRESSURIZE_CMD));
}

static void test_idle_to_armed(void)
{
    /* Jump directly to ARMED from IDLE */
    TEST_ASSERT_EQUAL_INT(ARMED, rc_update_state(IDLE, EV_ARM_CMD));
}

static void test_fill_n2_to_pre_pressurize(void)
{
    TEST_ASSERT_EQUAL_INT(PRE_PRESSURIZE, rc_update_state(FILL_N2, EV_PRE_PRESSURIZE_CMD));
}

static void test_pre_pressurize_to_fill_ox(void)
{
    TEST_ASSERT_EQUAL_INT(FILL_OX, rc_update_state(PRE_PRESSURIZE, EV_FILL_OX_CMD));
}

static void test_fill_ox_to_post_pressurize(void)
{
    TEST_ASSERT_EQUAL_INT(POST_PRESSURIZE, rc_update_state(FILL_OX, EV_POST_PRESSURIZE_CMD));
}

static void test_post_pressurize_to_armed(void)
{
    TEST_ASSERT_EQUAL_INT(ARMED, rc_update_state(POST_PRESSURIZE, EV_ARM_CMD));
}

static void test_armed_to_ignition(void)
{
    TEST_ASSERT_EQUAL_INT(IGNITION, rc_update_state(ARMED, EV_FIRE_CMD));
}

static void test_ignition_to_launch(void)
{
    TEST_ASSERT_EQUAL_INT(LAUNCH, rc_update_state(IGNITION, EV_LAUNCH));
}

static void test_launch_to_boost(void)
{
    TEST_ASSERT_EQUAL_INT(BOOST, rc_update_state(LAUNCH, EV_TAKEOFF));
}

static void test_boost_to_coast(void)
{
    TEST_ASSERT_EQUAL_INT(COAST, rc_update_state(BOOST, EV_BOOST_END));
}

static void test_coast_to_drogue_descent(void)
{
    TEST_ASSERT_EQUAL_INT(DROGUE_DESCENT, rc_update_state(COAST, EV_APOGEE));
}

static void test_drogue_descent_to_main_descent(void)
{
    TEST_ASSERT_EQUAL_INT(MAIN_DESCENT, rc_update_state(DROGUE_DESCENT, EV_MAIN_ALTITUDE));
}

static void test_main_descent_to_touchdown(void)
{
    TEST_ASSERT_EQUAL_INT(TOUCHDOWN, rc_update_state(MAIN_DESCENT, EV_TOUCHDOWN));
}

/* ------------------------------------------------------------------ */
/* Invalid / no-op transitions: state must not change                  */
/* ------------------------------------------------------------------ */

static void test_idle_ignores_fire(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(IDLE, EV_FIRE_CMD));
}

static void test_idle_ignores_none(void)
{
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(IDLE, EV_NONE));
}

static void test_armed_ignores_fill_n2(void)
{
    TEST_ASSERT_EQUAL_INT(ARMED, rc_update_state(ARMED, EV_FILL_N2_CMD));
}

static void test_armed_ignores_launch(void)
{
    TEST_ASSERT_EQUAL_INT(ARMED, rc_update_state(ARMED, EV_LAUNCH));
}

static void test_touchdown_ignores_all_flight_events(void)
{
    TEST_ASSERT_EQUAL_INT(TOUCHDOWN, rc_update_state(TOUCHDOWN, EV_LAUNCH));
    TEST_ASSERT_EQUAL_INT(TOUCHDOWN, rc_update_state(TOUCHDOWN, EV_TAKEOFF));
    TEST_ASSERT_EQUAL_INT(TOUCHDOWN, rc_update_state(TOUCHDOWN, EV_BOOST_END));
    TEST_ASSERT_EQUAL_INT(TOUCHDOWN, rc_update_state(TOUCHDOWN, EV_APOGEE));
    TEST_ASSERT_EQUAL_INT(TOUCHDOWN, rc_update_state(TOUCHDOWN, EV_MAIN_ALTITUDE));
}

static void test_abort_ignores_all_normal_events(void)
{
    /* Once in ABORT the only escape is an explicit STOP or another ABORT cmd */
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(ABORT, EV_FIRE_CMD));
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(ABORT, EV_ARM_CMD));
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(ABORT, EV_FILL_N2_CMD));
    TEST_ASSERT_EQUAL_INT(ABORT, rc_update_state(ABORT, EV_LAUNCH));
    /* STOP still resets to IDLE even from ABORT */
    TEST_ASSERT_EQUAL_INT(IDLE, rc_update_state(ABORT, EV_STOP_CMD));
}

static void test_fill_n2_ignores_fire_and_arm(void)
{
    TEST_ASSERT_EQUAL_INT(FILL_N2, rc_update_state(FILL_N2, EV_FIRE_CMD));
    TEST_ASSERT_EQUAL_INT(FILL_N2, rc_update_state(FILL_N2, EV_ARM_CMD));
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();

    /* Enum pinning */
    RUN_TEST(test_state_enum_values);
    RUN_TEST(test_event_enum_values);

    /* ABORT override */
    RUN_TEST(test_abort_from_idle);
    RUN_TEST(test_abort_from_fill_n2);
    RUN_TEST(test_abort_from_pre_pressurize);
    RUN_TEST(test_abort_from_fill_ox);
    RUN_TEST(test_abort_from_post_pressurize);
    RUN_TEST(test_abort_from_armed);
    RUN_TEST(test_abort_from_ignition);
    RUN_TEST(test_abort_from_launch);
    RUN_TEST(test_abort_from_boost);
    RUN_TEST(test_abort_from_coast);
    RUN_TEST(test_abort_from_drogue_descent);
    RUN_TEST(test_abort_from_touchdown);

    /* STOP override */
    RUN_TEST(test_stop_from_idle);
    RUN_TEST(test_stop_from_fill_n2);
    RUN_TEST(test_stop_from_armed);
    RUN_TEST(test_stop_from_ignition);
    RUN_TEST(test_stop_from_boost);
    RUN_TEST(test_stop_from_touchdown);

    /* Normal sequential transitions */
    RUN_TEST(test_idle_to_fill_n2);
    RUN_TEST(test_idle_to_pre_pressurize);
    RUN_TEST(test_idle_to_fill_ox);
    RUN_TEST(test_idle_to_post_pressurize);
    RUN_TEST(test_idle_to_armed);
    RUN_TEST(test_fill_n2_to_pre_pressurize);
    RUN_TEST(test_pre_pressurize_to_fill_ox);
    RUN_TEST(test_fill_ox_to_post_pressurize);
    RUN_TEST(test_post_pressurize_to_armed);
    RUN_TEST(test_armed_to_ignition);
    RUN_TEST(test_ignition_to_launch);
    RUN_TEST(test_launch_to_boost);
    RUN_TEST(test_boost_to_coast);
    RUN_TEST(test_coast_to_drogue_descent);
    RUN_TEST(test_drogue_descent_to_main_descent);
    RUN_TEST(test_main_descent_to_touchdown);

    /* No-op / invalid transitions */
    RUN_TEST(test_idle_ignores_fire);
    RUN_TEST(test_idle_ignores_none);
    RUN_TEST(test_armed_ignores_fill_n2);
    RUN_TEST(test_armed_ignores_launch);
    RUN_TEST(test_touchdown_ignores_all_flight_events);
    RUN_TEST(test_abort_ignores_all_normal_events);
    RUN_TEST(test_fill_n2_ignores_fire_and_arm);

    return UNITY_END();
}
