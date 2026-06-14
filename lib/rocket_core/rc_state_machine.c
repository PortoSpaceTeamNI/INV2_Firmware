#include "rc_state_machine.h"

RocketState rc_update_state(RocketState currentState, RocketEvent event)
{
    if (event == EV_ABORT_CMD)
        return ABORT;
    else if (event == EV_STOP_CMD)
        return IDLE;

    switch (currentState)
    {
    case IDLE:
        if (event == EV_FILL_N2_CMD)
            return FILL_N2;
        else if (event == EV_PRE_PRESSURIZE_CMD)
            return PRE_PRESSURIZE;
        else if (event == EV_FILL_OX_CMD)
            return FILL_OX;
        else if (event == EV_POST_PRESSURIZE_CMD)
            return POST_PRESSURIZE;
        else if (event == EV_ARM_CMD)
            return ARMED;
        break;
    case FILL_N2:
        if (event == EV_PRE_PRESSURIZE_CMD)
            return PRE_PRESSURIZE;
        break;
    case PRE_PRESSURIZE:
        if (event == EV_FILL_OX_CMD)
            return FILL_OX;
        break;
    case FILL_OX:
        if (event == EV_POST_PRESSURIZE_CMD)
            return POST_PRESSURIZE;
        break;
    case POST_PRESSURIZE:
        if (event == EV_ARM_CMD)
            return ARMED;
        break;
    case ARMED:
        if (event == EV_FIRE_CMD)
            return IGNITION;
        break;
    case IGNITION:
        if (event == EV_LAUNCH)
            return LAUNCH;
        break;
    case LAUNCH:
        if (event == EV_TAKEOFF)
            return BOOST;
        break;
    case BOOST:
        if (event == EV_BOOST_END)
            return COAST;
        break;
    case COAST:
        if (event == EV_APOGEE)
            return DROGUE_DESCENT;
        break;
    case DROGUE_DESCENT:
        if (event == EV_MAIN_ALTITUDE)
            return MAIN_DESCENT;
        break;
    case MAIN_DESCENT:
        if (event == EV_TOUCHDOWN)
            return TOUCHDOWN;
        break;
    default:
        break;
    }
    return currentState;
}
