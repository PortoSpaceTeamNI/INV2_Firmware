#include "Peripherals/Loadcells.h"
#include "Peripherals/IO_Map.h"
#include "Comms.h"

HX711 loadcell1, loadcell2, loadcell3;

int loadcells_setup(void)
{
  if (DEFAULT_ID == LIFT_THRUST_ID) {
    loadcell1.begin(LIFT_THRUST_DOUT1_PIN, LIFT_THRUST_SCK1_PIN);
    loadcell2.begin(LIFT_THRUST_DOUT2_PIN, LIFT_THRUST_SCK2_PIN);
    loadcell3.begin(LIFT_THRUST_DOUT3_PIN, LIFT_THRUST_SCK3_PIN);
    loadcell1.tare(10); // tare with 10 samples
    loadcell2.tare(10);
    loadcell3.tare(10);
  }
  else if (DEFAULT_ID == LIFT_TANK_ID) {
    loadcell1.begin(LIFT_TANK_DOUT_PIN, LIFT_TANK_SCK_PIN);
    loadcell1.tare(10);
  }
  else if (DEFAULT_ID == LIFT_BOTTLE_ID) {
    // TODO
    /*
    loadcell1.begin(LIFT_BOTTLE_DOUT_PIN, LIFT_BOTTLE_SCK_PIN);
    loadcell1.tare(10);
    */
    return 0;
  }
  else {
    return 0;
  }
  return 0;
}

int read_loadcells(data_t *data)
{
  #if DEFAULT_ID == LIFT_THRUST_ID
    data->loadcells.loadcell1 = loadcell1.get_value(10) / 21;
    data->loadcells.loadcell2 = loadcell2.get_value(10) / 21;
    data->loadcells.loadcell3 = loadcell3.get_value(10) / 21;
    return 0;
  #elif DEFAULT_ID == LIFT_TANK_ID
    data->loadcells.loadcell1 = loadcell1.get_value(10) / 21;
    return 0;
  #elif DEFAULT_ID == LIFT_BOTTLE_ID
    data->loadcells.loadcell1 = loadcell1.get_value(10) / 21;
    return 0;
  #endif
    return -1; // error, no loadcells configured
}
