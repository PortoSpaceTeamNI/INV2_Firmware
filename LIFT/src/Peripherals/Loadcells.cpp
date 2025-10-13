#include "Peripherals/Loadcells.h"
#include "Peripherals/IO_Map.h"
#include "Comms.h"

HX711 loadcell1, loadcell2, loadcell3;

int loadcells_setup(void)
{
    loadcell1.begin(LIFT_THRUST_DOUT1_PIN, LIFT_THRUST_SCK1_PIN);
    loadcell2.begin(LIFT_THRUST_DOUT2_PIN, LIFT_THRUST_SCK2_PIN);
    loadcell3.begin(LIFT_THRUST_DOUT3_PIN, LIFT_THRUST_SCK3_PIN);
    loadcell1.tare(10); // tare with 10 samples
    loadcell2.tare(10);
    loadcell3.tare(10);
    return 0;
}

int read_loadcells(data_t *data)
{
    data->loadcells.loadcell1 = loadcell1.get_value() / 21;
    data->loadcells.loadcell2 = loadcell2.get_value() / 21;
    data->loadcells.loadcell3 = loadcell3.get_value() / 21;
    return 0;
}
