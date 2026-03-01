#include "Peripherals/Loadcells.h"
#include "Peripherals/IO_Map.h"
#include "Comms.h"

HX711 loadcell1, loadcell2, loadcell3;

int loadcells_setup(void)
{

    loadcell1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
    if (MY_ID == LIFT_THRUST_ID)
    {
        loadcell2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
        loadcell3.begin(LOADCELL3_DOUT_PIN, LOADCELL3_SCK_PIN);
    }
    
    loadcell1.tare(10); // tare with 10 samples
    if (MY_ID == LIFT_THRUST_ID)
    {
        loadcell2.tare(10);
        loadcell3.tare(10);
    }
    return 0;
}

int read_loadcells(data_t *data)
{
    data->loadcells.loadcell1 = loadcell1.get_value() / 21;
    if (MY_ID == LIFT_THRUST_ID)
    {
        data->loadcells.loadcell2 = loadcell2.get_value() / 21;
        data->loadcells.loadcell3 = loadcell3.get_value() / 21;
    }
    return 0;
}
