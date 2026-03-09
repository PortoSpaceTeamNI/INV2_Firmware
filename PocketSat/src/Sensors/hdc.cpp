#include "Sensors/hdc.h"
#include "Sensors.h"
#include <HDC302x.h>

HDC302x hdc = HDC302x();

int InitializeHDC3020()
{
    pinMode(HDC3020_RDY_PIN, INPUT);

    int ret = hdc.Initialize(HDC302X_ADDR);
    return ret ? 0 : -1; // Return 0 on success, -1 on failure
}

bool IsHDC3020Ready()
{
    return digitalRead(HDC3020_RDY_PIN) == HIGH;
}

int ReadHDC3020(HDC302xDataResult& result)
{ 
    result = hdc.ReadData();
    return 0;
}
