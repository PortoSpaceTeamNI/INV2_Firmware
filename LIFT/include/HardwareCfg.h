#ifndef HARDWARE_CFG_H
#define HARDWARE_CFG_H

// Serial settings
#define USB_BAUD_RATE 115200
#define RS485_BAUD_RATE 115200 

#define LOADCELL_POLL_INTERVAL 10  // milliseconds

#define MY_ID LIFT_THRUST_ID

// Uncomment to run interactive serial calibration on next boot.
// Comment out again and re-flash after calibration is done.
// #define CALIBRATE_LOADCELLS

#if MY_ID == LIFT_THRUST_ID
    #define LOADCELL1_SCALE 21
    #define LOADCELL2_SCALE 21
    #define LOADCELL3_SCALE 21
#endif

#endif // HARDWARE_CFG_H