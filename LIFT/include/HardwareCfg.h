#ifndef HARDWARE_CFG_H
#define HARDWARE_CFG_H

// Serial settings
#define USB_BAUD_RATE 115200
#define RS485_BAUD_RATE 115200 

#define LOADCELL_POLL_INTERVAL 10  // milliseconds
#define SD_LOG_INTERVAL_MS 50  // milliseconds

#define MY_ID LIFT_BOTTLE_ID

// Uncomment to run interactive serial calibration on next boot.
// Comment out again and re-flash after calibration is done.
//#define CALIBRATE_LOADCELLS

#define THRUST_LOADCELL1_SCALE 21
#define THRUST_LOADCELL2_SCALE 21
#define THRUST_LOADCELL3_SCALE 21
#define BOTTLE_LOADCELL1_SCALE -2.2

#endif // HARDWARE_CFG_H