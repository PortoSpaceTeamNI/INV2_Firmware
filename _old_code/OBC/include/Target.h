/*
    The version for hardware uses:
serial1(subc) for write
serial2(uart serial) for read
    For this to run just comment the #define DIGITAL_TARGET
*/

/*
    This compiles a version of the program using: 
 serial1(usbc) for both read and write operations     
    Do not use it while testing with the hardware
    For this to run just uncomment the #define DIGITAL_TARGET
 */

//#define DIGITAL_TARGET
#define LoRa_TARGET
//#define RS485_TARGET

