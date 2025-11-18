#define EXT_LED 6
#define LASER1 45
#define LASER2 46
#define BUZZER 3
#define BATT_SENSE 1
#define EXT_5V_SENSE 34
#define STAT_CHARGER 2
#define SCL 4
#define SCA 5
#define SCL_DISPLAY 9
#define SCA_DISPLAY 8
#define H_CSN 10
#define H_MOSI 11
#define H_SCL 12
#define H_SDA 13
#define PS0 14
#define PS1 15
#define CLKSEL0 16
#define ONBOARDLED 17
#define XSHUT 47
#define GPIO1 33
#define BOOTN 36
#define NRST 39
#define PB_SWITCH 38
#define PB_ENCODER 37
#define CLK_ENCODER 36
#define DT_ENCODER 35  


/**
 * @brief The baud rate for serial communication.
 *
 * This constant sets the data transfer rate for the serial port,
 */
const unsigned long SERIAL_BAUD = 115200;

/**
 * @brief The delay in milliseconds after serial communication is initialized.
 *
 * This delay provides a brief pause after setting up the serial port,
 * allowing the serial connection to stabilize before any data transmission begins.
 * It can be useful to ensure that the connected device is ready to receive commands or data.
 */
const unsigned long DELAY_AFTER_SERIAL_INITIALIZED = 100;

/**
 * @brief The name of the BLE server/device.
 *
 * This name is displayed when scanning for BLE devices.
 */
const char *BLE_SERVER_NAME = "HermaTech";


/**
 * @brief A GATT characteristic that can publish sensor data.
 */
const char *TELEMETRY_UUID = "38938a29-7798-4dd6-bca2-c8fc70f65bfc";


/**
 * @brief A GATT characteristic that receive commands
 */
const char *COMMANDS_UUID = "f7a40c1c-e012-4c18-b838-51de426a868c";


//////////////////////////////////////////////////////////////



/**
 * @brief A GATT characteristic that can publish geomagnetic data.
 */
const char *MAGNETOMETER_UUID = "5d08a535-513b-47ce-8b28-7178dcfb69aa";