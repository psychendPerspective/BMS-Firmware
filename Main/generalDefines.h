// Define the hardware version here:

#ifndef XANADU_LEV
#define XANADU_LEV					    0
#endif

#ifndef XANADU_HV_EV
#define XANADU_HV_EV					1
#endif


// Firmware version
#define FW_VERSION_MAJOR				5
#define FW_VERSION_MINOR				4
#define FW_REAL_VERSION					"V5.4"
#define FW_TEST_VERSION_NUMBER			0
// UUID
#define STM32_UUID				((uint32_t*)0x1FFFF7AC)
#define STM32_UUID_8			((uint8_t*)0x1FFFF7AC)

// Hardware names and limits
#if XANADU_LEV
  	#define HW_NAME 										"XANADU_LEV"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE  18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                    18
	#define HW_LIM_MIN_NOT_USED_DELAY                       5000
	#define HAS_ON_BOARD_NTC				                1	   //has on-board NTC
	#define ISL28022_MASTER_ADDRES				            0x40
	#define HAS_PFET_OUTPUT                                 1
	#define HAS_DISCHARGE					                1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			        0
	#define HAS_COMMON_CHARGE_DISCHARGE_OPTION		        1
	#define HAS_EXTERNAL_VOLTAGE_MEASUREMENT				0      //has isolated Op-amp(AMC3302/ISL)
	#define BMS_16S_CONFIG                                  0
	#define BMS_BOARD_NTC									0      //has tempertaure sensors(NTC) on the BMS board
	#define HAS_COOLING                                     0
	#define HAS_HUMIDITY                                    0
	#define HAS_NO_DISCHARGE                                0

#elif XANADU_HV_EV
  	#define HW_NAME 										"XANADU_HV_EV"
	#define HW_LIM_CELL_BALANCE_MAX_SIMULTANEOUS_DISCHARGE  18
	#define HW_LIM_CELL_MONITOR_IC_COUNT                    18
	#define HW_LIM_MIN_NOT_USED_DELAY                       5000
	#define HAS_ON_BOARD_NTC				                0	   //has on-board NTC
	#define ISL28022_MASTER_ADDRES				            0
	#define HAS_PFET_OUTPUT                                 0
	#define HAS_DISCHARGE					                1
	#define HAS_CHARGER_VOLTAGE_MEASUREMENT			        0
	#define HAS_COMMON_CHARGE_DISCHARGE_OPTION		        0
	#define HAS_EXTERNAL_VOLTAGE_MEASUREMENT				0      //has isolated Op-amp(AMC3302/ISL)
	#define BMS_16S_CONFIG                                  1
	#define BMS_BOARD_NTC									0      //has tempertaure sensors(NTC) on the BMS board
	#define HAS_COOLING                                     1
	#define BMS_20S_CONFIG                                  0
#endif

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
