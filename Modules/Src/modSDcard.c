#include "modSDcard.h"
#include "modTerminal.h"

/*File system declerations */
FATFS fs;  // file system
FIL bms_log_file; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count


/**** SD card capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space; 
uint8_t sdCardStatus;

//Pack State and General Config State handlers
modPowerElectronicsPackStateTypedef *modSDcardPackStateHandle;
modConfigGeneralConfigStructTypedef *modSDcardGeneralConfigHandle;
uint32_t modSDcardLogIntervalTick;

//string buffer variables
#define BUFFER_SIZE 512
char stringBuffer[BUFFER_SIZE];

uint8_t modSDcard_Init(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer)
{
    modSDcardPackStateHandle = packState;
    modSDcardGeneralConfigHandle = generalConfigPointer;
    sdCardStatus = false;
    //init SPI2, FATFS driver and parameter config
    driverHWSPI2Init();
    MX_FATFS_Init();

    //mount SD card, check SD card mounting status and available space
	fresult = f_mount(&fs, "/", 1);
    if(fresult != FR_OK)
    {
        sdCardStatus = false;
        return sdCardStatus;
    }
    /* Check free space */
    f_getfree("", &fre_clust, &pfs);

    total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
    if(free_space < MIN_SDCARD_STORAGE_REQ)
    {
        sdCardStatus = false;
        return sdCardStatus;
    }

    #if BMS_16S_CONFIG

    /* Open file to write/ create a file if it doesn't exist */
    fresult = f_open(&bms_log_file, "XanaduBMS_Log_File.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    /* Move the offset to the end of the file */
    fresult = f_lseek(&bms_log_file, f_size(&bms_log_file));
    /* Write logging file header */
    f_puts("\n Timestamp(s),Pack_Voltage(V), Pack_Current(A),Pack_Power(W),SoC(percent),Remaining Capacity(Ah),Operational_State,Fault_State,Cycle_Count,SoH(percent),CV_Max(V),CV_Min(V),CV_Max_Imbalance(V),Battery_Temp_Max(degC),Battery_Temp_Min(degC),Battery_Temp_Avg(degC),CV_1(V),CV_2(V),CV_3(V),CV_4(V),CV_5(V),CV_6(V),CV_7(V),CV_8(V),CV_9(V),CV_10(V),CV_11(V),CV_12(V),CV_13(V),CV_14(V),CV_15(V),CV_16(V),CV_17(V),CV_18(V),Temp_1(degC),Temp_2(degC),Temp_3(degC),Temp_4(degC),Temp_5(degC),Temp_6(degC),Temp_7(degC)\n", &bms_log_file);
    /* Close the log file */
    fresult = f_close(&bms_log_file);
    if(fresult == FR_OK)
    {
        sdCardStatus = true;
    }

    #elif BMS_20S_CONFIG

    /* Open file to write/ create a file if it doesn't exist */
    fresult = f_open(&bms_log_file, "XanaduBMS_Log_File.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    /* Move the offset to the end of the file */
    fresult = f_lseek(&bms_log_file, f_size(&bms_log_file));
    /* Write logging file header */
    f_puts("\n Timestamp(s),Pack_Voltage(V), Pack_Current(A),Pack_Power(W),SoC(percent),Remaining Capacity(Ah),Operational_State,Fault_State,CV_Max(V),CV_Min(V),CV_Max_Imbalance(V),Battery_Temp_Max(degC),Battery_Temp_Min(degC),Battery_Temp_Avg(degC),CV_1(V),CV_2(V),CV_3(V),CV_4(V),CV_5(V),CV_6(V),CV_7(V),CV_8(V),CV_9(V),CV_10(V),CV_11(V),CV_12(V),CV_13(V),CV_14(V),CV_15(V),CV_16(V),CV_17(V),CV_18(V),CV_19(V),CV_20(V),Temp_1(degC),Temp_2(degC),Temp_3(degC),Temp_4(degC),Temp_5(degC),Temp_6(degC),Temp_7(degC),Temp_8(degC)\n", &bms_log_file);
    /* Close the log file */
    fresult = f_close(&bms_log_file);
    if(fresult == FR_OK)
    {
        sdCardStatus = true;
    }

    #endif

    return sdCardStatus;
};

//helper functions for string buffer
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
};

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) stringBuffer[i] = '\0';
};

void modSDcard_logtoCSV(void)
{
    if(modDelayTick1ms(&modSDcardLogIntervalTick, LOGGING_INTERVAL))
    {
        #if BMS_16S_CONFIG
        fresult = f_open(&bms_log_file, "XanaduBMS_Log_File.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
        if(fresult != FR_OK)
            return;
        fresult = f_lseek(&bms_log_file, f_size(&bms_log_file));
        if(fresult != FR_OK)
            return;
        snprintf(stringBuffer,BUFFER_SIZE, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \r\n",
                (HAL_GetTick()/1000.0), 
                modSDcardPackStateHandle->packVoltage,
                modSDcardPackStateHandle->packCurrent,
                modSDcardPackStateHandle->packPower,
                modSDcardPackStateHandle->SoC,
                modSDcardPackStateHandle->SoCCapacityAh,
                modSDcardPackStateHandle->operationalState,
                modSDcardPackStateHandle->faultState,
                modSDcardPackStateHandle->cycleCount,
                modSDcardPackStateHandle->SoH,
                modSDcardPackStateHandle->cellVoltageHigh,
                modSDcardPackStateHandle->cellVoltageLow,
                modSDcardPackStateHandle->cellVoltageMisMatch,
                modSDcardPackStateHandle->tempBatteryHigh,
                modSDcardPackStateHandle->tempBatteryLow,
                modSDcardPackStateHandle->tempBatteryAverage,
                modSDcardPackStateHandle->cellVoltagesIndividual[0].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[1].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[2].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[3].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[4].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[5].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[6].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[7].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[8].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[9].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[10].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[11].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[12].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[13].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[14].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[15].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[16].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[17].cellVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[2].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[3].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[4].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[5].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[6].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[7].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[8].auxVoltage);
        fresult = f_write(&bms_log_file,stringBuffer, bufsize(stringBuffer), &bw);
        if(fresult != FR_OK)
        {
            memset(stringBuffer, '\0' ,BUFFER_SIZE);
            return;
        }
        fresult = f_close(&bms_log_file);
        //clear_buffer();
        memset(stringBuffer, '\0', BUFFER_SIZE);
        return;

        #elif BMS_20S_CONFIG
        fresult = f_open(&bms_log_file, "XanaduBMS_Log_File.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
        if(fresult != FR_OK)
            return;
        fresult = f_lseek(&bms_log_file, f_size(&bms_log_file));
        if(fresult != FR_OK)
            return;
        snprintf(stringBuffer,BUFFER_SIZE, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f \r\n",
                (HAL_GetTick()/1000.0), 
                modSDcardPackStateHandle->packVoltage,
                modSDcardPackStateHandle->packCurrent,
                modSDcardPackStateHandle->packPower,
                modSDcardPackStateHandle->SoC,
                modSDcardPackStateHandle->SoCCapacityAh,
                modSDcardPackStateHandle->operationalState,
                modSDcardPackStateHandle->faultState,
                modSDcardPackStateHandle->cellVoltageHigh,
                modSDcardPackStateHandle->cellVoltageLow,
                modSDcardPackStateHandle->cellVoltageMisMatch,
                modSDcardPackStateHandle->tempBatteryHigh,
                modSDcardPackStateHandle->tempBatteryLow,
                modSDcardPackStateHandle->tempBatteryAverage,
                modSDcardPackStateHandle->cellVoltagesIndividual[0].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[1].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[2].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[3].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[4].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[5].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[6].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[7].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[8].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[9].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[10].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[11].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[12].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[13].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[14].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[15].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[16].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[17].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[18].cellVoltage,
                modSDcardPackStateHandle->cellVoltagesIndividual[19].cellVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[2].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[3].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[4].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[5].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[6].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[7].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[8].auxVoltage,
                modSDcardPackStateHandle->auxVoltagesIndividual[9].auxVoltage);
        fresult = f_write(&bms_log_file,stringBuffer, bufsize(stringBuffer), &bw);
        if(fresult != FR_OK)
        {
            memset(stringBuffer, '\0' ,BUFFER_SIZE);
            return;
        }
        fresult = f_close(&bms_log_file);
        //clear_buffer();
        memset(stringBuffer, '\0', BUFFER_SIZE);
        return;
        
        #endif
    }
};



