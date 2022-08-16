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

uint8_t modSDcard_Init(void)
{
    sdCardStatus = false;
    //init SPI2 driver and parameter config
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
    /* Open file to write/ create a file if it doesn't exist */
    fresult = f_open(&bms_log_file, "XanaduBMS_Log_File.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    /* Move the offset to the end of the file */
    fresult = f_lseek(&bms_log_file, f_size(&bms_log_file));
    /* Write logging file header */
    f_puts("Timestamp(s),Pack_Voltage(V), Pack_Current(A),Pack_Power(W),SoC(%%),Remaining Capacity(Ah),Operational_State,Fault_State,Cycle_Count,CV_Max(V),CV_Min(V),CV_Max_Imbalance(V),Battery_Temp_Max(degC),Battery_Temp_Min(degC),Battery_Temp_Avg(degC),CV_1(V),CV_2(V),CV_3(V),CV_4(V),CV_5(V),CV_6(V),CV_7(V),CV_8(V),CV_9(V),CV_10(V),CV_11(V),CV_12(V),CV_13(V),CV_14(V),CV_15(V),CV_16(V),CV_17(V),CV_18(V),Temp_1(degC),Temp_2(degC),Temp_3(degC),Temp_4(degC),Temp_5(degC),Temp_6(degC),Temp_7(degC)\n", &bms_log_file);
    /* Close the log file */
    fresult = f_close(&bms_log_file);
    if(fresult == FR_OK)
    {
        sdCardStatus = true;
    }

    return sdCardStatus;
};
