/*
	Copyright 2017 - 2018 Danny Bokma	  danny@diebie.nl
	Copyright 2019 - 2020 Kevin Dionne	kevin.dionne@ennoid.me
  	Copyright 2022        Vishal Bhat   vishal.bhat09@gmail.com

	This file is part of the Xanadu BMS firmware.

	The Xanadu BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The Xanadu BMS firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
 
#include "modStateOfCharge.h"
#include "modTerminal.h"

modStateOfChargeStructTypeDef modStateOfChargeGeneralStateOfCharge;
modPowerElectronicsPackStateTypedef *modStateOfChargePackStatehandle;
modConfigGeneralConfigStructTypedef *modStateOfChargeGeneralConfigHandle;
uint32_t modStateOfChargeLargeCoulombTick;
uint32_t modStateOfChargeStoreSoCTick;

bool modStateOfChargePowerDownSavedFlag = false;
bool readSoCfromOCV = false;
bool ocvInterpolatedFlag = false;

modStateOfChargeStructTypeDef* modStateOfChargeInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer){
	modStateOfChargePackStatehandle = packState;
	modStateOfChargeGeneralConfigHandle = generalConfigPointer;
	driverSWStorageManagerStateOfChargeStructSize = (sizeof(modStateOfChargeStructTypeDef)/sizeof(uint16_t)); // Calculate the space needed for the config struct in EEPROM
	
	modStateOfChargeLargeCoulombTick = HAL_GetTick();
	modStateOfChargeStoreSoCTick = HAL_GetTick();
	
	return &modStateOfChargeGeneralStateOfCharge;
};

void modStateOfChargeProcess(void){
	// Calculate accumulated energy
	uint32_t dt = HAL_GetTick() - modStateOfChargeLargeCoulombTick;
	modStateOfChargeStructTypeDef lastGeneralStateOfCharge;
	
	lastGeneralStateOfCharge = modStateOfChargeGeneralStateOfCharge;
	
	modStateOfChargeLargeCoulombTick = HAL_GetTick();
	modStateOfChargeGeneralStateOfCharge.remainingCapacityAh += dt*modStateOfChargePackStatehandle->packCurrent/(3600*1000);// (miliseconds * amps)/(3600*1000) accumulatedCharge in AmpHour.
	
	// Cap the max stored energy to the configured battery capacity.
	if(modStateOfChargeGeneralStateOfCharge.remainingCapacityAh > modStateOfChargeGeneralConfigHandle->batteryCapacity)
		modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = modStateOfChargeGeneralConfigHandle->batteryCapacity;
	
	if(modStateOfChargeGeneralStateOfCharge.remainingCapacityAh < 0.0f)
		modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = 0.0f;
	
	// Calculate state of charge
	modStateOfChargeGeneralStateOfCharge.stateofCharge = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh / modStateOfChargeGeneralConfigHandle->batteryCapacity * 100.0f;
	
	if(modStateOfChargeGeneralStateOfCharge.stateofCharge >= 100.0f)
		modStateOfChargeGeneralStateOfCharge.stateofCharge = 100.0f;
	
	modStateOfChargePackStatehandle->SoC = modStateOfChargeGeneralStateOfCharge.stateofCharge;
	modStateOfChargePackStatehandle->SoCCapacityAh = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh;

	// Store SoC every 'stateOfChargeStoreInterval' //TO DO:Debug issues
	if(modDelayTick1ms(&modStateOfChargeStoreSoCTick,modStateOfChargeGeneralConfigHandle->stateOfChargeStoreInterval) && !modStateOfChargePowerDownSavedFlag && (lastGeneralStateOfCharge.remainingCapacityAh != modStateOfChargeGeneralStateOfCharge.remainingCapacityAh))
		modStateOfChargeStoreStateOfCharge();
};

bool modStateOfChargeStoreAndLoadDefaultStateOfCharge(void){
	bool returnVal = false;
	readSoCfromOCV = true;
	if(driverSWStorageManagerStateOfChargeEmpty)
	{
		// TODO: Store type of cell used 
		modStateOfChargeStructTypeDef defaultStateOfCharge;
		defaultStateOfCharge.stateofCharge = 0.0f;
		defaultStateOfCharge.generalStateOfHealth = 100.0f;
		defaultStateOfCharge.remainingCapacityAh = 0.0f;
		defaultStateOfCharge.remainingCapacityWh = 0.0f;
		
		//driverSWStorageManagerStateOfChargeEmpty = false;

		readSoCfromOCV = true;         //readSoC from OCV when new cell is used or flash memory has changed

		//driverSWStorageManagerStoreStruct(&defaultStateOfCharge,STORAGE_STATEOFCHARGE);
		// TODO_EEPROM
	}
	
	modStateOfChargeStructTypeDef tempStateOfCharge;
	driverSWStorageManagerGetStruct(&tempStateOfCharge,STORAGE_STATEOFCHARGE);
	
	if(modStateOfChargeLoadStateOfCharge())
		//readSoCfromOCV = false;    //if flash memory has not changed, read SoC from stored SoC
		readSoCfromOCV = true;
		driverSWStorageManagerStateOfChargeEmpty = false;


	return returnVal;
};

bool modStateOfChargeStoreStateOfCharge(void){
	return driverSWStorageManagerStoreStruct(&modStateOfChargeGeneralStateOfCharge,STORAGE_STATEOFCHARGE);
};

bool modStateOfChargeLoadStateOfCharge(void){
	return driverSWStorageManagerGetStruct(&modStateOfChargeGeneralStateOfCharge,STORAGE_STATEOFCHARGE);
};

bool modStateOfChargePowerDownSave(void) {
	if(!modStateOfChargePowerDownSavedFlag) {
		modStateOfChargePowerDownSavedFlag = true;
		modStateOfChargeStoreStateOfCharge();
		// TODO_EEPROM
		return true;
	}else
		return false;
};

void modStateOfChargeVoltageEvent(modStateOfChargeVoltageEventTypeDef eventType) {
	switch(eventType) {
		case EVENT_EMPTY:
			break;
		case EVENT_FULL:
			modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = modStateOfChargeGeneralConfigHandle->batteryCapacity;
			break;
		default:
			break;
	}
};



float interpolate(const float a[], const float b[], size_t size, float value_a)
{
    if (a[0] < a[size - 1]) {
        for (unsigned int i = 0; i < size; i++) {
            if (value_a <= a[i]) {
                if (i == 0) {
                    return b[0]; // value_a smaller than first element
                }
                else {
                    return b[i - 1] + (b[i] - b[i - 1]) * (value_a - a[i - 1]) / (a[i] - a[i - 1]);
                }
            }
        }
        return b[size - 1]; // value_a larger than last element
    }
    else {
        for (unsigned int i = 0; i < size; i++) {
            if (value_a >= a[i]) {
                if (i == 0) {
                    return b[0]; // value_a smaller than first element
                }
                else {
                    return b[i - 1] + (b[i] - b[i - 1]) * (value_a - a[i - 1]) / (a[i] - a[i - 1]);
                }
            }
        }
        return b[size - 1]; // value_a larger than last element
    }
};

void modGetStateofChargeFromOCV()
{
	modPowerElectronicsCellMonitorsReadCellVoltageData();  //read cell voltage data
	modPowerElectronicsCalculateCellStats();
	modPowerElectronicsCellMonitorsStartCellConversion();        //start cell voltage and GPIO_1,2 ADC conversion
	modPowerElectronicsCellMonitorsStartTemperatureConversion(); //start conversion of temperature ADC values

	//TO DO: check if type of cell has been changed, in order to ignore stored SoC and read from OCV
	if(readSoCfromOCV)         
	{
		float OCV_vs_SOC[NUM_OCV_VS_SOC_POINTS] = {0}; //100%,95%,90%,85%,80%,......,5%,0% SoC
		static const float soc_pct[NUM_OCV_VS_SOC_POINTS] = { 100.0F, 95.0F, 90.0F, 85.0F, 80.0F, 85.0F, 70.0F,
                                 							65.0F,  60.0F, 55.0F, 50.0F, 45.0F, 40.0F, 35.0F,
                                 							30.0F,  25.0F, 20.0F, 15.0F, 10.0F, 5.0F,  0.0F };

		
		switch(modStateOfChargeGeneralConfigHandle->cellTypeUsed)
		{ //Extract lookup table based on discharge curves: https://automeris.io/WebPlotDigitizer/
			case unavailable:
				//TO DO:implement simple interpolation based on UV and OV limit
				break;
			
			case AMS_18650_2500mAh:
				static const float OCV_vs_SOC_AMS_18650_2500mAh[NUM_OCV_VS_SOC_POINTS] = 
													{4.188f, 4.063f, 4.013f, 3.960f, 3.902f, 3.857f, 3.816f, 3.776f, 
													3.735f, 3.689f, 3.649f, 3.610f, 3.573f, 3.542f, 3.506f, 3.465f,
													3.427f, 3.367f, 3.262f, 3.053f, 2.545f};
				memcpy(OCV_vs_SOC, OCV_vs_SOC_AMS_18650_2500mAh, sizeof(OCV_vs_SOC_AMS_18650_2500mAh));
				break;
			
			case MOLICEL_21700_P42A: //4200mAh
				static const float OCV_vs_SOC_MOLICEL_21700_P42A[NUM_OCV_VS_SOC_POINTS] = 
													{4.170f, 4.058f, 4.032f, 4.008f, 3.955f, 3.895f,3.847f, 3.803f, 
													3.761f, 3.717f, 3.668f, 3.631f, 3.588f, 3.551f, 3.507f, 3.467f, 
													3.429f, 3.378f, 3.276f, 2.970f, 2.504f};
				memcpy(OCV_vs_SOC, OCV_vs_SOC_MOLICEL_21700_P42A, sizeof(OCV_vs_SOC_MOLICEL_21700_P42A));
				break;
			
			case MOLICEL_18650_P28A:  //2800mAh
				static const float OCV_vs_SOC_MOLICEL_18650_P28A[NUM_OCV_VS_SOC_POINTS] = 
													{4.188f, 4.063f, 4.013f, 3.960f, 3.902f, 3.857f, 3.816f, 3.776f, 
													3.735f, 3.689f, 3.649f, 3.610f, 3.573f, 3.542f, 3.506f, 3.465f,
													3.427f, 3.367f, 3.262f, 3.053f, 2.545f};
				memcpy(OCV_vs_SOC,OCV_vs_SOC_MOLICEL_18650_P28A, sizeof(OCV_vs_SOC_MOLICEL_18650_P28A));
				break;
			
			case PANASONIC_18650_GA:   //3300mAh
				static const float OCV_vs_SOC_PANASONIC_18650_GA[NUM_OCV_VS_SOC_POINTS] = 
													{4.174f, 3.912f, 3.895f, 3.863f, 3.817f, 3.762f,3.714f,3.666f,
													3.619f, 3.564f, 3.516f, 3.465f, 3.419f, 3.381f, 3.334f, 3.292f,
													3.241f, 3.172f, 3.071f, 2.878f, 2.501f};
				memcpy(OCV_vs_SOC, OCV_vs_SOC_PANASONIC_18650_GA, sizeof(OCV_vs_SOC_PANASONIC_18650_GA));
				break;

			case PANSONIC_18650_BD:    //3180mAh
				static const float OCV_vs_SOC_PANSONIC_18650_BD[NUM_OCV_VS_SOC_POINTS] = 
													{4.196f, 4.070f, 4.011f, 3.946f, 3.893f, 3.847f, 3.803f, 3.755f,
													3.705f, 3.663f, 3.620f, 3.589f, 3.554f, 3.525f, 3.492f, 3.455f, 
													3.408f, 3.346f, 3.274f, 3.112f, 2.500f};
				memcpy(OCV_vs_SOC, OCV_vs_SOC_PANSONIC_18650_BD, sizeof(OCV_vs_SOC_PANSONIC_18650_BD));
				break;

			//TO DO : Add bigger database for different cell types and cell chemistry lookup tables 
		}
		

		for(int i = 0 ; i < NUM_OCV_VS_SOC_POINTS; i++)
		{
			if((OCV_vs_SOC[i] <= modStateOfChargePackStatehandle->cellVoltageAverage) && (!ocvInterpolatedFlag) )
			{
				if(i == 0)
				{
					modStateOfChargeGeneralStateOfCharge.stateofCharge = 100.0f;
					ocvInterpolatedFlag = true;
				}
				else
				{
					//Calculate Remaining capacity based on current OCV(CVaverage) by comparing lookup table values 
					modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = (double)
					((modStateOfChargeGeneralConfigHandle->batteryCapacity) / (NUM_OCV_VS_SOC_POINTS - 1.0)) *
					(NUM_OCV_VS_SOC_POINTS - 1.0 - i + ((modStateOfChargePackStatehandle->cellVoltageAverage - OCV_vs_SOC[i])/(OCV_vs_SOC[i-1] - OCV_vs_SOC[i])));

					//calculate SoC based on nominal battery capacity 
					modStateOfChargeGeneralStateOfCharge.stateofCharge = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh / modStateOfChargeGeneralConfigHandle->batteryCapacity * 100.0f;
					ocvInterpolatedFlag = true;

				}
			}
		}

		// modStateOfChargeGeneralStateOfCharge.stateofCharge = interpolate(OCV_vs_SOC, soc_pct, NUM_OCV_VS_SOC_POINTS, modStateOfChargePackStatehandle->cellVoltageAverage);
		// modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = (modStateOfChargeGeneralStateOfCharge.stateofCharge * modStateOfChargeGeneralConfigHandle->batteryCapacity) / 100.0f;

		modStateOfChargePackStatehandle->SoC = modStateOfChargeGeneralStateOfCharge.stateofCharge;
		modStateOfChargePackStatehandle->SoCCapacityAh = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh;
		modStateOfChargePackStatehandle->SoH = modStateOfChargeGeneralStateOfCharge.generalStateOfHealth = 100.0f;
		driverSWStorageManagerStoreStruct(&modStateOfChargeGeneralStateOfCharge,STORAGE_STATEOFCHARGE);
		driverSWStorageManagerStateOfChargeEmpty = false;
	}

};
