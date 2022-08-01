/*
	Copyright 2017 - 2018 Danny Bokma	danny@diebie.nl
	Copyright 2019 - 2020 Kevin Dionne	kevin.dionne@ennoid.me

	This file is part of the DieBieMS/ENNOID-BMS firmware.

	The DieBieMS/ENNOID-BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The DieBieMS/ENNOID-BMS firmware is distributed in the hope that it will be useful,
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
	modStateOfChargeGeneralStateOfCharge.generalStateOfCharge = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh / modStateOfChargeGeneralConfigHandle->batteryCapacity * 100.0f;
	
	if(modStateOfChargeGeneralStateOfCharge.generalStateOfCharge >= 100.0f)
		modStateOfChargeGeneralStateOfCharge.generalStateOfCharge = 100.0f;
	
	modStateOfChargePackStatehandle->SoC = modStateOfChargeGeneralStateOfCharge.generalStateOfCharge;
	modStateOfChargePackStatehandle->SoCCapacityAh = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh;
	
	// Store SoC every 'stateOfChargeStoreInterval'
	if(modDelayTick1ms(&modStateOfChargeStoreSoCTick,modStateOfChargeGeneralConfigHandle->stateOfChargeStoreInterval) && !modStateOfChargePowerDownSavedFlag && (lastGeneralStateOfCharge.remainingCapacityAh != modStateOfChargeGeneralStateOfCharge.remainingCapacityAh))
		modStateOfChargeStoreStateOfCharge();
};

bool modStateOfChargeStoreAndLoadDefaultStateOfCharge(void){
	bool returnVal = false;
	if(driverSWStorageManagerStateOfChargeEmpty)
	{
		// TODO: SoC manager is empy -> Determin SoC from voltage when voltages are available.
		modStateOfChargeStructTypeDef defaultStateOfCharge;
		defaultStateOfCharge.generalStateOfCharge = 0.0f;
		defaultStateOfCharge.generalStateOfHealth = 0.0f;
		defaultStateOfCharge.remainingCapacityAh = 0.0f;
		defaultStateOfCharge.remainingCapacityWh = 0.0f;
		
		driverSWStorageManagerStateOfChargeEmpty = false;
		driverSWStorageManagerStoreStruct(&defaultStateOfCharge,STORAGE_STATEOFCHARGE);
		// TODO_EEPROM
	}
	
	modStateOfChargeStructTypeDef tempStateOfCharge;
	driverSWStorageManagerGetStruct(&tempStateOfCharge,STORAGE_STATEOFCHARGE);
	
	modStateOfChargeLoadStateOfCharge();
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

void modGetStateofChargeFromOCV(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer)
{
	float OCV_vs_SOC[NUM_OCV_VS_SOC_POINTS] = {0}; //100%,95%,90%,85%,80%,......,5%,0% SoC

	modStateOfChargePackStatehandle = packState;
	modStateOfChargeGeneralConfigHandle = generalConfigPointer;
	
	switch(modStateOfChargeGeneralConfigHandle->cellTypeUsed)
	{
		case notValid:
			break;
		
		case AMS_18650_2500mAh:
			static const float OCV_vs_SOC_AMS_18650_2500mAh[NUM_OCV_VS_SOC_POINTS] = 
												{4.190f, 4.090f, 4.050f, 3.980f, 3.910f, 3.860f, 3.790f, 3.720f,
												3.650f, 3.580f, 3.510f, 3.440f, 3.370f, 3.300f, 3.230f, 3.160f,
												3.090f, 3.020f, 2.980f, 2.910f, 2.800f};
			memcpy(OCV_vs_SOC, OCV_vs_SOC_AMS_18650_2500mAh, sizeof(OCV_vs_SOC_AMS_18650_2500mAh));
			break;
		
		case MOLICEL_21700_P42A:
			static const float OCV_vs_SOC_MOLICEL_21700_P42A[NUM_OCV_VS_SOC_POINTS] = 
												{4.190f, 4.090f, 4.050f, 3.980f, 3.910f, 3.860f, 3.880f, 3.820f,
												3.810f, 3.800f, 3.780f, 3.740f, 3.720f, 3.710f, 3.690f, 3.660f,
												3.650f, 3.020f, 2.980f, 2.910f, 2.800f};
			memcpy(OCV_vs_SOC, OCV_vs_SOC_MOLICEL_21700_P42A, sizeof(OCV_vs_SOC_MOLICEL_21700_P42A));
			break;
		
		case MOLICEL_18650_P28A:
			//TO DO : Add lookup tables for different cells
			break;
		
		case PANASONIC_18650_GA:
			//TO DO : Add lookup tables for different cells
			break;

	}

	modPowerElectronicsCellMonitorsCheckConfigAndReadAnalogData();
	modPowerElectronicsCalculateCellStats();
	modPowerElectronicsCellMonitorsStartCellConversion();

	for(int i = 0 ; i < NUM_OCV_VS_SOC_POINTS; i++)
	{
		if(OCV_vs_SOC[i] <= modStateOfChargePackStatehandle->cellVoltageAverage )
		{
			if(i == 0)
			{
				modStateOfChargeGeneralStateOfCharge.generalStateOfCharge = 100.0f;
			}
			else
			{
				//Calculate Remaining capacity based on current OCV(CVaverage) by comparing lookup table values 
				modStateOfChargeGeneralStateOfCharge.remainingCapacityAh = 
				modStateOfChargeGeneralConfigHandle->batteryCapacity / (NUM_OCV_VS_SOC_POINTS - 1.0) *
				(NUM_OCV_VS_SOC_POINTS - 1.0 - i + (modStateOfChargePackStatehandle->cellVoltageAverage - OCV_vs_SOC[i])/(OCV_vs_SOC[i-1] - OCV_vs_SOC[i]));

				//calculate SoC based on nominal battery capacity 
				modStateOfChargeGeneralStateOfCharge.generalStateOfCharge = modStateOfChargeGeneralStateOfCharge.remainingCapacityAh / modStateOfChargeGeneralConfigHandle->batteryCapacity * 100.0f;

			}
		}
	}

};
