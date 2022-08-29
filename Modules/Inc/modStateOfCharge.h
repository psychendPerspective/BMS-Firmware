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

#ifndef __MODSTATEOFCHARGE_H
#define __MODSTATEOFCHARGE_H

#include "stdint.h"
#include "stdbool.h"
#include "modPowerElectronics.h"


#define NUM_OCV_VS_SOC_POINTS 21


typedef struct {
	float stateofCharge;
	float generalStateOfHealth;
	float remainingCapacityAh;
	float remainingCapacityWh;
} modStateOfChargeStructTypeDef;

typedef enum {
	EVENT_FULL = 0,
	EVENT_EMPTY,
} modStateOfChargeVoltageEventTypeDef;


modStateOfChargeStructTypeDef* modStateOfChargeInit(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer);
void modStateOfChargeProcess(void);
bool modStateOfChargeStoreAndLoadDefaultStateOfCharge(void);
bool modStateOfChargeStoreStateOfCharge(void);
bool modStateOfChargeLoadStateOfCharge(void);
bool modStateOfChargePowerDownSave(void);
void modStateOfChargeVoltageEvent(modStateOfChargeVoltageEventTypeDef eventType);
float interpolate(const float a[], const float b[], size_t size, float value_a);
void modGetStateofChargeFromOCV();

#endif
