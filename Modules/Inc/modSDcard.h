/*
	Copyright 2017 - 2018 Danny Bokma	danny@diebie.nl
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

#ifndef __MODSDCARD_H
#define __MODSDCARD_H

#include "fatfs.h"
#include "driverHWSPI2.h"
#include "ff.h"
#include "fatfs_sd.h"
#include "modPowerElectronics.h"
#include "modConfig.h"

/*Relevant FATFS library files compatible with stm32f30x HAL library : https://github.com/STMicroelectronics/stm32_mw_fatfs/releases/tag/r0.11 */

#define MIN_SDCARD_STORAGE_REQ           1024           //MiB
#define LOGGING_INTERVAL                 2000           //ms

uint8_t modSDcard_Init(modPowerElectronicsPackStateTypedef *packState, modConfigGeneralConfigStructTypedef *generalConfigPointer);
int bufsize (char *buf);
void clear_buffer (void);
void modSDcard_logtoCSV(void);
#endif