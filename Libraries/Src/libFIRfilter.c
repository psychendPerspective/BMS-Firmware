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


#include "libFIRfilter.h"

//10 point filter, 1/10 is the filter coefficients
//static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f}; 
//16 point filter, 1/16 is the filter coefficients
static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH]= {0.0625f,0.0625f,0.0625f,0.0625f,0.0625f,0.0625f,0.0625f,
                                                        0.0625f,0.0625f,0.0625f,0.0625f,0.0625f,0.0625f,0.0625f,
                                                        0.0625f,0.0625f};


void FIRFilter_Init(FIRFilter *fir) {

	/* Clear filter buffer */
	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		fir->buf[n] = 0.0f;

	}

	/* Reset buffer index */
	fir->bufIndex = 0;

	/* Clear filter output */
	fir->out = 0.0f;

}

float FIRFilter_Update(FIRFilter *fir, float inp) {

	/* Store latest sample in buffer */
	fir->buf[fir->bufIndex] = inp;

	/* Increment buffer index and wrap around if necessary */
	fir->bufIndex++;

	if (fir->bufIndex == FIR_FILTER_LENGTH) {

		fir->bufIndex = 0;

	}

	/* Compute new output sample (via convolution) */
	fir->out = 0.0f;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {

		/* Decrement index and wrap if necessary */
		if (sumIndex > 0) {

			sumIndex--;

		} else {

			sumIndex = FIR_FILTER_LENGTH - 1;

		}

		/* Multiply impulse response with shifted input sample and add to output */
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	/* Return filtered output */
	return fir->out;

}