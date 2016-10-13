/*
 * Copyright (C) wudong
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/sonar_for_ppz/sonar_for_ppz.c"
 * @author wudong
 * use the sonar data to hold the altitude
 */

#include "modules/sonar_for_ppz/sonar_for_ppz.h"
#include "subsystems/ins/vf_float.h"
#include "modules/optical_flow/px4flow.h"

#define sonar_min  0.3
#define sonar_max  3.5
#define sonar_half 1.5
#define sonar_max_cos 2.89;

#define SONAR_OUT_OF_RANGE -1

#define cos_8  = 0.9902
#define cos_10 = 0.9848
#define cos_15 = 0.9659
#define cos_20 = 0.9397


#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { int32_t temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }

float sonar_height; 
float sonar_offset;
bool offset_doing = true;

int32_t applySonarMedianFilter(int32_t data);
int32_t quickMedianFilter5(int32_t * v);


void sonarinit(void) 
{
	sonar_offset = 0.0;
	sonar_height = 0.0; 
}


void sonarperiodic(void) 
{
	int32_t data = 0;
	float scale = 0;
	float raw_data = optical_flow.ground_distance;

	if(offset_doing)
	{	
		if(vff.z < -0.2 || vff.z > 0.2){
		for(uint8_t i = 0;i<5;i++)	sonar_offset += (-vff.z - raw_data);
		sonar_offset /= 5;
		offset_doing = false;
		}
	}
	else
	{	raw_data += sonar_offset;
		data = (int32_t) (raw_data * 100);
		Bound(data, (int32_t)((sonar_min+sonar_offset)*100),(int32_t)((sonar_max+sonar_offset)*100));
		data = applySonarMedianFilter(data);
		scale = data/1500;//sonar_max*2;
		sonar_height = -data * 0.01*(1-scale) + vff.z*scale;

	}

/*
    if (distance > 0 && distance < sonar_half) 
	{
        // just use the SONAR
        baroAlt_offset = BaroAlt - distance;
        sonarAlt = distance;
    } 
	else 
	{
        BaroAlt -= baroAlt_offset;
        if (distance > 0  && distance <= sonar_max_cos) 
		{
	        // SONAR in range, so use complementary filter
	        sonarTransition = (float)(sonar_max_cos - distance) / (sonar_max_cos - sonar_half);
	        BaroAlt = distance * sonarTransition + BaroAlt * (1.0f - sonarTransition);
    	}
	}
*/
}


#if 1
int32_t applySonarMedianFilter(int32_t data)
{
    static int32_t sonarFilterSamples[5];
    static int i = 0;
    static bool medianFilterReady = false;
    int j;

    if (data > -1) // only accept samples that are in range
    {
        j = (i + 1);
        if (j == 5) 
		{
            j = 0;
            medianFilterReady = true;
        }
        sonarFilterSamples[i] = data;
        i = j;
    }
    if (medianFilterReady)
        return quickMedianFilter5(sonarFilterSamples);
    else
        return data;
}

int32_t quickMedianFilter5(int32_t * v)
{
    int32_t p[5];
    QMF_COPY(p, v, 5);

    QMF_SORT(p[0], p[1]); QMF_SORT(p[3], p[4]); QMF_SORT(p[0], p[3]);
    QMF_SORT(p[1], p[4]); QMF_SORT(p[1], p[2]); QMF_SORT(p[2], p[3]);
    QMF_SORT(p[1], p[2]); 
    return p[2];
}

#endif
