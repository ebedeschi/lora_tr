/*
 * temp.c
 *
 *  Created on: 27 lug 2017
 *      Author: Emanuele
 */

#include "temp.h"
#include "vcom.h"
#include "libtime/libtime.h"

struct TempStruct q[DIM_Q]; // queue
uint8_t iq = 0; // input queue
uint8_t oq = 0; // output queue

uint8_t en = 1;

bool acquire = true;

void initTempStruct(struct TempStruct* ele)
{
	uint8_t c;
	ele->complete=0;
	ele->ct=0;
	for(c=0;c<DIM_TEMP;c++)
	{
		ele->temp[c]=(float)-100;
	}
	initTimeStampStruct(&(ele->t));
}

void insert(float temp)
{
	if(en == 1)
	{
		if( q[iq].ct == 0 ) // take timestamp of first of the array
			q[iq].t = getRTCTime();
		q[iq].temp[q[iq].ct++] = temp;
		if(q[iq].ct>=DIM_TEMP)
		{
			q[iq].complete=1;
			if( ((iq+1)%DIM_Q) != oq )
			{
				initTempStruct(&q[(iq+1)%DIM_Q]);
				iq = (iq+1)%DIM_Q;
			}
			else
				en=0;
		}
	}

}

struct TempStruct extract()
{
	struct TempStruct ele;

	if( (iq != oq) && q[oq].complete==1)
	{
		ele = q[oq];
		oq = (oq+1)%DIM_Q;
		if(en == 0)
		{
			initTempStruct(&q[(iq+1)%DIM_Q]);
			iq = (iq+1)%DIM_Q;
			en=1;
		}
		return ele;
	}
	// else i can't extract element
	initTempStruct(&ele);
	return ele;
}

uint8_t checkExtract()
{
	if( (iq != oq) && q[oq].complete==1)
	{
		return 1;
	}
	else
		return 0;
}

bool getAcquire()
{
	return acquire;
}

void setAcquire(bool a)
{
	acquire = a;
}

void printStat()
{
	PRINTF("iq %d oq %d\r\n", iq, oq);
	PRINTF("q[iq].ct %d\r\n", q[iq].ct);
//	struct TimeStampStruct ts =  q[iq].t;
//	printTime(ts);
//	PRINTF("\r\n", iq, oq);
}
