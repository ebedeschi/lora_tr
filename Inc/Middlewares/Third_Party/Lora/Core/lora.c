 /*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Wael Guibene
*/
/******************************************************************************
  * @file    lora.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    15-September-2016
  * @brief   lora API to drive the lora state Machine
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "timeServer.h"
#include "LoRaMac.h"
#include "lora.h"
#include "temp/temp.h"
#include "libsinc/libsinc.h"
#include "../Src/SHT2x/SHT2x.h"

/*!
 * Join requests trials duty cycle.
 */
#define OVER_THE_AIR_ACTIVATION_DUTYCYCLE           10000  // 10 [s] value in ms

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

float appo[] = { 0.0, 0.125333234, 0.248689887, 0.368124553, 0.481753674, 0.587785252, 0.684547106, 0.770513243, 0.844327926, 0.904827052, 0.951056516, 0.982287251, 0.998026728, 0.998026728, 0.982287251, 0.951056516, 0.904827052, 0.844327926, 0.770513243, 0.684547106, 0.587785252, 0.481753674, 0.368124553, 0.248689887, 0.125333234, 2.32152E-14, -0.125333234, -0.248689887, -0.368124553, -0.481753674, -0.587785252, -0.684547106, -0.770513243, -0.844327926, -0.904827052, -0.951056516, -0.982287251, -0.998026728, -0.998026728, -0.982287251, -0.951056516, -0.904827052, -0.844327926, -0.770513243, -0.684547106, -0.587785252, -0.481753674, -0.368124553, -0.248689887, -0.125333234 };
uint8_t c_appo = 0;

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                           241//64

/*!
 * User application data
 */
static uint8_t AppDataBuff[LORAWAN_APP_DATA_BUFF_SIZE];

/*!
 * User application data structure
 */
static lora_AppData_t AppData={ AppDataBuff,  0 ,0 };

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static FunctionalState IsTxConfirmed ;

/*!
 * Defines the LoRa parameters at Init
 */
static  LoRaParam_t* LoRaParam;
/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

static DeviceState_t DeviceState = DEVICE_STATE_INIT ;

/*!
 * Timer to handle the state of LED1
 */

static LoRaMacPrimitives_t LoRaMacPrimitives;
static LoRaMacCallback_t LoRaMacCallbacks;
static MibRequestConfirm_t mibReq;

static LoRaMainCallback_t *LoRaMainCallbacks;


/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    FunctionalState IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( )
{
    if( ComplianceTest.Running == true )
    {
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppData.BuffSize = 3;
            AppData.Buff[0] = 5;
            AppData.Buff[1] = ComplianceTest.DemodMargin;
            AppData.Buff[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppData.BuffSize = 2;
                AppData.Buff[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData.Buff[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
    }
    else
    {
        LoRaMainCallbacks->LoraTxData(&AppData, &IsTxConfirmed);
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppData.BuffSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LoRaParam->TxDatarate;
    }
    else
    {
        if( IsTxConfirmed == DISABLE )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppData.Port;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData.Buff;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppData.BuffSize;
            mcpsReq.Req.Unconfirmed.Datarate = LoRaParam->TxDatarate;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppData.Port;
            mcpsReq.Req.Confirmed.fBuffer = AppData.Buff;
            mcpsReq.Req.Confirmed.fBufferSize = AppData.BuffSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LoRaParam->TxDatarate;
        }
    }
    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

void OnSendEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}
/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
	PRINTF("OnTxNextPacketTimerEvent\n");
    TimerStop( &TxNextPacketTimer );
    OnSendEvent();
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] McpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = DISABLE;
                    AppData.Port = 224;
                    AppData.BuffSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LoRaParam->AdrEnable;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppData.BuffSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = ENABLE;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = DISABLE;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppData.BuffSize = mcpsIndication->BufferSize;

                    AppData.Buff[0] = 4;
                    for( uint8_t i = 1; i < AppData.BuffSize; i++ )
                    {
                        AppData.Buff[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                default:                  
                  break;
                }
            }
            break;
        default:
            
            AppData.Port = mcpsIndication->Port;
            AppData.BuffSize = mcpsIndication->BufferSize;
            memcpy1( AppData.Buff, mcpsIndication->Buffer, AppData.BuffSize );
            
            LoRaMainCallbacks->LoraRxData( &AppData );
            break;
        }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] MlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mlmeConfirm->MlmeRequest )
        {
            case MLME_JOIN:
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_JOINED;
                NextTx = true;
                break;
            }
            case MLME_LINK_CHECK:
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;
}
/**
 *  lora Init
 */
void lora_Init (LoRaMainCallback_t *callbacks, LoRaParam_t* LoRaParamInit )
{
  /* init the DeviceState*/
  DeviceState= DEVICE_STATE_INIT;
  
  /* init the Tx Duty Cycle*/
  LoRaParam = LoRaParamInit;
  
  /* init the main call backs*/
  LoRaMainCallbacks = callbacks;
  
#if (STATIC_DEVICE_EUI != 1)
  LoRaMainCallbacks->BoardGetUniqueId( DevEui );  
#endif
  
#if( OVER_THE_AIR_ACTIVATION != 0 )

  PRINTF("OTAA\n"); 
  PRINTF("DevEui= %02X", DevEui[0]) ;for(int i=1; i<8 ; i++) {PRINTF("-%02X", DevEui[i]); }; PRINTF("\n");
  PRINTF("AppEui= %02X", AppEui[0]) ;for(int i=1; i<8 ; i++) {PRINTF("-%02X", AppEui[i]); }; PRINTF("\n");
  PRINTF("AppKey= %02X", AppKey[0]) ;for(int i=1; i<16; i++) {PRINTF(" %02X", AppKey[i]); }; PRINTF("\n\n");
#else

#if (STATIC_DEVICE_ADDRESS != 1)
  // Random seed initialization
  srand1( LoRaMainCallbacks->BoardGetRandomSeed( ) );
  // Choose a random device address
  DevAddr = randr( 0, 0x01FFFFFF );
#endif
  PRINTF("ABP\n"); 
  PRINTF("DevEui= %02X", DevEui[0]) ;for(int i=1; i<8 ; i++) {PRINTF("-%02X", DevEui[i]); }; PRINTF("\n");
  PRINTF("DevAdd=  %08X\n", DevAddr) ;
  PRINTF("NwkSKey= %02X", NwkSKey[0]) ;for(int i=1; i<16 ; i++) {PRINTF(" %02X", NwkSKey[i]); }; PRINTF("\n");
  PRINTF("AppSKey= %02X", AppSKey[0]) ;for(int i=1; i<16 ; i++) {PRINTF(" %02X", AppSKey[i]); }; PRINTF("\n");
#endif

}

/**
 *  lora class A state machine
 */

void lora_fsm( void)
{
  switch( DeviceState )
  {
    case DEVICE_STATE_INIT:
    {
        LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
        LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
        LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;

        LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

        TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
        
        mibReq.Type = MIB_ADR;
        mibReq.Param.AdrEnable = LoRaParam->AdrEnable;
        LoRaMacMibSetRequestConfirm( &mibReq );

        mibReq.Type = MIB_PUBLIC_NETWORK;
        mibReq.Param.EnablePublicNetwork = LoRaParam->EnablePublicNetwork;
        LoRaMacMibSetRequestConfirm( &mibReq );
                        
        mibReq.Type = MIB_DEVICE_CLASS;
        mibReq.Param.Class= LoRaParam->Class;
        LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 ) 
                LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
                LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
                LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
                LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
                LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
                LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
                LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );
#endif

#endif
      DeviceState = DEVICE_STATE_JOIN;
      break;
    }
    case DEVICE_STATE_JOIN:
    {
#if( OVER_THE_AIR_ACTIVATION != 0 )
      MlmeReq_t mlmeReq;
    
      mlmeReq.Type = MLME_JOIN;
      mlmeReq.Req.Join.DevEui = DevEui;
      mlmeReq.Req.Join.AppEui = AppEui;
      mlmeReq.Req.Join.AppKey = AppKey;

      if( NextTx == true )
      {
          LoRaMacMlmeRequest( &mlmeReq );
      }

      DeviceState = DEVICE_STATE_SLEEP;
#else
      mibReq.Type = MIB_NET_ID;
      mibReq.Param.NetID = LORAWAN_NETWORK_ID;
      LoRaMacMibSetRequestConfirm( &mibReq );

      mibReq.Type = MIB_DEV_ADDR;
      mibReq.Param.DevAddr = DevAddr;
      LoRaMacMibSetRequestConfirm( &mibReq );

      mibReq.Type = MIB_NWK_SKEY;
      mibReq.Param.NwkSKey = NwkSKey;
      LoRaMacMibSetRequestConfirm( &mibReq );

      mibReq.Type = MIB_APP_SKEY;
      mibReq.Param.AppSKey = AppSKey;
      LoRaMacMibSetRequestConfirm( &mibReq );

      mibReq.Type = MIB_NETWORK_JOINED;
      mibReq.Param.IsNetworkJoined = true;
      LoRaMacMibSetRequestConfirm( &mibReq );

      DeviceState = DEVICE_STATE_SEND;
#endif
      break;
    }
    case DEVICE_STATE_JOINED:
    {
      PRINTF("JOINED\n");
      DeviceState = DEVICE_STATE_SEND;
      setPacketSinc(true);
      break;
    }
    case DEVICE_STATE_SEND:
    {
    	struct SincStatus status = getStatus();
    	if(status._packet_sinc == 1) // controlla se si � in fase di sicronizzazione o invio
    	{
    		  PRINTF("_packet_sinc 1\r\n");
			  PrepareTxFrame( );
			  NextTx = SendFrame( );

			  if(status._send2==false)
			  {
				  // Schedule next packet transmission as soon as possible
				  TimerSetValue( &TxNextPacketTimer,  3000); /* 3s */
				  TimerStart( &TxNextPacketTimer );
			  }
			  else
			  {
				  if( ComplianceTest.Running == true )
				  {
					  // Schedule next packet transmission as soon as possible
					  TimerSetValue( &TxNextPacketTimer,  5000); /* 5s */
					  TimerStart( &TxNextPacketTimer );
				  }
				  else if (LoRaParam->TxEvent == TX_ON_TIMER )
				  {
					  // Schedule next packet transmission
					  TimerSetValue( &TxNextPacketTimer, LoRaParam->TxDutyCycleTime );
					  TimerStart( &TxNextPacketTimer );
				  }
		//      }
			  }
    	}
    	else
    	{

    		if(getAcquire())
    		{
//    			uint16_t sT;
//    			float   temperatureC;           //variable for temperature[�C] as float
//    			uint8_t  error = 0;              //variable for error code. For codes see system.h
//    			error |= SHT2x_MeasureHM(TEMP, &sT);
//    			temperatureC = SHT2x_CalcTemperatureC(sT);
//    			if(error==0)
//    				insert(temperatureC);
//    			else
//    				insert((float)-100);

    			insert(appo[c_appo]);
    			c_appo = (c_appo+1) % 50;
    		}

			if(checkExtract()==1)
			{
				PRINTF("check 1\r\n");
				if( NextTx == true )
				{
				  PrepareTxFrame( );
				  NextTx = SendFrame( );
				}
			}
			else
				PRINTF("check 0\r\n");

			TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
			TimerSetValue( &TxNextPacketTimer,  10000); /* 10s */
			TimerStart( &TxNextPacketTimer );

//			  if( ComplianceTest.Running == true )
//			  {
//				  // Schedule next packet transmission as soon as possible
//				  TimerSetValue( &TxNextPacketTimer,  5000); /* 5s */
//				  TimerStart( &TxNextPacketTimer );
//			  }
//			  else if (LoRaParam->TxEvent == TX_ON_TIMER )
//			  {
//				  // Schedule next packet transmission
//				  TimerSetValue( &TxNextPacketTimer, LoRaParam->TxDutyCycleTime );
//				  TimerStart( &TxNextPacketTimer );
//			  }
    	}

      DeviceState = DEVICE_STATE_SLEEP;
      break;
    }
    case DEVICE_STATE_SLEEP:
    {
        // Wake up through events
      break;
    }
    default:
    {
      DeviceState = DEVICE_STATE_INIT;
      break;
    }
  }
}


DeviceState_t lora_getDeviceState( void )
{
  return DeviceState;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

