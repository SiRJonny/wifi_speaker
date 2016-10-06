//*****************************************************************************
// network.c
//
// Network Interface
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include <stdio.h>


// Simplelink includes
#include "simplelink.h"

//driverlib includes
#include "hw_types.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "gpio_if.h"
#include "uart_if.h"
//common interface includes
#include "common.h"

//App Includes
#include "network.h"
#include "audioCodec.h"
#include "circ_buff.h"

// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************

#define ROLE_INVALID                    (-5)
#define AP_SSID_LEN_MAX                 (33)
#define SH_GPIO_9                       (9)            /* Red */
#define SH_GPIO_11                      (11)           /* Green */
#define SH_GPIO_25                      (25)           /* Yellow */
#define AUTO_CONNECTION_TIMEOUT_COUNT   (50)           /* 5 Sec */

#define CC3200_MDNS_NAME  "CC3200._audio._udp.local"

//#define PREFIX_BUFFER           "/441khz.wav"		//"/bye.wav"		//"/meee.wav" 		//"/graphics/folders/partimages/CC3200.jpg"
//#define HOST_NAME               "csmcs.uw.hu"		//"www.ti.com"
#define HOST_PORT               (80)

#define SIZE_40K                40960  /* Serial flash file size 40 KB */

//#define READ_SIZE               1450
#define MAX_BUFF_SIZE           1460

// File on the serial flash to be replaced
#define FILE_NAME               "cc3200.jpg"


#define IP_ADDR             0xc0a80064 /* 192.168.0.100 */
#define PORT_NUM            80
#define BUF_SIZE            1400
#define TCP_PACKET_COUNT    10

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

	DEVICE_START_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
	INVALID_HEX_STRING = DEVICE_START_FAILED - 1,
	TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
	TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
	FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
	INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
	FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
	FILE_OPEN_FAILED = FORMAT_NOT_SUPPORTED - 1,
	FILE_WRITE_ERROR = FILE_OPEN_FAILED - 1,
	INVALID_FILE = FILE_WRITE_ERROR - 1,
	SERVER_CONNECTION_FAILED = INVALID_FILE - 1,
	GET_HOST_IP_FAILED = SERVER_CONNECTION_FAILED  - 1,
	SOCKET_CREATE_ERROR = GET_HOST_IP_FAILED - 1,
	BIND_ERROR = SOCKET_CREATE_ERROR - 1,
	LISTEN_ERROR = BIND_ERROR -1,
	SOCKET_OPT_ERROR = LISTEN_ERROR -1,
	CONNECT_ERROR = SOCKET_OPT_ERROR -1,
	ACCEPT_ERROR = CONNECT_ERROR - 1,
	SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern tUDPSocket g_UdpSock;
extern OsiTaskHandle g_NetworkTask;
extern unsigned char g_loopback;
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_uiIpAddress = 0; //Device IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;

unsigned long  g_ulDestinationIP; // IP address of destination server
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char g_buff[MAX_BUFF_SIZE+1];
long bytesReceived = 0; // variable to store the file size

extern long          fileHandle;
extern unsigned long Token;

extern tCircularBuffer *pPlayBuffer;
extern tCircularBuffer *pRecordBuffer;

int IP_last = 999;

unsigned char POST_token[] = "__SL_P_UIP";
unsigned char GET_token_IP[]  = "__SL_G_UIP";

#define LED_STRING              "LED"
#define LED1_STRING             "LED1_"
#define LED2_STRING             ",LED2_"
#define LED_ON_STRING           "ON"
#define LED_OFF_STRING          "OFF"

char UDP_notify1[] = "NOTIFY * HTTP/1.1\nHOST: 239.255.255.250:1900\nCACHE-CONTROL: max-age=80\nLOCATION: http://192.168.1.102:80/upnp/description.xml\nNT: upnp:rootdevice\nNTS: ssdp:alive\nSERVER: FreeRTOS/1.0, UPnP/1.0, CC3200/0.1\nUSN: uuid:c03107e0-08f4-11e6-a837-0800200c9a66::upnp:rootdevice\n\n";
char UDP_notify2[] = "NOTIFY * HTTP/1.1\nHOST: 239.255.255.250:1900\nCACHE-CONTROL: max-age=80\nLOCATION: http://192.168.1.102:80/upnp/description.xml\nNT: urn:schemas-upnp-org:device:MediaRenderer:1\nNTS: ssdp:alive\nSERVER: FreeRTOS/1.0, UPnP/1.0, CC3200/0.1\nUSN: uuid:c03107e0-08f4-11e6-a837-0800200c9a66::urn:schemas-upnp-org:device:MediaRenderer:1\n\n";
char UDP_notify3[] = "NOTIFY * HTTP/1.1\nHOST: 239.255.255.250:1900\nCACHE-CONTROL: max-age=80\nLOCATION: http://192.168.1.102:80/upnp/description.xml\nNT: uuid:c03107e0-08f4-11e6-a837-0800200c9a66\nNTS: ssdp:alive\nSERVER: FreeRTOS/1.0, UPnP/1.0, CC3200/0.1\nUSN: uuid:c03107e0-08f4-11e6-a837-0800200c9a66\n\n";
char UDP_notify4[] = "NOTIFY * HTTP/1.1\nHOST: 239.255.255.250:1900\nCACHE-CONTROL: max-age=80\nLOCATION: http://192.168.1.102:80/upnp/description.xml\nNT: urn:schemas-upnp-org:service:AVTransport:1\nNTS: ssdp:alive\nSERVER: FreeRTOS/1.0, UPnP/1.0, CC3200/0.1\nUSN: uuid:c03107e0-08f4-11e6-a837-0800200c9a66::urn:schemas-upnp-org:service:AVTransport:1\n\n";
char UDP_notify5[] = "NOTIFY * HTTP/1.1\nHOST: 239.255.255.250:1900\nCACHE-CONTROL: max-age=80\nLOCATION: http://192.168.1.102:80/upnp/description.xml\nNT: urn:schemas-upnp-org:service:ConnectionManager:1\nNTS: ssdp:alive\nSERVER: FreeRTOS/1.0, UPnP/1.0, CC3200/0.1\nUSN: uuid:c03107e0-08f4-11e6-a837-0800200c9a66::urn:schemas-upnp-org:service:ConnectionManager:1\n\n";
char UDP_notify6[] = "NOTIFY * HTTP/1.1\nHOST: 239.255.255.250:1900\nCACHE-CONTROL: max-age=80\nLOCATION: http://192.168.1.102:80/upnp/description.xml\nNT: urn:schemas-upnp-org:service:RenderingControl:1\nNTS: ssdp:alive\nSERVER: FreeRTOS/1.0, UPnP/1.0, CC3200/0.1\nUSN: uuid:c03107e0-08f4-11e6-a837-0800200c9a66::urn:schemas-upnp-org:service:RenderingControl:1\n\n";

// tcp socket:
char g_cBsdBuf[BUF_SIZE];


char PREFIX_BUFFER[200] = "/441khz.wav";
char HOST_NAME[25] = "csmcs.uw.hu";

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//
//! mDNS_Task function
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void mDNS_Task()
{
    int lRetValmDNS;
    unsigned int pAddr;
    unsigned long usPort;
    unsigned short     ulTextLen = 200;
    char cText[201]; 

 
    //UnRegister mDNS Service if done Previously
    lRetValmDNS = sl_NetAppMDNSUnRegisterService((signed char *)CC3200_MDNS_NAME,
                              strlen(CC3200_MDNS_NAME));

    while(1)
    {  
        lRetValmDNS = 1;
        
        //Read mDNS service.
        while(lRetValmDNS)
        {
            ulTextLen = 200;
            lRetValmDNS = sl_NetAppDnsGetHostByService((signed char *) \
                                    CC3200_MDNS_NAME,
                                    strlen((const char *)CC3200_MDNS_NAME),
                                    SL_AF_INET,(unsigned long *)&pAddr,&usPort,
                                    &ulTextLen,(signed char *)&cText[0]);
        }
        if(lRetValmDNS == 0 && (pAddr!=INVALID_CLIENT_ADDRESS) && \
                                                 (pAddr!=g_uiIpAddress))
        {               
             //Speaker Detected - Add Client
             g_UdpSock.Client.sin_family = AF_INET;
             g_UdpSock.Client.sin_addr.s_addr = htonl(pAddr);
             g_UdpSock.Client.sin_port = htons(usPort);
             g_UdpSock.iClientLength = sizeof(g_UdpSock.Client);

             g_loopback = 0;

        }
         
             MAP_UtilsDelay(80*1000*100);
    }    
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on application's "
                           "request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_uiIpAddress = pEventData->ip;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));

            IP_last = SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0);
        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! This function gets triggered when HTTP Server receives Application
//! defined GET and POST HTTP Tokens.
//!
//! \param pHttpServerEvent Pointer indicating http server event
//! \param pHttpServerResponse Pointer indicating http server response
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent,
                               SlHttpServerResponse_t *pSlHttpServerResponse)
{
    unsigned char strLenVal = 0;

    if(!pSlHttpServerEvent || !pSlHttpServerResponse)
    {
        return;
    }

    switch (pSlHttpServerEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
          unsigned char *ptr;

          ptr = pSlHttpServerResponse->ResponseData.token_value.data;
          pSlHttpServerResponse->ResponseData.token_value.len = 0;
          if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data, GET_token_IP,
                    strlen((const char *)GET_token_IP)) == 0)
          {

            strLenVal = strlen("192.168.1.");
            pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            memcpy(ptr, "192.168.1.", strLenVal);
            ptr += strLenVal;
            ltoa(IP_last,ptr);
            strLenVal = 3;
            pSlHttpServerResponse->ResponseData.token_value.len += strLenVal;
            ptr += strLenVal;

            *ptr = '\0';
          }

        }
        break;

        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {
          unsigned char led;
          unsigned char *ptr = pSlHttpServerEvent->EventData.httpPostData.token_name.data;

          if(memcmp(ptr, POST_token, strlen((const char *)POST_token)) == 0)
          {
            ptr = pSlHttpServerEvent->EventData.httpPostData.token_value.data;
            strLenVal = strlen(LED_STRING);
            if(memcmp(ptr, LED_STRING, strLenVal) != 0)
              break;
            ptr += strLenVal;
            led = *ptr;
            strLenVal = strlen(LED_ON_STRING);
            ptr += strLenVal;
            if(led == '1')
            {
              if(memcmp(ptr, LED_ON_STRING, strLenVal) == 0)
              {
                      GPIO_IF_LedOn(MCU_RED_LED_GPIO);
              }
              else
              {
                      GPIO_IF_LedOff(MCU_RED_LED_GPIO);
              }
            }
            else if(led == '2')
            {
              if(memcmp(ptr, LED_ON_STRING, strLenVal) == 0)
              {
                      GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
              }
              else
              {
                      GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
              }
            }

          }
        }
          break;
        default:
          break;
    }
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        return;
    }

    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
        	switch( pSock->socketAsyncEvent.SockTxFailData.status )
            {
                case SL_ECLOSE:
                    /*UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->EventData.sd);*/
                    break;
                default:
                    /*UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->EventData.sd, pSock->EventData.status); */
                  break;
            }
            break;

        default:
           /*UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);*/
          break;
    }
}

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_uiIpAddress = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
}

#ifdef MULTICAST
//*****************************************************************************
//
//! Add to Multicast Group to receive Audio Stream 
//!
//! \param none
//!
//! \return  0 - Success
//!            -1 - Failure

//
//*****************************************************************************
long ReceiveMulticastPacket()
{
    long lRetVal = -1;
    SlSockIpMreq mreq;
    memset(&mreq,0,sizeof(SlSockIpMreq));

    // set interface
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    mreq.imr_multiaddr.s_addr = ADDR_MULTICAST_GROUP;

    // do membership call
    lRetVal = setsockopt(g_UdpSock.iSockDesc, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                             &mreq, sizeof(SlSockIpMreq)); 
    ASSERT_ON_ERROR(lRetVal);

    return SUCCESS;
}
#endif

//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//!
//! \return   SlWlanMode_t
//!                        
//
//****************************************************************************
static int ConfigureMode(int iMode)
{
    long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(iMode);
    ASSERT_ON_ERROR(lRetVal);

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}


//*****************************************************************************
//
//! Connect the Device to Network
//!
//! \param  None
//!
//! \return  0 - Success
//!            -1 - Failure
//!
//*****************************************************************************

long ConnectToNetwork()
{
    long lRetVal = -1;
    unsigned int uiConnectTimeoutCnt =0;
    
    SlSecParams_t secParams = {0};

	secParams.Key = (signed char *)SECURITY_KEY;
	secParams.KeyLen = strlen(SECURITY_KEY);
	secParams.Type = SECURITY_TYPE;

    //Start Simplelink Device 
    lRetVal =  sl_Start(NULL,NULL,NULL);
    ASSERT_ON_ERROR(lRetVal);

    if(lRetVal != ROLE_STA)
    {
        if (ROLE_AP == lRetVal)
        {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#endif
            }
        }
        //
        // Configure to STA Mode
        //
        lRetVal = ConfigureMode(ROLE_STA);
        if(lRetVal !=ROLE_STA)
        {
            UART_PRINT("Unable to set STA mode...\n\r");
            lRetVal = sl_Stop(SL_STOP_TIMEOUT);
            CLR_STATUS_BIT_ALL(g_ulStatus);
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }

 	lRetVal = sl_WlanConnect((signed char *)SSID_NAME,
						   strlen((const char *)SSID_NAME), 0, &secParams, 0);
	ASSERT_ON_ERROR(lRetVal);

	// Wait for WLAN Event


        while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
        {
        	//Turn Green LED On
			GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
			osi_Sleep(50);
			//Turn Green LED Off
			GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
			osi_Sleep(50);
        }

        //Turn Green LED Off      
        GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);    


    return SUCCESS;
    
}
//*****************************************************************************
//
//! Create Socket and Bind to Local IP
//!
//! \param  None
//!
//! \return  0 - Success
//!            -1 - Failure
//
//*****************************************************************************

long CreateUdpServer(tUDPSocket *pSock)
{
    int uiPort = AUDIO_PORT;
    long lRetVal = -1;
    
    pSock->iSockDesc = socket(AF_INET, SOCK_DGRAM, 0);
    pSock->Server.sin_family = AF_INET;
    pSock->Server.sin_addr.s_addr = htonl(INADDR_ANY);
    pSock->Server.sin_port = htons(uiPort);
    pSock->iServerLength = sizeof(pSock->Server);

    pSock->Client.sin_family = AF_INET;
    pSock->Client.sin_addr.s_addr = htonl(INVALID_CLIENT_ADDRESS);
    pSock->Client.sin_port = htons(uiPort);
    pSock->iClientLength = sizeof(pSock->Client);
    lRetVal = bind(pSock->iSockDesc,(struct sockaddr*)&(pSock->Server),
                 pSock->iServerLength);
    ASSERT_ON_ERROR(lRetVal);

    return SUCCESS;

}


//*****************************************************************************
//
//! This function flush received HTTP response
//!
//! \param[in] cli - Instance of the HTTP connection
//!
//! \return o on success else -ve
//!
//*****************************************************************************
static int FlushHTTPResponse(HTTPCli_Handle cli)
{
    const char *ids[2] = {
                            HTTPCli_FIELD_NAME_CONNECTION, /* App will get connection header value. all others will skip by lib */
                            NULL
  	  	  	  	  	     };
    char  buf[128];
    int id;
    int len = 1;
    bool moreFlag = 0;
    char ** prevRespFilelds = NULL;


    prevRespFilelds = HTTPCli_setResponseFields(cli, ids);

    // Read response headers
    while ((id = HTTPCli_getResponseField(cli, buf, sizeof(buf), &moreFlag))
            != HTTPCli_FIELD_ID_END)
    {
        if(id == 0)
        {
            if(!strncmp(buf, "close", sizeof("close")))
            {
                UART_PRINT("Connection terminated by server\n\r");
            }
        }
    }

    HTTPCli_setResponseFields(cli, (const char **)prevRespFilelds);

    while(1)
    {
        len = HTTPCli_readResponseBody(cli, buf, sizeof(buf) - 1, &moreFlag);
        ASSERT_ON_ERROR(len);

        if ((len - 2) >= 0 && buf[len - 2] == '\r' && buf [len - 1] == '\n')
        {
            break;
        }

        if(!moreFlag)
        {
            break;
        }
    }
    return SUCCESS;
}


//****************************************************************************
//
//! \brief Obtain the file from the server
//!
//!  This function requests the file from the server and save it on serial flash.
//!  To request a different file for different user needs to modify the
//!  PREFIX_BUFFER macros.
//!
//! \param[in] cli - Instance of the HTTP connection
//!
//! \return         0 for success and negative for error
//
//****************************************************************************
static int GetData(HTTPCli_Handle cli)
{
    long          lRetVal = 0;

    int content_length = 0;

    int id;
    int len=0;
    bool moreFlag = 0;
    HTTPCli_Field fields[3] = {
                                {HTTPCli_FIELD_NAME_HOST, HOST_NAME},
                                {HTTPCli_FIELD_NAME_ACCEPT, "text/html, application/xhtml+xml, */*"},
                                {NULL, NULL}
                              };

    const char *ids[4] = {
                            HTTPCli_FIELD_NAME_CONTENT_LENGTH,
                            HTTPCli_FIELD_NAME_TRANSFER_ENCODING,
                            HTTPCli_FIELD_NAME_CONNECTION,
                            NULL
                         };


    UART_PRINT("Start downloading the file\r\n");

    // Set request fields
    HTTPCli_setRequestFields(cli, fields);

    memset(g_buff, 0, sizeof(g_buff));

    // Make HTTP 1.1 GET request
    lRetVal = HTTPCli_sendRequest(cli, HTTPCli_METHOD_GET, PREFIX_BUFFER, 0);
    if (lRetVal < 0)
    {
        // error
        ASSERT_ON_ERROR(TCP_SEND_ERROR);
    }

    // Test getResponseStatus: handle
    lRetVal = HTTPCli_getResponseStatus(cli);
    if (lRetVal != 200)
    {
        FlushHTTPResponse(cli);
        if(lRetVal == 404)
        {
            ASSERT_ON_ERROR(FILE_NOT_FOUND_ERROR);
        }
        ASSERT_ON_ERROR(INVALID_SERVER_RESPONSE);
    }

    HTTPCli_setResponseFields(cli, ids);

    // Read response headers
    while ((id = HTTPCli_getResponseField(cli, (char *)g_buff, sizeof(g_buff), &moreFlag))
               != HTTPCli_FIELD_ID_END)
    {

        if(id == 0)
        {
            UART_PRINT("Content length: %s\n\r", g_buff);
            content_length = atoi((char*)g_buff);
        }
        else if(id == 1)
        {
            if(!strncmp((const char *)g_buff, "chunked", sizeof("chunked")))
            {
                UART_PRINT("Chunked transfer encoding\n\r");
            }
        }
        else if(id == 2)
        {
            if(!strncmp((const char *)g_buff, "close", sizeof("close")))
            {
                ASSERT_ON_ERROR(FORMAT_NOT_SUPPORTED);
            }
        }

    }

    // Open file to save the downloaded file
    lRetVal = sl_FsOpen((_u8 *)FILE_NAME, FS_MODE_OPEN_WRITE, &Token, &fileHandle);
    if(lRetVal < 0)
    {
        // File Doesn't exit create a new of 40 KB file
        lRetVal = sl_FsOpen((unsigned char *)FILE_NAME, \
                           FS_MODE_OPEN_CREATE(SIZE_40K, \
                           _FS_FILE_OPEN_FLAG_COMMIT|_FS_FILE_PUBLIC_WRITE),
                           &Token, &fileHandle);
        ASSERT_ON_ERROR(lRetVal);

    }



    while(1)
    {
    	int buffersize = 0;
    	int iRetVal = 0;
    	int iBufferFilled = 0;
        iBufferFilled = GetBufferSize(pPlayBuffer);
        if(iBufferFilled <= (66*PACKET_SIZE))
        {
        	len = HTTPCli_readResponseBody(cli, (char *)g_buff, sizeof(g_buff) - 1, &moreFlag);
			if(len < 0)
			{
				// Close file without saving
				lRetVal = sl_FsClose(fileHandle, 0, (unsigned char*) "A", 1);
				return lRetVal;
			}

			/*lRetVal = sl_FsWrite(fileHandle, bytesReceived,
									(unsigned char *)g_buff, len);*/

			/*if(lRetVal < len)
			{
				UART_PRINT("Failed during writing the file, Error-code: %d\r\n", \
							 FILE_WRITE_ERROR);
				// Close file without saving
				lRetVal = sl_FsClose(fileHandle, 0, (unsigned char*) "A", 1);
				return lRetVal;
			}*/

			if(len>0)
			{
				iRetVal = FillBuffer(pPlayBuffer, (unsigned char*)g_buff, len);
				if(iRetVal < 0)
				{
					UART_PRINT("Unable to fill buffer");
					LOOP_FOREVER();
				}
			}
			buffersize = GetBufferSize(pPlayBuffer);
			if (buffersize < 10000){
				UART_PRINT("--%d\n\r",buffersize);
			}

			bytesReceived +=len;

			if(bytesReceived >= content_length){
				break;
			}

			/*if ((len - 2) >= 0 && g_buff[len - 2] == '\r' && g_buff [len - 1] == '\n'){
				break;
			}*/

			if(!moreFlag)
			{
				break;
			}
        }else{
        	osi_Sleep(10);
        }
    }

    //
    // If user file has checksum which can be used to verify the temporary
    // file then file should be verified
    // In case of invalid file (FILE_NAME) should be closed without saving to
    // recover the previous version of file
    //

    // Save and close file
    UART_PRINT("Total bytes received: %d\n\r", bytesReceived);
    lRetVal = sl_FsClose(fileHandle, 0, 0, 0);
    ASSERT_ON_ERROR(lRetVal);

    return SUCCESS;
}

static long ServerFileDownload()
{
	long lRetVal = -1;
	struct sockaddr_in addr;
	HTTPCli_Struct cli;

	lRetVal = sl_NetAppDnsGetHostByName((signed char *)HOST_NAME,
	                                       strlen((const char *)HOST_NAME),
	                                       &g_ulDestinationIP,SL_AF_INET);
	    if(lRetVal < 0)
	    {
	        ASSERT_ON_ERROR(GET_HOST_IP_FAILED);
	    }

	    // Set up the input parameters for HTTP Connection
	    addr.sin_family = AF_INET;
	    addr.sin_port = htons(5001);
	    addr.sin_addr.s_addr = sl_Htonl(g_ulDestinationIP);

	    // Testing HTTPCli open call: handle, address params only
	    HTTPCli_construct(&cli);
	    lRetVal = HTTPCli_connect(&cli, (struct sockaddr *)&addr, 0, NULL);
	    if (lRetVal < 0)
	    {
	        UART_PRINT("Connection to server failed\n\r");
		    ASSERT_ON_ERROR(SERVER_CONNECTION_FAILED);
	    }
	    else
	    {
	        UART_PRINT("Connection to server created successfully\r\n");
	    }
	    // Download the file, verify the file and replace the exiting file
	    lRetVal = GetData(&cli);
	    if(lRetVal < 0)
	    {
	        UART_PRINT("Device couldn't download the file from the server\n\r");
	    }

	    HTTPCli_destruct(&cli);

	    return SUCCESS;

}

static long StartHttpServer(){
	long lRetVal;

	//Stop Internal HTTP Server
	lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
	if(lRetVal < 0)
	{
		return lRetVal;
	}

	//Start Internal HTTP Server
	lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
	if(lRetVal < 0)
	{
		return lRetVal;
	}

	uint16_t httpPort;
	unsigned char length = sizeof(httpPort);

	lRetVal = sl_NetAppGet(SL_NET_APP_HTTP_SERVER_ID,NETAPP_SET_GET_HTTP_OPT_PORT_NUMBER, &length, (unsigned char *) &httpPort);
	if(lRetVal < 0)
	{
		UART_PRINT("HTTP server get port failed\n\r");
		return lRetVal;
	}

	UART_PRINT("HTTP server started, port: %d\n\r", httpPort);
	return lRetVal;
}

static long StopHttpServer(){
	long lRetVal;

	//Stop Internal HTTP Server
	lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
	if(lRetVal < 0)
	{
		UART_PRINT("ERROR: stopping HTTP server\n\r");
		return lRetVal;
	}

	UART_PRINT("HTTP server stopped\n\r");
	return lRetVal;
}


void SendUDPNotify(){
	//////create UDP client socket for multicast
	SlSockAddrIn_t  sAddr;
	int             iSockID;
	unsigned long   lLoopCount = 0;
	int             iAddrSize;
	int             iStatus;

	//filling the UDP server socket address
	sAddr.sin_family = SL_AF_INET;
	sAddr.sin_port = sl_Htons((unsigned short)1900);
	sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)0xEFFFFFFA);

	iAddrSize = sizeof(SlSockAddrIn_t);

	// creating a UDP socket
	iSockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
	if( iSockID < 0 )
	{
		// error
		//ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
		UART_PRINT("ERROR: UDP socket create failed\n\r");
		LOOP_FOREVER();
	}


	// for a UDP connection connect is not required
	// sending 10 packets to the UDP server
	while (lLoopCount < 2)
	{


		// sending packet
		iStatus = sl_SendTo(iSockID, UDP_notify1, sizeof(UDP_notify1), 0,
								(SlSockAddr_t *)&sAddr, iAddrSize);

		if( iStatus <= 0 )
		{
			// error
			sl_Close(iSockID);
			//ASSERT_ON_ERROR(UDP_CLIENT_FAILED);
			UART_PRINT("ERROR: UDP socket send failed\n\r");
			LOOP_FOREVER();
		}

		// sending packet
	   iStatus = sl_SendTo(iSockID, UDP_notify2, sizeof(UDP_notify2), 0,
							   (SlSockAddr_t *)&sAddr, iAddrSize);

	   if( iStatus <= 0 )
	   {
		   // error
		   sl_Close(iSockID);
		   //ASSERT_ON_ERROR(UDP_CLIENT_FAILED);
		UART_PRINT("ERROR: UDP socket send failed\n\r");
		LOOP_FOREVER();
	   }

	   // sending packet
	   iStatus = sl_SendTo(iSockID, UDP_notify3, sizeof(UDP_notify3), 0,
							   (SlSockAddr_t *)&sAddr, iAddrSize);

	   if( iStatus <= 0 )
	   {
		   // error
		   sl_Close(iSockID);
		   //ASSERT_ON_ERROR(UDP_CLIENT_FAILED);
		UART_PRINT("ERROR: UDP socket send failed\n\r");
		LOOP_FOREVER();
	   }

	   // sending packet
	   iStatus = sl_SendTo(iSockID, UDP_notify4, sizeof(UDP_notify4), 0,
							   (SlSockAddr_t *)&sAddr, iAddrSize);

	   if( iStatus <= 0 )
	   {
		   // error
		   sl_Close(iSockID);
		   //ASSERT_ON_ERROR(UDP_CLIENT_FAILED);
		UART_PRINT("ERROR: UDP socket send failed\n\r");
		LOOP_FOREVER();
	   }

	   // sending packet
	   iStatus = sl_SendTo(iSockID, UDP_notify5, sizeof(UDP_notify5), 0,
							   (SlSockAddr_t *)&sAddr, iAddrSize);

	   if( iStatus <= 0 )
	   {
		   // error
		   sl_Close(iSockID);
		   //ASSERT_ON_ERROR(UDP_CLIENT_FAILED);
		UART_PRINT("ERROR: UDP socket send failed\n\r");
		LOOP_FOREVER();
	   }

	   // sending packet
	   iStatus = sl_SendTo(iSockID, UDP_notify6, sizeof(UDP_notify6), 0,
							   (SlSockAddr_t *)&sAddr, iAddrSize);

	   if( iStatus <= 0 )
	   {
		   // error
		   sl_Close(iSockID);
		   //ASSERT_ON_ERROR(UDP_CLIENT_FAILED);
		UART_PRINT("ERROR: UDP socket send failed\n\r");
		LOOP_FOREVER();
	   }
		//UART_PRINT("UDP socket send %d\n\r", lLoopCount);

		lLoopCount++;
	}
	UART_PRINT("UDP packets sent..\n\r");

	lLoopCount = 0;
}

// POST üzenetbõl az elérési cím kikeresése
int ParsePOST(char* buf, int len){
	char * pos = buf;
	char * start;
	int cntr = 0;
	while(cntr < len){		// <CurrentURI> tag keresése
		if(*pos == 'C'){
			if(*(pos+10) == '>'){
				start = pos + 18;		// http:// -t levágjuk
				pos = start;
				cntr += 11;
				break;
			}else{
				pos++;
				cntr++;
			}
		}else{
			pos++;
			cntr++;
		}
	}

	while(cntr < len){	// host name
		pos++;
		cntr++;
		if(*pos == ':'){
			memset(HOST_NAME, '\0', sizeof(HOST_NAME));
			strncpy(HOST_NAME,start,pos-start);
			break;
		}
	}

	while(cntr < len){	// rest of url
		pos++;
		cntr++;
		if(*pos == '/'){
			start = pos;
			break;
		}
	}

	while(cntr < len){	// end of url
		pos++;
		cntr++;
		if(*pos == '<'){
			memset(PREFIX_BUFFER, '\0', sizeof(PREFIX_BUFFER));
			strncpy(PREFIX_BUFFER,start,pos-start);
			break;
		}
	}

	if(cntr >= (len-10)){
		return -1;
	}else{
		UART_PRINT("%s\n\r",HOST_NAME);
		UART_PRINT("%s\n\r",PREFIX_BUFFER);
		return 1;
	}


}

//****************************************************************************
//
//! \brief Opening a TCP server side socket and receiving data
//!
//! This function opens a TCP socket in Listen mode and waits for an incoming
//!    TCP connection.
//! If a socket connection is established then the function will try to read
//!    1000 TCP packets from the connected client.
//!
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//
//****************************************************************************
int BsdTcpServer(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             iNewSockID;
    long            lLoopCount = 0;
    long            lNonBlocking = 1;
    int             iTestBufLen;

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 10);
    }

    iTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        // error
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    iAddrSize = sizeof(SlSockAddrIn_t);

    // binding the TCP socket to the TCP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        ASSERT_ON_ERROR(BIND_ERROR);
    }

    // putting the socket for listening to the incoming TCP connection
    iStatus = sl_Listen(iSockID, 0);
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(LISTEN_ERROR);
    }

    // setting socket option to make the socket as non blocking
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                            &lNonBlocking, sizeof(lNonBlocking));
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }
    iNewSockID = SL_EAGAIN;

    // waiting for an incoming TCP connection
    while( iNewSockID < 0 )
    {
        // accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
        iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr,
                                (SlSocklen_t*)&iAddrSize);
        if( iNewSockID == SL_EAGAIN )
        {
           MAP_UtilsDelay(10000);
        }
        else if( iNewSockID < 0 )
        {
            // error
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
    }

    // waits for 10 packets from the connected TCP client
    while (lLoopCount < g_ulPacketCount)
    {
        iStatus = sl_Recv(iNewSockID, g_cBsdBuf, iTestBufLen, 0);
        if( iStatus <= 0 )
        {
          // error
          sl_Close(iNewSockID);
          sl_Close(iSockID);
          ASSERT_ON_ERROR(RECV_ERROR);
        }

        //UART_PRINT(g_cBsdBuf);

        iStatus = ParsePOST(g_cBsdBuf,iStatus);
        if(iStatus == 1){
        	break;
        }

        lLoopCount++;
    }

    Report("Recieved %u packets successfully\n\r",g_ulPacketCount);

    // close the connected socket after receiving from connected TCP client
    iStatus = sl_Close(iNewSockID);
    ASSERT_ON_ERROR(iStatus);
    // close the listening socket
    iStatus = sl_Close(iSockID);
    ASSERT_ON_ERROR(iStatus);

    return SUCCESS;
}


//*****************************************************************************
//
//! Network Task
//!
//! \param  pvParameters - Parameters to the task's entry function
//!
//! \return None
//!
//*****************************************************************************
void Network( void *pvParameters )
{
    long lRetVal = -1;


    //Initialize Global Variable
    InitializeAppVariables();

    //Connect to Network
    lRetVal = ConnectToNetwork();
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Connected to the AP: %s\r\n", SSID_NAME);

    lRetVal = StartHttpServer();
    if(lRetVal < 0)
	{
    	UART_PRINT("ERROR: HTTP server start failed\n\r");
		ERR_PRINT(lRetVal);
		LOOP_FOREVER();
	}

    SendUDPNotify();
    osi_Sleep(5000);

    lRetVal = StopHttpServer();
    if(lRetVal < 0)
	{
		UART_PRINT("ERROR: HTTP server stop failed\n\r");
		ERR_PRINT(lRetVal);
		LOOP_FOREVER();
	}
    osi_Sleep(1000);


    lRetVal = BsdTcpServer(PORT_NUM);
    if(lRetVal < 0)
    {
        UART_PRINT("ERROR: TCP server\n\r");
        LOOP_FOREVER();
    }

    lRetVal = ServerFileDownload();

    if(lRetVal < 0)
    {
        UART_PRINT("Server file download failed\n\r");
        LOOP_FOREVER();
    }
    else
    {
        UART_PRINT("Downloading File Completed\n\r");
    }




#ifdef MULTICAST  
    //Add to Multicast Group
    lRetVal = ReceiveMulticastPacket();
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to Create UDP Server \n\r");
        LOOP_FOREVER();
    }

    //Delete the Networking Task as Service Discovery is not needed
    osi_TaskDelete(&g_NetworkTask);
#else
    //Discover CC3200 Audio Devices  
    //mDNS_Task();
    while(1){
    	//MAP_UtilsDelay(100);
    	osi_Sleep(200);
    }
#endif    

}
