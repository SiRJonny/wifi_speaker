/*
 * ActionTask.c
 *
 *  Created on: 2016 okt. 6
 *      Author: Csabi
 */


#include "ActionTask.h"
#include "gpio_if.h"
#include "simplelink.h"
#include "common.h"



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


// tcp socket:
char g_cBsdBuf2[BUF_SIZE];

char PREFIX_BUFFER2[200] = "/441khz.wav";
char HOST_NAME2[25] = "csmcs.uw.hu";


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
			memset(HOST_NAME2, '\0', sizeof(HOST_NAME2));
			strncpy(HOST_NAME2,start,pos-start);
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
			memset(PREFIX_BUFFER2, '\0', sizeof(PREFIX_BUFFER2));
			strncpy(PREFIX_BUFFER2,start,pos-start);
			break;
		}
	}

	if(cntr >= (len-10)){
		return -1;
	}else{
		UART_PRINT("%s\n\r",HOST_NAME2);
		UART_PRINT("%s\n\r",PREFIX_BUFFER2);
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
//****************************************************************************/
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
        g_cBsdBuf2[iCounter] = (char)(iCounter % 10);
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

    Report("waiting for incoming TCP connection...\n\r");

    // waiting for an incoming TCP connection
    while( iNewSockID < 0 )
    {
        // accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
        iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr,
                                (SlSocklen_t*)&iAddrSize);
        if( iNewSockID == SL_EAGAIN )
        {
           osi_Sleep(100);
        }
        else if( iNewSockID < 0 )
        {
            // error
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
    }

    Report("Recieving TCP packets...\n\r");

    // waits for 10 packets from the connected TCP client
    while (1)
    {
        iStatus = sl_Recv(iNewSockID, g_cBsdBuf2, iTestBufLen, 0);
        if( iStatus <= 0 )
        {
          // error
          sl_Close(iNewSockID);
          sl_Close(iSockID);
          ASSERT_ON_ERROR(RECV_ERROR);
        }

        //UART_PRINT(g_cBsdBuf);
        //UART_PRINT("ParsePOST: %s\r\n", g_cBsdBuf);
        iStatus = ParsePOST(g_cBsdBuf2,iStatus);

        /*if(iStatus == 1){
        	break;
        }*/

        osi_Sleep(100);
        lLoopCount++;

    }

    Report("Recieved packets successfully\n\r");

    // close the connected socket after receiving from connected TCP client
    iStatus = sl_Close(iNewSockID);
    ASSERT_ON_ERROR(iStatus);
    // close the listening socket
    iStatus = sl_Close(iSockID);
    ASSERT_ON_ERROR(iStatus);

    return SUCCESS;
}





void ActionTask( void *pvParameters )
{

	while(1){
		osi_Sleep(15000);
		UART_PRINT("asdf...\n\r");
		BsdTcpServer(801);


		//osi_Sleep(1250);


	}
}
