#ifndef DHCP5500_H
#define DHCP5500_H
/*==================================================================
  File Name: Dhcp.h
  Author   : Jordan Terrell - blog.jordanterrell.com,
             STM8 C-version by Emile
  ------------------------------------------------------------------
  Purpose  : This is the header-file for Dhcp.c
  ------------------------------------------------------------------
  This file is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include <stdint.h>

/* DHCP state machine. */
#define STATE_DHCP_START     (0)
#define	STATE_DHCP_DISCOVER  (1)
#define	STATE_DHCP_REQUEST   (2)
#define	STATE_DHCP_LEASED    (3)
#define	STATE_DHCP_REREQUEST (4)
#define	STATE_DHCP_RELEASE   (5)

#define DHCP_FLAGSBROADCAST	0x8000

/* UDP port numbers for DHCP */
#define	DHCP_SERVER_PORT	(67)	/* from server to client */
#define DHCP_CLIENT_PORT	(68)	/* from client to server */

/* DHCP message OP code */
#define DHCP_BOOTREQUEST	(1)
#define DHCP_BOOTREPLY		(2)

/* DHCP message type */
#define	DHCP_DISCOVER		(1)
#define DHCP_OFFER		(2)
#define	DHCP_REQUEST		(3)
#define	DHCP_DECLINE		(4)
#define	DHCP_ACK		(5)
#define DHCP_NAK		(6)
#define	DHCP_RELEASE		(7)
#define DHCP_INFORM		(8)

#define DHCP_HTYPE10MB		(1)
#define DHCP_HTYPE100MB		(2)

#define DHCP_HLENETHERNET	(6)
#define DHCP_HOPS		(0)
#define DHCP_SECS		(0)

#define MAGIC_COOKIE		(0x63825363)
#define MAX_DHCP_OPT	        (16)

#define HOST_NAME 		"WIZnet"
#define DEFAULT_LEASE		(900)

#define DHCP_CHECK_NONE         (0)
#define DHCP_CHECK_RENEW_FAIL   (1)
#define DHCP_CHECK_RENEW_OK     (2)
#define DHCP_CHECK_REBIND_FAIL  (3)
#define DHCP_CHECK_REBIND_OK    (4)

#define DHCP_TIMEOUT 	        (12000)
#define DHCP_RESPONSE_TIMEOUT   (8000)

enum
{
    padOption		       = 0,
    subnetMask		       = 1,
    timerOffset		       = 2,
    routersOnSubnet		   = 3,
    /* timeServer		   = 4,
    nameServer		       = 5,*/
    dns			           = 6,
    /*logServer		       = 7,
    cookieServer		   = 8,
    lprServer		       = 9,
    impressServer		   = 10,
    resourceLocationServer = 11,*/
    hostName		       = 12,
    /*bootFileSize		   = 13,
    meritDumpFile		   = 14,*/
    domainName		       = 15,
    /*swapServer		   = 16,
    rootPath		       = 17,
    extentionsPath		   = 18,
    IPforwarding		   = 19,
    nonLocalSourceRouting  = 20,
    policyFilter		   = 21,
    maxDgramReasmSize	   = 22,
    defaultIPTTL		   = 23,
    pathMTUagingTimeout	   = 24,
    pathMTUplateauTable	   = 25,
    ifMTU			       = 26,
    allSubnetsLocal		   = 27,
    broadcastAddr		   = 28,
    performMaskDiscovery   = 29,
    maskSupplier		   = 30,
    performRouterDiscovery = 31,
    routerSolicitationAddr = 32,
    staticRoute		       = 33,
    trailerEncapsulation   = 34,
    arpCacheTimeout		   = 35,
    ethernetEncapsulation  = 36,
    tcpDefaultTTL		   = 37,
    tcpKeepaliveInterval   = 38,
    tcpKeepaliveGarbage	   = 39,
    nisDomainName		   = 40,
    nisServers		       = 41,
    ntpServers		       = 42,
    vendorSpecificInfo	   = 43,
    netBIOSnameServer	   = 44,
    netBIOSdgramDistServer = 45,
    netBIOSnodeType		   = 46,
    netBIOSscope		   = 47,
    xFontServer		       = 48,
    xDisplayManager		   = 49, */
    dhcpRequestedIPaddr	   = 50,
    dhcpIPaddrLeaseTime	   = 51,
    /*dhcpOptionOverload   = 52,*/
    dhcpMessageType		   = 53,
    dhcpServerIdentifier   = 54,
    dhcpParamRequest	   = 55,
    /*dhcpMsg			   = 56,
    dhcpMaxMsgSize		   = 57,*/
    dhcpT1value		       = 58,
    dhcpT2value		       = 59,
    /*dhcpClassIdentifier  = 60,*/
    dhcpClientIdentifier   = 61,
    endOption			   = 255
};

typedef struct _RIP_MSG_FIXED
{
	uint8_t  op; 
	uint8_t  htype; 
	uint8_t  hlen;
	uint8_t  hops;
	uint32_t xid;
	uint16_t secs;
	uint16_t flags;
	uint8_t  ciaddr[4];
	uint8_t  yiaddr[4];
	uint8_t  siaddr[4];
	uint8_t  giaddr[4];
	uint8_t  chaddr[6];
} RIP_MSG_FIXED;

int     request_DHCP_lease(void);
void    reset_DHCP_lease(void);
void    presend_DHCP(void);
void    send_DHCP_MESSAGE(uint8_t messageType, uint16_t secondsElapsed);
void dhcp_printByte(char *buf, uint8_t n);
uint8_t parseDHCPResponse(unsigned long responseTimeout, uint32_t *transactionId);
int     dhcp_begin(uint8_t *mac);
int     dhcp_checkLease(void);

#endif
