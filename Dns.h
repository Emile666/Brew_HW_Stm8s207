#ifndef DNSClient_h
#define DNSClient_h
/*==================================================================
  File Name: Dns.h
  Author   : (c) Copyright 2009-2010 MCQN Ltd.
             STM8 C-version by Emile
  ------------------------------------------------------------------
  Purpose  : Header-file for Dns.c
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

#define SOCKET_NONE	255
// Various flags and header field values for a DNS message
#define UDP_HEADER_SIZE		 (8)
#define DNS_HEADER_SIZE		 (12)
#define TTL_SIZE        	 (4)

#define QUERY_FLAG               (0)
#define RESPONSE_FLAG            (0x8000)
#define QUERY_RESPONSE_MASK      (0x8000)
#define OPCODE_STANDARD_QUERY    (0)
#define OPCODE_INVERSE_QUERY     (1<<11)
#define OPCODE_STATUS_REQUEST    (2<<11)
#define OPCODE_MASK              (15<<11)
#define AUTHORITATIVE_FLAG       (1<<10)
#define TRUNCATION_FLAG          (1<<9)
#define RECURSION_DESIRED_FLAG   (1<<8)
#define RECURSION_AVAILABLE_FLAG (1<<7)
#define RESP_NO_ERROR            (0)
#define RESP_FORMAT_ERROR        (1)
#define RESP_SERVER_FAILURE      (2)
#define RESP_NAME_ERROR          (3)
#define RESP_NOT_IMPLEMENTED     (4)
#define RESP_REFUSED             (5)
#define RESP_MASK                (15)
#define TYPE_A                   (0x0001)
#define CLASS_IN                 (0x0001)
#define LABEL_COMPRESSION_MASK   (0xC0)
// Port number that DNS servers listen on
#define DNS_PORT        	 (53)

// Possible return codes from ProcessResponse
#define SUCCESS          (1)
#define TIMED_OUT        (-1)
#define INVALID_SERVER   (-2)
#define TRUNCATED        (-3)
#define INVALID_RESPONSE (-4)

void     dns_begin(uint8_t *aDNSServer);
int      dns_inet_aton(const char *aIPAddrString, uint8_t *aResult);
int      dns_getHostByName(const char *aHostname, uint8_t *aResult);
uint16_t dns_BuildRequest(const char* aName);
int16_t  dns_ProcessResponse(uint16_t aTimeout, uint8_t *aAddress);

#endif
