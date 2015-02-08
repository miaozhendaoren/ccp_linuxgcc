/*
 * ccp_linuxgcc
 * Copyright (C) 2015  Grzegorz Mazurkiewicz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ccp.h"

#include <stdio.h>

unsigned char gDebugLevel = 255;

unsigned char database[8] = { 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE, 0xDE };

int main(int argc, const char* argv[])
{
	ccpInit();

	unsigned char cmd[8];

	/* connect */
	cmd[0] = CC_CONNECT;
	cmd[1] = 1;
	cmd[2] = 0;
	cmd[3] = 0;

	ccpCommand(cmd);
	printf("======================\n");

	/* get version */
	cmd[0] = 0x1B;
	cmd[1] = 2;

	ccpCommand(cmd);
	printf("======================\n");

	/* set mta */
	cmd[0] = CC_SET_MTA;
	cmd[1] = 3;
	cmd[2] = 0; /* MTA0 */
	cmd[3] = 2; /* address extension */
	cmd[4] = 1;
	cmd[5] = 2;
	cmd[6] = 3;
	cmd[7] = 4;

	ccpCommand(cmd);
	printf("======================\n");

	/* upload */
	cmd[0] = CC_UPLOAD;
	cmd[1] = 4;
	cmd[2] = 5;

	ccpCommand(cmd);
	printf("======================\n");

	return 0;
}

void ccpSend( CCP_BYTEPTR msg )
{
	printf("ccpSend: %02x %02x %02x %02x %02x %02x %02x %02x\n",
	msg[0], msg[1], msg[2], msg[3],
	msg[4], msg[5], msg[6], msg[7]);
	ccpSendCallBack();
}

void ccpUserBackground(void)
{
	printf("ccpUserBackground\n");
}

CCP_MTABYTEPTR ccpGetPointer( CCP_BYTE addr_ext, CCP_DWORD addr )
{
	printf("ccpGetPointer: addr_ext = %u, addr = %lu\n", addr_ext, addr);
	return database;
}
