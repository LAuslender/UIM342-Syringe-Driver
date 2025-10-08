
/*////////////////////////////////////////////////////////////////////////////
MIT License

Copyright (c) [2022] UIROBOT

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Disclaimer: UIROBOT shall not be held responsible for any direct or indirect
consequences resulting from the misuse of this software, including but not
limited to damages caused by unauthorized purchases, improper configurations,
or unintended usage. Users are solely responsible for ensuring the proper and
safe application of this software in their respective environments.
////////////////////////////////////////////////////////////////////////////*/

#pragma once

#include "Arduino.h"
#include "mcp2515.h"



class SimpleCAN
{
	public:



	can_frame ReadMLConfig(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data);
	can_frame ReadSNConfig(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data);
	can_frame ReadPPConfig(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data);

	can_frame SetToUim342(unsigned long id, unsigned char  len, unsigned char* data);
	can_frame CommandSet(unsigned char pid, unsigned char cid, unsigned char cw, unsigned char datalen, unsigned char* data);
};
