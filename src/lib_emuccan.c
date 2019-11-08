#include <linux/string.h>
#include <linux/module.h> /*build in kernel 2017.1.11*/
#include "lib_emuccan.h"

#define ID_SET_BAUDRATE     0x3A
#define ID_SET_FEEDBACK     0x3B
#define ID_TRANSMIT_DATA    0x3C
#define ID_RECEIVE_DATA     0x3D

#define EMUC_RESET          999

//#ifndef hex_asc_lo /*build in kernel 2017.1.11*/

#define FIX_ARRAY_TO_PTR    /* Allen 2016.12.05 */

/* Change kernel functions name by Tony 2017.1.18.
Add "_" in front of the names */

static const char _hex_asc[] = "0123456789ABCDEF";

#if defined(FIX_ARRAY_TO_PTR)
   #define _hex_asc_lo(x) *(_hex_asc + (x & 0x0f))
   #define _hex_asc_hi(x) *(_hex_asc + ((x & 0xf0) >> 4))
#else
   #define _hex_asc_lo(x) _hex_asc[((x) & 0x0f)]
   #define _hex_asc_hi(x) _hex_asc[((x) & 0xf0) >> 4]
#endif

static inline char *_hex_byte_pack(char *p, unsigned char byte)
{
	*p++ = _hex_asc_hi(byte);
	*p++ = _hex_asc_lo(byte);
	return p;
}

static int _hex_to_bin(char ch)
{

	//printk("emuc : %c\n", ch);

	if ((ch >= '0') && (ch <= '9'))
		return ch - '0';
	if ((ch >= 'a') && (ch <= 'f'))
		return ch - 'a' + 10;
	if ((ch >= 'A') && (ch <= 'F'))
		return ch - 'A' + 10;
		
	return -1;
}

static int _hex2bin(unsigned char *dst, const char *src, unsigned int count)
{
	while (count--) {
		int hi = _hex_to_bin(*src++);
		int lo = _hex_to_bin(*src++);
		if ((hi < 0) || (lo < 0))
			return -1;

		*dst++ = (hi << 4) | lo;
	}
	return 0;
}

/*
static char * _bin2hex(char *dst, const void *src, unsigned int count)
{
	const unsigned char *_src = src;
	while (count--)
		dst = _hex_byte_pack(dst, *_src++);
	return dst;
}
*/

//#endif /*build in kernel 2017.1.11*/

static int lrc_check(char *str, int len)
{
	int i;
	char tmp[3];
	unsigned int lrc;

   #if defined(FIX_ARRAY_TO_PTR)
	   // the length includin 2 bytes lrc and \r\n, so, -4
	   for (i = 0, lrc = 0; i < len - 4; i++)
		   lrc += (unsigned char) *(str + i);
	
	   lrc -= 0x01;
	   _hex_byte_pack(tmp, (unsigned char)(lrc & 0xff));

      if(!(*(str + len - 4) == *tmp) || !(*(str + len - 3) == *(tmp+1)))
		   return -1;
   #else
	   // the length includin 2 bytes lrc and \r\n, so, -4
	   for (i = 0, lrc = 0; i < len - 4; i++)
		   lrc += (unsigned char)str[i];
	
	   lrc -= 0x01;
	   _hex_byte_pack(tmp, (unsigned char)(lrc & 0xff));

	   if (!(str[len - 4] == tmp[0]) || !(str[len - 3] == tmp[1]))
		   return -1;
   #endif
	
	return 0;
}

static void lrc_calc(char *str, int len)
{
	unsigned int lrc;
	int i;

   #if defined(FIX_ARRAY_TO_PTR)
	// length include 2 bytes lrc and \r\n, so, -4
	   for (i = 0, lrc = 0; i < len - 4; i++)
		   lrc += (unsigned char) *(str + i);
   #else   
	   // length include 2 bytes lrc and \r\n, so, -4
	   for (i = 0, lrc = 0; i < len - 4; i++)
		   lrc += (unsigned char)str[i];
   #endif
	
	lrc -= 0x01;
	_hex_byte_pack(str + len - 4, (unsigned char)(lrc & 0xff));
}

void EMUCSendHex(EMUC_CAN_FRAME *frame)
{
	int i;
	char *p;
	unsigned char func;

	p = (char *)frame->com_buf;
	memset(p, 0, sizeof(frame->com_buf));

	/* CMD */
	*p++ = ID_TRANSMIT_DATA;
	/* CHANNEL */
	p = _hex_byte_pack(p, (unsigned char)frame->channel);

	/* FUNC (DLC, MOD, RTR) */
	if (frame->rtr == 0)
		func = (frame->dlc << 2);
	else
		func = 0;
	if (frame->mod) func |= 0x01;
	if (frame->rtr) func |= 0x02;
	p = _hex_byte_pack(p, (unsigned char)func);

   #if defined(FIX_ARRAY_TO_PTR)
	   /* ID */
	   for (i = 0; i < ID_LEN; i++)
		   p = _hex_byte_pack(p, *(frame->id + i));
	   /* DATA */
	   for (i = 0; i < DATA_LEN; i++)
		   p = _hex_byte_pack(p, *(frame->data + i));
   #else
	   /* ID */
	   for (i = 0; i < ID_LEN; i++)
		   p = _hex_byte_pack(p, frame->id[i]);
	   /* DATA */
	   for (i = 0; i < DATA_LEN; i++)
		   p = _hex_byte_pack(p, frame->data[i]);
   #endif

	/* LRC (reserved) */
	*p++ = '0';
	*p++ = '0';
	/* Command end with '\r\n' */
	*p++ = 0x0D;
	*p++ = 0x0A;

	/* Caculate LRC */
	lrc_calc((char *)frame->com_buf, p - (char *)frame->com_buf);
}	

int EMUCRevHex(EMUC_CAN_FRAME *frame)
{
	int i;
	unsigned char func;
	char *p = frame->com_buf;

	if (!lrc_check(frame->com_buf, strlen(frame->com_buf))) {
		/* CHANNEL */
		p++;
		if (_hex2bin((unsigned char *)&frame->channel, p, 1) < 0)
			return -2;
		/* FUNC */
		p += 2;
		if (_hex2bin(&func, p, 1) < 0)
			return -3;
		frame->mod = (func & 0x01) ? 1 : 0;
		frame->rtr = (func & 0x02) ? 1 : 0;
		frame->dlc = (int)(func << 2) >> 4;

      #if defined(FIX_ARRAY_TO_PTR)
		   /* ID */
		   for (i = 0; i < ID_LEN; i++) {
			   p += 2;
			   if (_hex2bin(frame->id + i, p, 1) < 0)
				   return -4;
		   }
		   /* DATA */
		   for (i = 0; i < DATA_LEN; i++) {
			   p += 2;
			   if (_hex2bin(frame->data + i, p, 1) < 0)
				   return -5;
		   }
      #else
		   /* ID */
		   for (i = 0; i < ID_LEN; i++) {
			   p += 2;
			   if (_hex2bin(&frame->id[i], p, 1) < 0)
				   return -4;
		   }
		   /* DATA */
		   for (i = 0; i < DATA_LEN; i++) {
			   p += 2;
			   if (_hex2bin(&frame->data[i], p, 1) < 0)
				   return -5;
		   }
      #endif

		return 0;
	}
	return -1;
}

