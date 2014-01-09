/*! 
\file  b64.c
\brief This file contains some simple 8bit-to-6bit encoding/deconding routines

Most of these routines are outdated and should be converted using glibc's equivalent
routines.

\date   Started 2/22/05
\author George
\version\verbatim $Id: b64.c 10711 2011-08-31 22:23:04Z karypis $ \endverbatim

\verbatim 
$Copyright$ 
$License$
\endverbatim

*/


#include "GKlib.h"

#define B64OFFSET       48      /* This is the '0' number */


/******************************************************************************
* Encode 3 '8-bit' binary bytes as 4 '6-bit' characters
*******************************************************************************/
void encodeblock(unsigned char *in, unsigned char *out)
{
  out[0] = (in[0] >> 2);
  out[1] = (((in[0] & 0x03) << 4) | (in[1] >> 4));
  out[2] = (((in[1] & 0x0f) << 2) | (in[2] >> 6));
  out[3] = (in[2] & 0x3f);

  out[0] += B64OFFSET;
  out[1] += B64OFFSET;
  out[2] += B64OFFSET;
  out[3] += B64OFFSET;

//  printf("%c %c %c %c %2x %2x %2x %2x %2x %2x %2x\n", out[0], out[1], out[2], out[3], out[0], out[1], out[2], out[3], in[0], in[1], in[2]);
}

/******************************************************************************
* Decode 4 '6-bit' characters into 3 '8-bit' binary bytes
*******************************************************************************/
void decodeblock(unsigned char *in, unsigned char *out)
{   
  in[0] -= B64OFFSET;
  in[1] -= B64OFFSET;
  in[2] -= B64OFFSET;
  in[3] -= B64OFFSET;

  out[0] = (in[0] << 2 | in[1] >> 4);
  out[1] = (in[1] << 4 | in[2] >> 2);
  out[2] = (in[2] << 6 | in[3]);
}


/******************************************************************************
* This function encodes an input array of bytes into a base64 encoding. Memory
* for the output array is assumed to have been allocated by the calling program
* and be sufficiently large. The output string is NULL terminated.
*******************************************************************************/
void GKEncodeBase64(int nbytes, unsigned char *inbuffer, unsigned char *outbuffer)
{
  int i, j;

  if (nbytes%3 != 0)
    gk_errexit(SIGERR, "GKEncodeBase64: Input buffer size should be a multiple of 3! (%d)\n", nbytes);

  for (j=0, i=0; i<nbytes; i+=3, j+=4) 
    encodeblock(inbuffer+i, outbuffer+j);

//printf("%d %d\n", nbytes, j);
  outbuffer[j] = '\0';
}



/******************************************************************************
* This function decodes an input array of base64 characters into their actual
* 8-bit codes. Memory * for the output array is assumed to have been allocated 
* by the calling program and be sufficiently large. The padding is discarded.
*******************************************************************************/
void GKDecodeBase64(int nbytes, unsigned char *inbuffer, unsigned char *outbuffer)
{
  int i, j;

  if (nbytes%4 != 0)
    gk_errexit(SIGERR, "GKDecodeBase64: Input buffer size should be a multiple of 4! (%d)\n", nbytes);

  for (j=0, i=0; i<nbytes; i+=4, j+=3) 
    decodeblock(inbuffer+i, outbuffer+j);
}

