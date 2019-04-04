#include "cfg_seaduck.h"
#ifdef HAS_XMODEM
#include <stdlib.h>

#include <SdFat.h>
#include "XModem.h"
#include <elapsedMillis.h>

// Number of seconds until giving up hope of receiving sync packets from
// host.
#define SYNC_TIMEOUT 10
// Number of times we try to send a packet to the host until we give up
// sending..
#define MAX_RETRY    4

#define ACK_TIMEOUT 10

/* Initialize XModem session */
XModem::XModem(Stream *port, char mode)
{
  packetNo = 1;
  crcBuf = 0;
  checksumBuf = 0;
  filepos = 0;
  packetLen = 128; // Default number of payload bytes
  if (mode == ModeYModem)	{
    this->mode = ModeYModem;
  } else {
    this->mode = ModeXModem;
  }
  this->port = port;
}

/* Send out a byte of payload data,
 * includes checksumming
 */
void XModem::outputByte(unsigned char inChar)
{
  char j;
  checksumBuf += inChar;

  crcBuf = crcBuf ^ (int) inChar << 8;
  j = 8;
  do
    {
      if (crcBuf & 0x8000)
        crcBuf = crcBuf << 1 ^ 0x1021;
      else
        crcBuf = crcBuf << 1;
    } while(--j);
  
  port->write(inChar);
}

/* Wait for either C or NACK as a sync packet.
 * Determines protocol details, like block size
 * and checksum algorithm.
 */
char XModem::sync(void)
{
  char tryNo;
  char inChar;
  // Wait a second every time until timeout.
  // The magic char expected from the host is 'C' or
  // NAK
  port->setTimeout(1000);
  tryNo = 0;
  do
    {
      port->readBytes(&inChar, 1);
      tryNo++;
      // When timed out, leave immediately
      if (tryNo == SYNC_TIMEOUT)
        return(-1);
    } while ((inChar != 'C') && (inChar != NAK));
  // Determine which checksum algorithm to use
  // this routine also determines the packet length
  this->packetLen = 128;
  if (inChar == NAK)
    oldChecksum=1;
  else
    { // This is assumed to be 'C'
      oldChecksum=0;
      // Check if there is a K after the C, that means we can do 1024 byte blocks
      // Give sender a short time to send the K
      port->setTimeout(100);
      tryNo=0;
      port->readBytes(&tryNo, 1);
      if (tryNo=='K')
        {
          this->packetLen = 1024;
        }
      // Reset the timeout to one second
      port->setTimeout(1000);
    }
  
  return(0);
}

/** Wait for the remote to acknowledge or cancel.
  * Returns the received char if no timeout occured or
  * a CAN was received. In this cases, it returns -1.
  **/
char XModem::waitACK(void)
{
  char i, inChar;
  
  i = 0;
  do
    {
      port->readBytes(&inChar, 1);
      i++;
      if (i>ACK_TIMEOUT) // RH: had been 200
        return(-1);
      if (inChar == CAN)
        return(-1);
    } while ((inChar != NAK) && (inChar != ACK) && (inChar != 'C'));
  return(inChar);
}

void XModem::sendFile(SdFile &dataFile, const char *fileName)
{
	unsigned char finished=0;
	char inChar;
	int i;
	unsigned char tryNo;
        char fileSize[16];

	// Rewind data file before sending the file..
	dataFile.seekSet(0);

        // RH: original code would sync before *and* after the YModem
        // packet 0.  seems unnecessary.
        if (this->sync()!=0)
          goto err;
		
	// When doing YModem, send block 0 to inform host about 
	// file name to be received
	if (this->mode == ModeYModem)
	{
		// Send header for virtual block 0 (file name)
		port->write(SOH);
		port->write((uint8_t)0);
		port->write(0xFF);

                // a little unclear how file size is to be added
                // either: filename is the first 64 bytes, nul padded,
                // other places suggested filename could be up to 256.
                // filesize is the next 16.
                // or they are separated by a nul (I think that's more likely)
                // here limit the filename to fit in the packet, along with
                // up to 16 bytes of filesize string, and a NUL between
                // based on the SOH above, this packet is always a 128-byte
                // packet.
		for (i=0; (i<strlen(fileName)) && (i<128-1-16) ; i++)
		{
                  this->outputByte(fileName[i]);
		}
                // null-terminate that
                this->outputByte(0);
                i++;

                // include YMODEM filesize
                itoa(dataFile.fileSize(),fileSize,10);
                for(int j=0;(j<strlen(fileSize));j++,i++) {
                  this->outputByte(fileSize[j]);
                }
                
		for (; i<128; i++)
		{
			this->outputByte((uint8_t)0x00);
		}
		if (oldChecksum){
                  // RH: used to XOR, but that doesn't match any docs
                  // or other codes I've seen.
                  port->write((char)checksumBuf);
                } else {
                  port->write((char) (crcBuf >>8));
                  port->write((char) (crcBuf & 0xFF));
		}
		// Discard ACK/NAK/CAN, in case
		// we communicate to an XMODEM-1k client
		// which might not know about the 0 block.
		waitACK();
	} else {
          //
	}

        // RH: no longer sync before and after ymodem packet --
        // just before.
	// if ( this->sync() !=0)
	// 	goto err;

        // RH: no! sync will update oldChecksum, and this
        // could silently revert it.
	// oldChecksum =  (inChar == NAK);

	while (!finished)
	{
		filepos = dataFile.curPosition();

		// Sending a packet will be retried
		tryNo = 0;
		do
		{
			// Seek to start of current data block, 
			// will advance through the file as
			// block will be acked..
			dataFile.seekSet(filepos);

			// Reset checksum stuff
			checksumBuf = 0x00;
			crcBuf = 0x00; 

			// Try to send packet, so header first
			if (packetLen == 128)
				port->write(SOH);
			else
				port->write(STX);

			port->write(packetNo);
			port->write(~packetNo);
			for (i = 0; i<packetLen; i++)
			{
				inChar = dataFile.read();
				this->outputByte(inChar);

                                // apparently some SdFat.h don't have available(), so
                                // use this:
                                finished=(dataFile.fileSize()==dataFile.curPosition());
				// finished = !dataFile.available();
                                
				// Pad file with zeroes
				if (finished)
					inChar = 0x00;

			}
			// Send out checksum, either CRC-16 CCITT or
			// classical inverse of sum of bytes. 
			// Depending on how the received introduced himself
			if (oldChecksum){
                          // RH: used to xor, but I don't think that's correct.
                          port->write((char)checksumBuf);
                        } else {
                          port->write((char) (crcBuf >>8));
                          port->write((char) (crcBuf & 0xFF));
			}

			inChar = waitACK();
			tryNo++;
			if (tryNo > MAX_RETRY)
				goto err;
		} while (inChar != ACK);
		
		packetNo++;
	}
	// Send EOT and wait for ACK
	tryNo = 0;
	do
	{
		port->write(EOT);
		inChar = waitACK();
		tryNo++;
		// When timed out, leave immediately
		if (tryNo == SYNC_TIMEOUT)
			goto err;
	} while (inChar != ACK);

	// When we get here everything was successful.
	port->println("Transfer complete...");
        return;
err:
	port->println("Error sending...");
}

#endif // HAS_XMODEM
