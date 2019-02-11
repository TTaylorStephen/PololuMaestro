**For more detailed information on Polulu Maestro download and read the users guide found @ [https://www.pololu.com/docs/pdf/0J40/maestro.pdf]**

**Intructions for setting up Maestro board on Linux:**

**1. To set certain defaults, you must download the Maestro Control Center and the Maestro command-line utility (UscCmd). These can be found here :**
  [https://www.pololu.com/file/download/maestro-linux-150116.tar.gz?file_id=0J315]
  
  -Download the file and uzip using “tar -xzvf” followed by the name of the file. Follow the instruction in the README.txt to complete install.

**2. After download you can set default parameters for the board in channel settings.**

  -board must be in dual port mode for use.
  
  -full range of pulses for position are from 64-2567, where 2567 correspond to a value of 0x07 or 1 byte. 
  
  -read/write data sent/received as 2 bytes of type unsigned short
  
  -command bytes have MSB set (128-255 or 0x80)
  
  -data bytes have MSB cleared (0-127 or 0x00)
  
  -each data byte only transmits 7 bits of information.



