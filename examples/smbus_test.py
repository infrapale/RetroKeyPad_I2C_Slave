#!/usr/bin/env python

# https://www.abelectronics.co.uk/kb/article/1094/i2c-part-4---programming-i-c-with-python


def main():
    '''
    Main program function
    '''
buf = [0,0]

if __name__ == "__main__":
    main()
    
from smbus import SMBus
import time

i2cbus = SMBus(1)  # Create a new I2C bus
i2caddress = 0x18  # Address of keypad
RD_KEY_CMND          = 0x05
SET_KEY_VALUES_CMND  = 0x10

New_Key_Values = ['9','E','x','x','x',
                  '0','4','x','x','x',
                  'A','B','x','x','x',
                  'F','G','x','x','x',
                  'x','x','x','x','x'  
                  ]
New_Int_Values = [ord(str) for str in New_Key_Values]
print(New_Key_Values)
print(New_Int_Values)

i2cbus.write_i2c_block_data(i2caddress,SET_KEY_VALUES_CMND, New_Int_Values)

while 1:
    buf = i2cbus.read_i2c_block_data(i2caddress,RD_KEY_CMND,2)
    # duration = i2cbus.read_byte(i2caddress)
    #if buf[0] != 0:
    print(buf)
    time.sleep(1)