from machine import I2C,Pin,PWM,SoftI2C
# import machine 
from utime import sleep
import time 


# i2c = I2C(id=0,scl=Pin(7),sda=Pin(6),freq=100000)
i2c = I2C(id=0,scl=Pin(9),sda=Pin(8),freq=100000)
# i2c = SoftI2C(Pin(9),Pin(8))

# def read2():
#     i2c0.start()
#     i2c0.writeto(0x00,b'00')

if __name__== '__main__':
#     current_pressure=read(sensor_address,0,range_max)
#     buf = bytearray(1)
#     buf[0]=0
#     print(buf[0])
#     i2c0.scan()
#     while True:
#         i2c.start()
#         i2c.write(0x01)
#     I2C.init(scl=Pin(9), sda=Pin(8), freq=100000)
#     writeadr=0x00<<1|0
#     readadr=0x00<<1|1
#     i2c.start()
#     i2c.writeto(0x00,b'\x01')
    i2c.readfrom(0x00,1)
#     print(ack)
#     i2c.readfrom(0x01,4)
#     i2c.write(b'\x00')
#     print(ack)
#     ack1=i2c.write(b'\xAA')
#     print(ack1)
#     i2c.write(b'\x00')
#     i2c.write(b'\x80')
#     i2c.stop()
#     sleep(0.3)
#     
#     i2c.start()
#     ack=0
# #     while ack==0:
# #         ack=i2c.write(b'\x01')
# #         print(ack)
#     i2c.readfrom(0x01,1)
#     print(ack1)

#     i2c.readfrom(0x01,2)




    


