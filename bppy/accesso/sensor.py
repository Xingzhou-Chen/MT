from time import sleep_ms

class AMS:
    def __init__(self,i2c,address,pmin,pmax):
        self.i2c=i2c
        self.address=address
        self.pmax=pmax
        self.pmin=pmin

    def read(self):
        data=self.i2c.readfrom(self.address,4)
        pressureM=data[0]
        pressureL=data[1]
        pressure = (((256*(pressureM&0x3F)+pressureL)-1638.0)*(self.pmax-self.pmin)/13107+self.pmin)
        pressure=pressure*0.75
        pressure=round(pressure,2)
        return pressure

class LWLP:
    def __init__(self,i2c,address,pmax,pmin):
        self.address=address
        self.i2c=i2c
        self.pmax=pmax
        self.pmin=pmin

    def config(self):
            config=b'\xaa\x00\x80'
            self.i2c.writeto(self.address,config)
            sleep_ms(40)

    def read(self):
        self.config()
        data=self.i2c.readfrom(self.address,7)
        SensorDatH=data[1]
        SensorDatM=data[2]
        SensroDat = ((SensorDatH<<8)+(SensorDatM))>>2
        pressure=((self.pmax-self.pmin)/16384.0)*SensroDat+self.pmin
        pressure=pressure*0.0075 
        pressure=round(pressure,2)
        return pressure
    

class HX717:
    def __init__(self,i2c,address):
        self.address=address
        self.i2c=i2c
     
    def read_adc(self):
        data=self.i2c.readfrom(self.address,4)
        pres=(data[1]<<16)+(data[2]<<8)+data[3]
        return pres

    def read(self):
        adccount=self.read_adc()
        pressure=(adccount-8620000)/5318
        pressure=round(pressure,2)
        return pressure
     