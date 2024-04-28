from time import sleep_ms


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
    
