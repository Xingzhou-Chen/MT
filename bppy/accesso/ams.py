
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
