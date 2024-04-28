class Circuit():
    def __init__(self,pump_pin,valve_pin):
        self.pump=pump_pin
        self.valve=valve_pin

    def pump_on(self):
        self.pump.value(1)

    def pump_off(self):
        self.pump.value(0)
    
    def valve_off(self):
        self.valve.duty_u16(65535)

    def valve_on(self,release_speed):
        self.valve.duty_u16(release_speed)

    def release(self):
        self.valve.duty_u16(0)