last_err=0
next_err=0
err=0

class PID_Controller:
    def __init__(self,P_end,T_end,Kp,Ki,Kd,min_pwm,max_pwm):
        self.P_end=P_end
        self.T_end=T_end
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.min_pwm=min_pwm
        self.max_pwm=max_pwm

    def calculate_ref(self,P_init,current_t):
        k=(self.P_end-P_init)/self.T_end
        P_ref=k*current_t+P_init
        return P_ref 

    def PID_control(self,P_init,current_p,current_t):
        global err
        global last_err
        global next_err
        set_point=self.calculate_ref(P_init,current_t)
        err=set_point-current_p
        P=self.Kp*(err-last_err)
        I=self.Ki*err
        D=self.Kd*(err+next_err-2*last_err)
        u=P+I+D
        next_err=last_err
        last_err=err
        return u   
    
    def calibrate(self,v):
        if(v>self.max_pwm or v<self.min_pwm): return self.max_pwm
        else: return v