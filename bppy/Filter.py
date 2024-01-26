class filter:
    def __init__(self,order,h):
        self.order=order
        self.h=h
        self.output=[]
    def FIR_Filter(self,vi):
        for i in range(len(vi)):
            sum=0
            if i < self.order:
                for j in range(i):
                    sum=sum + self.h[j]*vi[i-j]
            else:      
                for j in range(self.order):
                    sum=sum + self.h[j]*vi[i-j]
                
            self.output.append(sum)   
        return self.output
        
       
