pwm = []
delim = ','
f = open("pwmtest.csv", "r")
line = f.readline()
pwm = line.split(',')
f.flush()
# for i in len(pwm):
#     pwm[i]=float(pwm)
# # items = items[1:]

# print(len(pwm))

# import gc
# # Later of in a loop
# gc.collect()
# print(gc.mem_free())