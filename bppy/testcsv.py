csvdata = []
delim = ','
with open('add.csv','r') as file:
    for line in file:
        data=line.rstrip('\n').rstrip('\r').split(delim)
        csvdata.append(float(data[0]))
print(type(csvdata[1]))