#HW 1
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import math as m
def movmean(a,l): #Function called to calculate the moving averages
i=0
mov_avg=[] #Array to store the moving averages
e=0
while i < len(a) - l + 1:
win=a[i:i+l] #Creating a window array to store 2,4,8...128 elements at a time
win_avg=sum(win)/l #Computing the average
mov_avg.append(win_avg) #Appending to a new array
e+=1
i+=1
return mov_avg
if __name__ == "__main__":
pitch = [] #Array to store the raw data
c=0
count=[] #Array to store the X values
mov_pitch=[] #Array to store the moving averages
mean=0 #Variable to compute the mean
s=0
sd=0 #Variable to compute the Standard Deviation
j=2
file = open('/home/jayesh/Downloads/imudata.txt', 'r')
lines = file.readlines() #Reading the imudata.txt file
for line in lines:
pitch.append(line.split()[4]) #Moving the 5th column in the file containing the pitch
angle to the array
count.append(c)
c+=1
for i in range(0, len(pitch)):
pitch[i] = int(pitch[i]) #Converting the string values to integer
plt.plot(count, pitch, label='raw data') #Plotting the raw data
while j<129: #Loop to compute the 2,4,8...128- moving averages
mov_pitch=movmean(pitch,j) #Calling the function to compute moving averages
by passing the array containing the raw data and the window length
count1=[]
for i in range(0, len(mov_pitch)):
count1.append(i)
mean=sum(mov_pitch)/len(mov_pitch) #Computing the mean
for i in range(0,len(mov_pitch)):
s+=((mov_pitch[i]-mean)**2)
sd=(s/len(mov_pitch))**(1/2) #Computing the standard deviation
label= "{Point}- Pt moving average, Mean = {mean}, SD = {sd}".format(Point = j,
mean = mean, sd=sd)
plt.plot(count1, mov_pitch, label=(label)) #Plotting the different moving averages
j*=2
plt.title('Accelerometer Plot')
plt.xlabel('Instance')
plt.ylabel('Degrees')
plt.legend()
plt.show()
