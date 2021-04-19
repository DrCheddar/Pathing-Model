import matplotlib.pyplot as plt                                                 
import numpy as np
import cmath
import math
import copy
from typing import TypeVar, Generic, List, NamedTuple
from math import e
## Create functions and set domain length
Point = complex

PointList = List[complex]

PhaseCoord = List[complex];
PhaseList = List[PhaseCoord];


def doubleArrayCopy( arr : PointList) -> PointList:
    # //size first dimension of array
    a = copy.deepcopy(arr);
    return a



def DefinePath(startPoint : Point, endPoint : Point,  startAngle : int) -> PointList :
    disp = endPoint - startPoint;
    # // disp between the 2 points
    distance = abs(disp) / 4;
    midwayPoint = startPoint + distance * Point(np.cos(startAngle), np.sin(startAngle));
    a =  [startPoint, midwayPoint, endPoint];
    return a;


def InjectPoints(path : PointList, spacing : float) -> PointList :
    # // spacing is space between 2 points that r injected
    newPointList : PointList = [path[0]];
    # // additional points for line 1: solve for linear equation between both points 1 and 2
    # // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    Disp = path[1] - path[0];
    distance1 = abs(Disp);
    i = 0;
    while i < distance1:
        newPoint = path[0] + Disp * i / distance1;
        newPointList.append(newPoint);
        i+= spacing;
        
    # // additional points for line 1: solve for linear equation between both points 1 and 2
    # // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    Disp2 = path[2] - path[1];
    distance2 = abs(Disp2);
    i = 0;
    while i < distance2:
        newPoint = path[1] + Disp2 * i / distance2;
        newPointList.append(newPoint);
        i+= spacing;
    newPointList.append(path[2]);
    return newPointList;

def smoother(path : PointList, weight_data :  float, weight_smooth : float, tolerance : float) -> PointList : 
    # //copy array

    newPath = copy.deepcopy(path);
    change = tolerance;
    print("lengths" + str(newPath.__len__()) + " " + str(path.__len__()))
    while (change >= tolerance):

        change = 0.0;
        for i in range(1, len(path)-1):
            auxR = newPath[i].real;
            newPath[i] += Point((weight_data * (path[i] - newPath[i]) + weight_smooth * (newPath[i - 1] + newPath[i + 1] - (2.0 * newPath[i]))).real, 0);
            change += abs(auxR - newPath[i].real);

            auxI = newPath[i].imag;
            newPath[i] += Point(0, (weight_data * (path[i] - newPath[i]) + weight_smooth * (newPath[i - 1] + newPath[i + 1] - (2.0 * newPath[i]))).imag);
            change += abs(auxI - newPath[i].imag);


    return newPath;

def GeneratePath(startpoint: complex, endpoint : complex, startAngle : float, spacing : float) : 
    start = startpoint
    end = endpoint
    a0 = startAngle * math.pi 
    # a0 = startAngle * math.pi + math.pi/2
    r = ((start.real - end.real)**2 + (start.imag-end.imag)**2) / (2*(start.real - end.real)*np.cos(a0) + 2*(start.imag - end.imag)*np.sin(a0))
    a = math.atan2(((start.real - end.real)**2 - (start.imag-end.imag)**2)*np.sin(a0) - 2*(start.real - end.real)*(start.imag-end.imag)*np.cos(a0), ((start.imag - end.imag)**2 - (start.real - end.real)**2)*np.cos(a0) - 2*(start.real - end.real)*(start.imag-end.imag)*np.sin(a0)) - a0
    center = start - cmath.rect(r, a0);
    newPointList : PointList = []
        
        
    i = 0.0
    angleDiff = a;
    print(a, a0, r, angleDiff)

    for i in range(100):
        t = i/100
        newPoint = center + cmath.rect(r, a0 + a*t)
        newPointList.append(newPoint)
    return newPointList
    
def modAngle(angle : float, piBy = 1):
    return np.remainder(angle + np.pi * piBy, 2* np.pi * piBy) - np.pi * piBy


state = [pos, heading, velocity, motorInputs, wheelSpeeds]

feedIn = [targetPositions]

output = [leftVoltage, rightVoltage]



MAX_ACCEL = 2; # inch  s^-2
ROLLING_RESISTANCE = 1; # kg s ^ -1?
ROBOT_WIDTH = 5; # inch

wheel_accel = voltage * MAX_ACCEL - wheelSpeed * ROLLING_RESISTANCE;

def GeneratePath2(startpoint: complex, endpoint : complex, startAngle : float, spacing : float) : 
    start = startpoint
    end = endpoint
    startAngle = modAngle(startAngle);
    # a0 = startAngle * math.pi
    a0 = modAngle(startAngle-0.5*math.pi);

    r = ((start.real - end.real)**2 + (start.imag-end.imag)**2) / (2*(start.real - end.real)*np.cos(a0) + 2*(start.imag - end.imag)*np.sin(a0))
    a = math.atan2(((start.real - end.real)**2 - (start.imag-end.imag)**2)*np.sin(a0) - 2*(start.real - end.real)*(start.imag-end.imag)*np.cos(a0), ((start.imag - end.imag)**2 - (start.real - end.real)**2)*np.cos(a0) - 2*(start.real - end.real)*(start.imag-end.imag)*np.sin(a0)) - a0
    center = start - cmath.rect(r, a0);
    newPointList : PointList = []
    phaseList : PhaseList = []
    i = 0.0
    angleDiff = modAngle(a,2);
    print(a, a0, r, angleDiff)
    prevPoint = startpoint;
    while(abs(i/r) < abs(angleDiff)):

        newPoint = center + cmath.rect(r, a0 + i/r)
        phase = [newPoint, newPoint - prevPoint]
        phaseList.append(phase);
        newPointList.append(newPoint)
        prevPoint = newPoint;
        i+=  spacing
    return newPointList

def GeneratePhase(startpoint: complex, endpoint : complex, startAngle : float, spacing : float) : 
    start = startpoint
    end = endpoint
    startAngle = modAngle(startAngle);
    # a0 = startAngle * math.pi
    a0 = modAngle(startAngle-0.5*math.pi);

    r = ((start.real - end.real)**2 + (start.imag-end.imag)**2) / (2*(start.real - end.real)*np.cos(a0) + 2*(start.imag - end.imag)*np.sin(a0))
    a = math.atan2(((start.real - end.real)**2 - (start.imag-end.imag)**2)*np.sin(a0) - 2*(start.real - end.real)*(start.imag-end.imag)*np.cos(a0), ((start.imag - end.imag)**2 - (start.real - end.real)**2)*np.cos(a0) - 2*(start.real - end.real)*(start.imag-end.imag)*np.sin(a0)) - a0
    center = start - cmath.rect(r, a0);
    newPointList : PointList = []
    phaseList : PhaseList = []
    i = 0.0
    angleDiff = modAngle(a,2);
    print(a, a0, r, angleDiff)
    prevPoint = startpoint;
    while(abs(i/r) < abs(angleDiff)):
        newPoint = center + cmath.rect(r, a0 + i/r)
        phase = [newPoint, newPoint - prevPoint]
        phaseList.append(phase);
        prevPoint = newPoint;
        i+=  spacing
    return phaseList

# accel = voltage - l * velocity
# in theory, but could be weird with turning?

Path = GeneratePhase(Point(0,0), Point(36, 36), math.pi/2, 2)



# def GeneratePath(startpoint: complex, endpoint : complex, startAngle : float, spacing : float) :
#     path = DefinePath(startpoint, endpoint, startAngle);
#     path = InjectPoints(path, spacing);
#     # Second Picture
#     path = smoother(path, 0.1, 0.9, 1);
#     return path;

# Path = GeneratePath(Point(0,0), Point(36, 36), 0, 2)

for point in Path:
    # plt.plot(point[0].real, point[0].imag, 'or')
    # plt.arrow
    # plt.plot('or')
    plt.arrow(point[0].real, point[0].imag, (point[1]).real, (point[1]).imag, head_width = 0.2)
    
# for point in Path2:
#     plt.plot(point.real, point.imag, 'bo')
# reee = complex(0,0);
# x = np.arange(0.0, 2.0, 0.01)
# y = x**2
# dy = 2*x - 1

# ## Plot functions and a point where they intersect
# plt.plot(x, y)
# plt.plot(x, dy)
# plt.plot(1, 1, 'or')

## Config the graph
plt.title('A Cool Graph')
# plt.xlabel('X')
# plt.ylabel('Y')
#plt.ylim([0, 4])
plt.grid(True)
# plt.legend(['y = x^2', 'y = 2x'], loc='upper left')

## Show the graph
plt.show()
