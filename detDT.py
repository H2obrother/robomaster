#!/usr/bin/python
import numpy as np
import cv2
import time
import math
import serial

#ser = serial.Serial ("/dev/ttyUSB0")    #Open named port 
#ser.baudrate = 9600

BLUE = True
RED  = False
MyColor  = BLUE
def sendPoint(point):
    if point == None:
        return None
    x = point[0]
    y = point[1]
    print(x,y)
    s = 'SSS'+format(x,'03d')+format(y,'03d')+'E'
    
    ser.write(s)                 #Send back the received data
    #receive = ser.read(8)
    #print(receive)
        
cap = cv2.VideoCapture(1)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,1000)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,1000)
ret, frame = cap.read()

maxCol = 480
maxRow = 640
maxArea = maxCol * maxCol

def slope(line):
    #prameter:[[x1,y1],[x2,y2]],(y1-y2)/(x1-x2)
    if(line[0][0] == line[1][0]):
        return 2000
    tan = abs((line[0][1]*1.0 - line[1][1])/(line[0][0] - line[1][0]))
    return tan

def isRectParallel(rect1,rect2):
    return abs(rect1[2] - rect2[2]) < 8
def isSameArea(rect1,rect2):
    perimeter = (rect1[1][0] + rect1[1][1] + rect2[1][0] + rect2[1][1]) 
    return abs(rect1[1][0] * rect1[1][1] -  rect2[1][0] * rect2[1][1]) < 2000
def isCenterNearby(rect1,rect2):
    perimeter = (rect1[1][0] + rect1[1][1] + rect2[1][0] + rect2[1][1])
    distance = math.sqrt((rect1[0][0]-rect2[0][0])**2 + (rect1[0][1]-rect2[0][1])**2)
    return  distance < perimeter*1.5  and distance > perimeter*0.5 
def balckPointRate(frame,p1,p2):
    Vp1p2 = [p2[0]-p1[0]]
def DTcolor(frame,box):
    numFitPoint = 0
    for point in box:
        x = point[0]
        y = point[1]
        if y >= maxCol:
            y = maxCol - 1
        if x >= maxRow:
            x = maxRow - 1
        if frame[y][x][0] > frame[y][x][2]:#if dengTiao is BLUE
            numFitPoint +=1
    return (BLUE if numFitPoint >2 else RED)
def findMidPoint(frame):
    filteredCont = []
    approxCont = []
    boxes = []
    rects = []
    centerSlope = 1000
    finalPair = []
    midPoint = None
    t1 = cv2.getTickCount()
    
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray,220,255,cv2.THRESH_BINARY)
    #thresh = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
    #        cv2.THRESH_BINARY_INV,11,2)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) 

    # Display the resulting frame
    
    numCont = len(contours)
    if numCont == 0:
        return None
    for cnt in contours:
        contArea = cv2.contourArea(cnt)
        rect = cv2.minAreaRect(cnt)
        #print('contArea > 60',contArea > 60  , '  contArea < 25000:',contArea < 25000 , '  rect[1][1] > rect [1][0]:',rect[1][1] > rect [1][0])
        boxTemp = cv2.cv.BoxPoints(rect)
        boxTemp = np.int0(boxTemp)
        if  contArea > 75  and contArea < 25000 and MyColor!=DTcolor(frame,boxTemp):
            filteredCont.append(cnt)
            rects.append(rect)
    if len(rects) == 0:
        return None
    for i in range(len(rects)):
        direction1 = rects[i][1][1] > rects[i][1][0]
        for rect2 in rects[i+1:]:
            direction2 = rect2[1][1] > rect2[1][0]
            #print('isRectParallel:',isRectParallel(rects[i],rect2) ,'  isSameArea:' ,isSameArea(rects[i],rect2) , '  isCenterNearby:',isCenterNearby(rects[i],rect2))
            if isRectParallel(rects[i],rect2) and isSameArea(rects[i],rect2) and isCenterNearby(rects[i],rect2) and direction1==direction2:
                line = [rects[i][0],rect2[0]]
                  #cv2.line(frame,(int(math.floor(rects[i][0][0])),int(math.floor(rects[i][0][1]))),(int(math.floor(rect2[0][0])),int(math.floor(rect2[0][1]))),(0,255,0),2)

                box = cv2.cv.BoxPoints(rects[i])
                box = np.int0(box)
                boxes.append(box)
                box = cv2.cv.BoxPoints(rect2)
                box = np.int0(box)
                boxes.append(box)
                tempSlope = slope([[int(math.floor(rects[i][0][0])),int(math.floor(rects[i][0][1]))],[int(math.floor(rect2[0][0])),int(math.floor(rect2[0][1]))]])
                if centerSlope > tempSlope: 
                    centerSlope = tempSlope
                    box1 = cv2.cv.BoxPoints(rects[i])
                    box1 = np.int0(box1)
                    midPoint= (int((math.floor(rects[i][0][0])+math.floor(rect2[0][0]))/2.0),int((math.floor(rects[i][0][1])+math.floor(rect2[0][1]))/2.0))
                    box2 = cv2.cv.BoxPoints(rect2)
                    box2 = np.int0(box2)
                    finalPair=[box1,box2]
                    t2 = cv2.getTickCount()
                    time = (t2 - t1) / cv2.getTickFrequency()
                    FPS = 1.0 / time
        cv2.drawContours(frame,filteredCont,-1,(0,255,0),3)
        cv2.imshow('frame',frame)
        return midPoint
##while(True):
##    # Capture frame-by-frame
##    ret, frame = cap.read()
##
##    midPoint = findMidPoint(frame)
##    if midPoint == None:
##        continue
##    #cv2.drawContours(frame,filteredCont,-1,(0,255,0),3)
##    cv2.circle(frame,midPoint,10,(255,255,0),5)
##    print(frame.shape)
##    #cv2.drawContours(frame,finalPair,-1,(255,0,0),3)
##    cv2.imshow('frame',frame)
##    #cv2.imshow('gray',thresh)
##    #sendPoint(midPoint)
##    if cv2.waitKey(1) & 0xFF == ord('q'):
##        break

# When everything done, release the capture
##cap.release()
##cv2.destroyAllWindows()
##ser.close()
