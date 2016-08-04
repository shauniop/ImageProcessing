import cv2
import numpy
import serial

serialPort = serial.Serial("COM3",9600)

cap = cv2.VideoCapture(0)

#cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,640)
#cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,480)
point = numpy.zeros((50,50,3),numpy.uint8)
point[:] = [64,164,67]
def nothing(x):
    pass

lower = [10,10,10]
upper = [0,0,0]
threshU = 5
threshL = 5

cv2.namedWindow('Trackbar')
cv2.createTrackbar('threshL','Trackbar',0,100,nothing)
cv2.createTrackbar('threshU','Trackbar',0,100,nothing)
def click_for_window(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print "Coordinates :",x,y
        color = image[(y,x)]
        print "clicked "+str(color)
        point[:]=[color[0],color[1],color[2]]

while 1:
    threshL = cv2.getTrackbarPos('threshL','Trackbar')
    threshU = cv2.getTrackbarPos('threshU','Trackbar')
    ret, image = cap.read()
    cv2.imshow('color',point)
    HSV_MAT = cv2.cvtColor(point,cv2.COLOR_BGR2HSV)
    HSV_COLOR = HSV_MAT[(8,8)]
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    #print HSV_COLOR[8,8]
    lower[0] = HSV_COLOR[0]-threshL
    lower[1] = HSV_COLOR[1]-threshL
    lower[2] = HSV_COLOR[2]-threshL
    upper[0] = HSV_COLOR[0]+threshU
    upper[1] = HSV_COLOR[1]+threshU
    upper[2] = HSV_COLOR[2]+threshU
    #dilation = cv2.dilate(erosion,kernel,iterations = 5)
    #print "VALUES : ",lower[0],lower[1],lower[2],upper[0],upper[1],upper[2]
    lowert = numpy.array([lower[0],lower[1],lower[2]])
    uppert = numpy.array([upper[0],upper[1],upper[2]])
    threshold = cv2.inRange(hsv,lowert,uppert)
    kernel = numpy.ones((5,5),numpy.uint8)
    erosion = cv2.erode(threshold,kernel,iterations = 3)
    data = cv2.moments(erosion)
    dx = data['m10']
    dy = data['m01']
    area = data['m00']
    a,b,c,d = cv2.boundingRect(erosion)
    #print a,b,c,d
    img = cv2.rectangle(image,(a,b),(a+c,b+d),(0,0,255),1)
    M = cv2.moments(threshold)
    dx1 = M['m10']
    dy1 = M['m01']
    area1 = int(M['m00'])
    rectArea = c*d
    if area>1000:
        x = int(dx/area)
        y = int(dy/area)
        xDiv = x -320
        yDiv = y -240
        #print "COORDINATES : ",xDiv,yDiv
        print "AREA :",rectArea
        if area1 <  5101275:
            if xDiv > 50 :
                print "GO LEFT",xDiv
                serialPort.write('l')
            elif xDiv<-50 :
                print "GO RIGHT",xDiv
                serialPort.write('r')
            else:
                print "GO FORWARD",xDiv
                serialPort.write('f')
        else:
           print "go back",xDiv
           serialPort.write('b')
    cv2.imshow('THRESHOLD',erosion)
    cv2.setMouseCallback('IMAGE',click_for_window)
    

    cv2.imshow('IMAGE',image)
    if(cv2.waitKey(1)==32):break
cap.release()
cv2.destroyAllWindows()
serialPort.close()
