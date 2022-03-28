import cv2
import numpy as np
import serial
import time
import imutils
from jetsonvideostream import JetsonVideoStream
import sys
sys.path.append('imutils/imutils/video')


class Video():
    def __init__(self, resolution=(960, 460)):
        self.frameResolution = resolution
        self.__HorizontalFOV = 62
        self.__VerticalFOV = 37
        self.__video = None
        self.__actualFrame = None
        self.initCamera()

    def initCamera(self):
        self.__video = JetsonVideoStream(outputResolution=self.frameResolution)
        self.__video.start()
        time.sleep(2.0)
        frame = self.__video.read()
        try:
            height, width = frame.shape[0:2]
        except:
            print('##################################################')
            print('## Camera NOT initilized properly, use command: ##')
            print('##    sudo systemctl restart nvargus-daemon     ##')
            print('##################################################')
            self.clean()
            sys.exit(0)
        print('## Camera initilized properly ##')

    def read(self):
        self.__actualFrame = self.__video.read()
        return self.__actualFrame

    @property
    def frame(self):
        return self.__actualFrame

    @property
    def HorizontalFOV(self):
        return self.__HorizontalFOV

    @property
    def VerticalFOV(self):
        return self.__VerticalFOV

    def clean(self):
        self.__video.stop()
        time.sleep(1.0)
        print("## Camera closed successfuly ##")


class Serial():
    def __init__(self):
        self.__serial = None
        self.initSerial()

    def initSerial(self, port='/dev/ttyACM0', baudrate=115200):
        if self.__serial is not None:
            self.__serial.close()
        self.__serial = serial.Serial(
            port=port, baudrate=baudrate, timeout=0.05)
        print("## Serial initialized on port " + port + " ##")

    def send(self, operation: str, a=0, b=0):
        if self.__serial is None:
            print("## Serial NOT initialized ##")
            return
        packet = '<' + operation + ', ' + str(a) + ', ' + str(b) + '>'
        packetBytes = bytes(packet, 'utf-8')
        self.__serial.write(packetBytes)

    def clean(self):
        self.__serial.close()
        print("## Serial closed successfuly ##")


class Color():
    def __init__(self):
        self.__colorRanges = []

    def addRange(self, lowerColor=(0, 0, 0), upperColor=(360, 100, 100)):
        h = lowerColor[0]//2
        s = (lowerColor[1] * 255)//100
        v = (lowerColor[2] * 255)//100
        lowerColor = (h, s, v)
        h = upperColor[0]//2
        s = (upperColor[1] * 255)//100
        v = (upperColor[2] * 255)//100
        upperColor = (h, s, v)
        self.__colorRanges.append((lowerColor, upperColor))

    def preprocessMask(self, frame, iterations=4):
        blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        frame_HSV = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        resoult = None
        for i, color in enumerate(self.__colorRanges):
            if i == 0:
                resoult = cv2.inRange(frame_HSV, color[0], color[1])
            else:
                mask = cv2.inRange(frame_HSV, color[0], color[1])
                resoult = cv2.add(resoult, mask)

        kernel = np.ones((4, 4), np.uint8)
        resoult = cv2.morphologyEx(resoult, cv2.MORPH_OPEN, kernel)
        resoult = cv2.dilate(resoult, kernel, iterations=iterations)
        resoult = cv2.morphologyEx(resoult, cv2.MORPH_CLOSE, kernel)
        #resoult = cv2.erode(resoult, kernel, iterations=iterations)
        return resoult

    def getContours(self, image, mode=cv2.RETR_EXTERNAL):
        contours = cv2.findContours(image, mode, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        return contours

    def getBoundingBoxes(self, contours, minArea):
        boundingBoxes = []

        if contours:
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            for i, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > (minArea):
                    x, y, w, h = cv2.boundingRect(contour)
                    boundingBoxes.append((x, y, w, h))

        return boundingBoxes


class Detect():
    def __init__(self):
        self.tolerance = 0.2

    @property
    def tolerance(self):
        return self.__tolerance

    @tolerance.setter
    def tolerance(self, tolerance: float):
        self.__tolerance = tolerance

    def isCircle(self, contour) -> bool:
        M = cv2.moments(contour)
        cX, cY = 0, 0
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        r = []
        R = 0
        i = 0
        for x in contour:
            i += 1
            try:
                val = pow((x[0][0] - cX), 2) + pow((x[0][1] - cY), 2)
            except:
                val = 1
            r.append(val)
            R += val
        R /= i
        j = 0
        for t in r:
            j += abs((t/R) - 1)
        j /= i
        if j < self.tolerance:
            return True
        return False


class Movment(Serial):
    def __init__(self):
        super().__init__()

    def __translate(self, value, oldMin, oldMax, newMin=-100, newMax=100):
        # Figure out how 'wide' each range is
        oldRange = oldMax - oldMin
        newRange = newMax - newMin
        NewValue = (((value - oldMin) * newRange) / oldRange) + newMin
        return int(NewValue)

    def centerOnPoint(self, videoObject, biggestObjectMidPoint):
        height, width = videoObject.frame.shape[0:2]
        screenMidPoint = width//2, height//2
        distanceVector = tuple(
            map(lambda x, y: x - y, biggestObjectMidPoint, screenMidPoint))

        hFOV = videoObject.HorizontalFOV
        vFOV = videoObject.VerticalFOV
        yaw = self.__translate(
            distanceVector[0], -width//2, width // 2, -hFOV//2, hFOV//2)  # up-down
        yawError = yaw / (hFOV/2)
        pitch = self.__translate(
            distanceVector[1], -height//2, height//2, -vFOV//2, vFOV//2)  # left-right
        pitchError = pitch / (vFOV/2)
        return yawError, pitchError

    def setCamera(self, yaw, pitch):
        self.send('servo', yaw, pitch)

    def setMotor(self, yaw):
        self.send('follow', yaw)

    def forwardMotor(self):
        self.send('StartMotor', 50)

    def stopMotor(self):
        self.send('StopMotor')


class Draw():
    def __init__(self):
        pass

    def drawBoundingBoxes(self, image, boundingBoxes):
        if len(boundingBoxes) <= 0:
            return
        self.drawBigest(image, boundingBoxes[0])
        if len(boundingBoxes) > 1:
            self.drawRegular(image, boundingBoxes[1:])

    def drawBigest(self, image, boundingBox):
        x, y, w, h = boundingBox
        midPoint = ((x + w//2), (y + h//2))
        cv2.rectangle(image, (x, y), ((x+w), (y+h)),
                      (0, 0, 255), thickness=3)
        cv2.circle(image, midPoint,
                   4, (0, 0, 255), thickness=3)

    def drawRegular(self, image, boundingBoxes):
        for i in boundingBoxes:
            x, y, w, h = i
            cv2.rectangle(image, (x, y), ((x+w), (y+h)),
                          (0, 255, 0), thickness=2)


def main(debug=False, video_path="/home/jetson/output", save_video=False):
    frame_count = 0
    vs = Video()

    color = Color()
    color.addRange((330, 40, 25), (360, 100, 95))  # red
    # color.addRange((0, 40, 35), (5, 100, 70))  # red
    color.addRange((110, 30, 10), (180, 100, 95)) # green
    detect = Detect()
    detect.tolerance = 0.16

    ser = Movment()
    ser.send('start')

    draw = Draw()

    paused = False
    while True:
        if not paused:
            # get frame
            frame = vs.read()
            height, width = frame.shape[0:2]

            # color
            mask = color.preprocessMask(frame)

            # contours
            contours = color.getContours(mask)

            # contours that are circle
            contours = [x for x in contours if detect.isCircle(x)]

            # crop smallest contours
            minimalContourArea = (width * height)/256

            # boundings
            boundingBoxes = color.getBoundingBoxes(
                contours, minimalContourArea)

            if len(boundingBoxes) > 0:
                # midpoint of tracking object
                x, y, w, h = boundingBoxes[0]
                biggestObjectMidPoint = ((x + w//2), (y + h//2))

                # camera tracking object
                yaw, pitch = ser.centerOnPoint(vs, biggestObjectMidPoint)
                ser.setCamera(yaw, pitch)

                # car tracking object
                ser.setMotor(yaw)

                # drive forward when object on center
                ser.forwardMotor()

                # stop when object is close
                if (height//3 < h):
                    ser.stopMotor()

            else:
                # no object detected
                ser.stopMotor()

            if debug:
                # draw all boundings
                draw.drawBoundingBoxes(frame, boundingBoxes)

                cv2.imshow("video", frame)
                cv2.imshow("mask", mask)

            if save_video:
                frame_count += 1
                cv2.imwrite(
                    (video_path + '/video/frame_'+str(frame_count)+'.jpg'), frame)
                cv2.imwrite(
                    (video_path + '/mask/frame_'+str(frame_count)+'.jpg'), mask)

        # key events
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):  # quit
            break
        elif key == ord('p'):  # pause
            paused = not paused
        elif key == ord('d'):  # stop camera movment
            ser.send('stop')
        elif key == ord('f'):  # start camera movment
            ser.send('start')

    ser.clean()
    vs.clean()
    cv2.destroyAllWindows()
    sys.exit(0)


if __name__ == "__main__":
    main()
    # main(debug=True)
