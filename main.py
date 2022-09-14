import random
import cv2 as cv
import numpy as np

import serial
import time

face_cascade = cv.CascadeClassifier('venv/lib/haarcascade_frontalface_default.xml')
print("Pan Tilt running opening camera...")

cap = cv.VideoCapture(2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

noFaceCount = 0
fileCount = 0
lastTimeSaved = time.time() * 1000


def sgn(x):
    if x < 0:
        return -1
    if x > 0:
        return 1
    return 0


def getFace():
    global lastTimeSaved, fileCount, noFaceCount
    # Capture frame-by-frame
    for i in range(30): # frame doesn't always seem to be current so read for a bit
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            exit()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 6, minSize=(60, 60)) # scale factor, # of neighbors
    if len(faces) > 0:
        x, y, w, h = faces[0]
        mask = np.zeros(frame.shape[:2], dtype="uint8")
        cv.circle(mask, (int(x + w / 2), int(y + h / 2)), int(max(w / 2, h / 2)), 255, -1)
        masked = cv.bitwise_and(frame, frame, mask=mask)
        crop_img = masked[y:y + h, x:x + w]
        newimage = cv.resize(crop_img, (480, 480))
        myBack = np.zeros((480, 640, 3), dtype="uint8")
        newimage = cv.putText(newimage, "warning", (0, 105), cv.FONT_ITALIC, 4,
                              (0, 0, 255), 12, cv.LINE_AA, False)
        myBack[0:480, 320 - 240:320 + 240] = newimage

        cv.imshow('img', myBack)
        print("writing file")
        cv.imwrite("/tmp/test/test" + str(fileCount) + ".png", myBack)
        fileCount += 1
    else:
        print(str(noFaceCount) + ": no face detected")
        cv.imshow('img', frame)

        noFaceCount += 1
    if cv.waitKey(1) == ord('q'):
        exit()


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)
    print("Pan Tilt running...")
    time.sleep(2)
    packet = bytearray()
    currentPan = 100
    currentTilt = 100

    while True:
        newPan = random.randint(50, 140)
        newTilt = random.randint(110, 150)
        speed = random.randint(1, 10) / 300.0
        while (currentPan != newPan) or (currentTilt != newTilt):
            currentPan += sgn(newPan - int(currentPan))
            currentTilt += sgn(newTilt - int(currentTilt))
            packet.append(255)
            packet.append(currentPan)
            packet.append(currentTilt)
            ser.write(packet)
            packet.clear()
            time.sleep(speed)
        getFace()
        time.sleep(random.random() * 3)
