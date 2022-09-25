import random
from typing import Any

import cv2 as cv
import numpy as np
import subprocess
import socket
import serial
import time
import logging

logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                    level=logging.DEBUG)

face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
logging.info("Pan Tilt running opening camera...")
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
testing = True
if testing:
    ipaddr = "localhost"
else:
    ipaddr = "172.16.1.255"

cap = cv.VideoCapture(2)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

noFaceCount = 0
fileCount = 0


def sgn(x):
    if x < 0:
        return -1
    if x > 0:
        return 1
    return 0


def run_bash_command(cmd: str) -> Any:
    process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    if error:
        raise Exception(error)
    else:
        return output


def scpCopy(myFileCount):
    if not testing:
        for i in range(12):
            run_bash_command("scp /tmp/warning" + str(myFileCount) +
                             ".mp4 172.16.1." + str(i) + ":/tmp/.")
        run_bash_command("scp /tmp/warning" + str(myFileCount) +
                         ".mp4 172.16.1.255:/tmp/.")


def doTheFaceStuff(faceImg):
    global fileCount

    if logging.INFO >= logging.root.level:
        cv.imshow('img', faceImg)
    logging.info("writing file")
    cv.imwrite("/tmp/warning" + str(fileCount) + ".png", faceImg)
    run_bash_command("rm /tmp/warning" + str(fileCount) + ".mp4")
    run_bash_command("ffmpeg -loop 1 -i /tmp/warning" + str(fileCount) +
                     ".png -c:v libx264 -t 15 -pix_fmt yuv420p -vf scale=320:240 /tmp/warning" +
                     str(fileCount) + ".mp4")
    scpCopy(fileCount)

    # trigger Pd to show the movie (image):
    fileCountAsc = 48 + fileCount
    sock.sendto(fileCountAsc.to_bytes(1, "big") + b";\n", (ipaddr, 4000))
    fileCount = (fileCount + 1) % 10


def getFace():
    global noFaceCount
    # Capture frame-by-frame
    for i in range(30):  # frame doesn't always seem to be current so read for a bit
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            exit()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 6, minSize=(60, 60))  # scale factor, # of neighbors
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
        doTheFaceStuff(myBack)
    else:
        logging.info(str(noFaceCount) + ": no face detected")
        if logging.INFO >= logging.root.level:
            cv.imshow('img', frame)

        noFaceCount += 1
    if cv.waitKey(1) == ord('q'):
        exit()


if __name__ == '__main__':

    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=5)
    except:
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=5)

    print("Pan Tilt running...")
    time.sleep(2)
    packet = bytearray()
    currentPan = 100
    currentTilt = 100
    runTime = time.time()

    while True:
        time.sleep(.1)
        if ser.in_waiting:
            if ser.readline()[0] == 49:  # human detected
                runTime = time.time() + 15 * 60 # run until 15 min after human detected
                print("human detected")
        if time.time() <= runTime:
            newPan = random.randint(50, 140)
            newTilt = random.randint(65, 105)  # range 40 to 115
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
