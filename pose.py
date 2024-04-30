import cv2
import mediapipe as mp
import numpy as np
import time
import socket


HOST = '127.0.0.1'     # localhost
PORT = 8888            
sck = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
dest = (HOST, PORT)
sck.connect(dest)

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

imgWidth = int.from_bytes(sck.recv(4), byteorder='big')
imgHeight = int.from_bytes(sck.recv(4), byteorder='big')

print(f'\nRunning pose with screen size: {imgWidth}x{imgHeight}')
try:
  with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while True:
      imgSize = int.from_bytes(sck.recv(4), byteorder='big') # get image size
      
      bytes2Read = 16384 # benchmarked on i7-6700HQ
      imgData = b''
      while len(imgData) != imgSize: # get image
        imgData += sck.recv(bytes2Read)
        remainingData = imgSize - len(imgData)
        bytes2Read = 16384 if remainingData > 16384 else remainingData

      mp3DSize = int.from_bytes(sck.recv(4), byteorder='big') # get if should get MP 3D
      
      bytes2Read = mp3DSize
      mp3D = b''
      while bytes2Read > 0: # get MP 3D or not
        mp3D += sck.recv(bytes2Read)
        bytes2Read = mp3DSize - len(mp3D)

      mp3D = mp3D.decode()

      getMP3D = True if mp3D == "Get MP 3D" else False

      img = np.frombuffer(imgData, dtype=np.uint8).reshape(imgHeight, imgWidth, 3)

      image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
      # To improve performance, optionally mark the image as not writeable to
      # pass by reference.
      image.flags.writeable = False
      results = pose.process(image)

      # Draw the pose annotation on the image.
      # image.flags.writeable = True
      # mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
      # cv2.imshow('Pose', image)

      if not results.pose_landmarks:
        sck.send(int.to_bytes(-2, byteorder='big'))
        continue
        
      landmarks = ""
      for l in results.pose_landmarks.landmark: # converting to int because floating point can vary usage of '.' and ','
          landmarks += f'{int(l.x * (10**7))}; {int(l.y * (10**7))}; {int(l.z * (10**7))}; {int(l.visibility * (10**7))};'

      sck.send(len(landmarks).to_bytes(length=4, byteorder='big'))
      sck.send(landmarks.encode()) 

      if getMP3D:
        landmarks3d = ""
        for l in results.pose_world_landmarks.landmark:
            landmarks3d += f'{int(l.x * (10**7))}; {int(l.y * (10**7))}; {int(l.z * (10**7))}; {int(l.visibility * (10**7))};' 

        sck.send(len(landmarks3d).to_bytes(length=4, byteorder='big'))
        sck.send(landmarks3d.encode())

      # if cv2.waitKey(10) & 0xFF == 27:
      #   break

  # cv2.destroyAllWindows()
except: # when c++ stops, we get some errors. For now, we will be capturing all here
  print("Stopping pose tracking...")
  sck.close()