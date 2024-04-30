import cv2
import mediapipe as mp
import numpy as np
import time
import redis

r = redis.Redis(host='localhost', port=6379)
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

imgWidth = int(r.get('width').decode())
imgHeight = int(r.get('height').decode())

print(f'\nRunning pose with screen size: {imgWidth}x{imgHeight}')
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
  while True:
    data = r.get('image')

    while not data:
      time.sleep(0.05)
      data = r.get('image')

    r.set('image', '')

    img = np.frombuffer(data, dtype=np.uint8).reshape(imgHeight, imgWidth, 3)

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
      r.set('noLandmarks', 'true')
      continue

    r.set('noLandmarks', 'false')
      
    landmarks = ""
    for l in results.pose_landmarks.landmark: # converting to int because floating point can vary usage of '.' and ','
        landmarks += f'{int(l.x * (10**7))}; {int(l.y * (10**7))}; {int(l.z * (10**7))};' 

    landmarks3d = ""
    for l in results.pose_world_landmarks.landmark:
        landmarks3d += f'{int(l.x * (10**7))}; {int(l.y * (10**7))}; {int(l.z * (10**7))};'
        
    r.set('landmarks', landmarks)
    r.set('landmarks3d', landmarks3d)

      
    # if cv2.waitKey(10) & 0xFF == 27:
    #   break

    time.sleep(0.005)

# cv2.destroyAllWindows()
