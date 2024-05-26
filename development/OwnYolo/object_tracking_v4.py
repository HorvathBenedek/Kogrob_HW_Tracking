#!/usr/bin/env python3


import cv2
import numpy as np
#from object_detection import ObjectDetection
from temo.object_detection import ObjectDetection
import math


def drawboundingboxes(img):
    (class_ids, scores, boxes) = od.detect(img)
    detect_images = []
    center_points_cur_frame = []
    i = 0

    for box in boxes:
        (x, y, w, h) = box
        print(x,y,w,h)
        
        #temp =  img[0:(x+w),0:(y+h)]
        x1 = x
        x2 = x + w
        y1 = y
        y2 = y + h
        print(x1,x2,y1,y2)
        temp = []
        #tempim =  img[x1:x2,y1:y2]
        #temp =  img[x1:x2,y1:y2]
        temp =  img[y1:y2,x1:x2]
        #tempim =  img[100:200,100:200]
        #tempim =  img[x:(x+w),y:(y+h)]
        #print(np.shape(temp))

        
        #cv2.imshow(tempim)
        filename = "outpics/" + str(i) + ".jpg"
        #cv2.imwrite(filename, tempim)
        cv2.imwrite(filename, temp)
        '''
        cx = int((x + x + w) / 2)
        cy = int((y + y + h) / 2)
        center_points_cur_frame.append((cx, cy))        
        #temp =  img[(x-w):(x+w),(y-h):(y+h)]

        #detect_images = detect_images.append(img[x:(x+w),y:(y+h)])
        #cv2.imshow("i", temp)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        '''
        i = i+1
    return img    

print(cv2.__version__)

# Initialize Object Detection
od = ObjectDetection()

#cap = cv2.VideoCapture("los_angeles.mp4")
img = cv2.imread("pics/Képernyőkép 2024-05-21 110015.png")
count = 0
center_points_prev_frame = []

tracking_objects = {}
track_id = 0

img = drawboundingboxes(img)
cv2.imshow("Image", img)

while True:
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()

'''
while True:

    img = drawboundingboxes(img)
    cv2.imshow("Image", img)

    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
'''


'''
while True:
    #ret, frame = cap.read()
    #count += 1
    #if not ret:
    #    break

    # Point current frame
    frame = img
    center_points_cur_frame = []

    # Detect objects on frame
    (class_ids, scores, boxes) = od.detect(frame)
    
    for box in boxes:
        (x, y, w, h) = box
        cx = int((x + x + w) / 2)
        cy = int((y + y + h) / 2)
        center_points_cur_frame.append((cx, cy))
        #print("FRAME N°", count, " ", x, y, w, h)

        # cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Only at the beginning we compare previous and current frame
    '''
'''
    if count <= 2:
        for pt in center_points_cur_frame:
            for pt2 in center_points_prev_frame:
                distance = math.hypot(pt2[0] - pt[0], pt2[1] - pt[1])

                if distance < 20:
                    tracking_objects[track_id] = pt
                    track_id += 1
    else:

        tracking_objects_copy = tracking_objects.copy()
        center_points_cur_frame_copy = center_points_cur_frame.copy()

        for object_id, pt2 in tracking_objects_copy.items():
            object_exists = False
            for pt in center_points_cur_frame_copy:
                distance = math.hypot(pt2[0] - pt[0], pt2[1] - pt[1])

                # Update IDs position
                if distance < 20:
                    tracking_objects[object_id] = pt
                    object_exists = True
                    if pt in center_points_cur_frame:
                        center_points_cur_frame.remove(pt)
                    continue

            # Remove IDs lost
            if not object_exists:
                tracking_objects.pop(object_id)

        # Add new IDs found
        for pt in center_points_cur_frame:
            tracking_objects[track_id] = pt
            track_id += 1

    for object_id, pt in tracking_objects.items():
        cv2.circle(frame, pt, 5, (0, 0, 255), -1)
        cv2.putText(frame, str(object_id), (pt[0], pt[1] - 7), 0, 1, (0, 0, 255), 2)

    print("Tracking objects")
    print(tracking_objects)
    '''
'''
    print("CUR FRAME LEFT PTS")
    print(center_points_cur_frame)


    cv2.imshow("Frame", frame)

    # Make a copy of the points
    center_points_prev_frame = center_points_cur_frame.copy()

    key = cv2.waitKey(1)
    if key == 27:
        break

#cap.release()
cv2.destroyAllWindows()
'''