# -*- coding: utf-8 -*-
import cv2, os
import sys
import time

rt = '../../PreData/Origin-Test/'
out_dir = '../../PreData/'
frame_rate = 30

fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')

for i in range(1, 151):

    print(('foward'+'{x}').format(x=i))  
    id = str(i)
    # path for background frames
    video = cv2.VideoWriter(os.path.join(out_dir,id + '.mp4')
                            , fourcc, 30, (800, 410))

    if os.path.exists(os.path.join(rt, id+'.mp4')):
        cap = cv2.VideoCapture(os.path.join(rt, id+'.mp4'))

    ret, frame = cap.read()

    # build MOG2 model
    bs = cv2.createBackgroundSubtractorMOG2(120, 16, False)

    count = 0
    while ret:

        count += 1
        cap.set(cv2.CAP_PROP_POS_MSEC, 1.0/frame_rate * 1000 * count)
        ret, frame = cap.read()
        if ret == False:
            break

        fg_mask = bs.apply(frame)
        bg_img = bs.getBackgroundImage()
        video.write(bg_img)
    
    video.release()
    cap.release()

