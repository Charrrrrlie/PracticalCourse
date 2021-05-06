import cv2
import os
import numpy as np
import sys
import skimage
from skimage.measure import label 
from scipy.ndimage.filters import gaussian_filter

rt = '../../PreData/Origin-Test/'
wrt_fgmask = './data/mask_diff/' 

if not os.path.exists(wrt_fgmask):
    os.makedirs(wrt_fgmask)

video_start = sys.argv[1]
video_end = sys.argv[2]
for j in range(int(video_start),int(video_end)+1):
    count = 0
    videos=str(j)+'.mp4'
    print (videos)
    out = 0 

    #read video
    cap = cv2.VideoCapture(os.path.join(rt, videos))
    ret, frame = cap.read()
    print(ret)
    while ret:
        if count % 10 == 0:
            last_frame = frame
        count += 1
        cap.set(cv2.CAP_PROP_POS_MSEC, 0.2 * 1000 * count)
        ret, frame = cap.read()
        
        if not ret:
            break
        
        fg = cv2.subtract(frame,last_frame)
        fg = cv2.cvtColor(fg, cv2.COLOR_BGR2GRAY)
        _, fg1 = cv2.threshold(fg, 100, 255, cv2.THRESH_BINARY)
        fg1[fg1==255] = 1
        
        if sum(sum(fg1)) > 13000:
            continue
        
        out = cv2.bitwise_or(out,fg) #||
        
        out = cv2.medianBlur(out, 3)

        if count % 6 == 0:
            out = cv2.GaussianBlur(out, (3, 3), 0)
        
        _, out = cv2.threshold(out, 99, 255, cv2.THRESH_BINARY)

    out = cv2.GaussianBlur(out, (5, 5), 0)

    cap.release()
    min_area = 6000
    mask = label(out, connectivity = 1)
    num = np.max(mask)
    for i in range(1,int(num+1)):
        if np.sum(mask==i)<min_area:
            mask[mask==i]=0     
    mask = mask>0
    mask = mask.astype(float)
    print(os.path.join(wrt_fgmask, str(int(videos.split('.')[0])).zfill(3) + '.jpg'))
    cv2.imwrite(os.path.join(wrt_fgmask, str(int(videos.split('.')[0])).zfill(3) + '.jpg'), mask*255)


