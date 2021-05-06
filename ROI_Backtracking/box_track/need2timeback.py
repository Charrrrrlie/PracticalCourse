#compare deepsort_res with pixel_res

import numpy as np
import os

pixel_path='../../pre/pixel_res.npy'
deepsort_path='deepsort_with_pixel.npy'

output='./need2timeback/'

if not os.path.exists(output):
    os.makedirs(output)

pixel_res={}

temp_pixel=np.load(pixel_path,allow_pickle=True)
temp_deepsort=np.load(deepsort_path,allow_pickle=True)

for i in range(len(temp_pixel)):
    pixel_res[int(temp_pixel[i][0])]=[[int(temp_pixel[i][0]),
                    temp_pixel[i][2],temp_pixel[i][3],temp_pixel[i][4],temp_pixel[i][5]],
                    temp_pixel[i][1],891.9,1]

for i in range(len(temp_deepsort)):
    deepsort_res = {}
    x = temp_deepsort[i][2]
    y = temp_deepsort[i][3]
    w = temp_deepsort[i][4]-x
    h = temp_deepsort[i][5]-y

    #deepsort finds new anomalies
    if not pixel_res.get(int(temp_deepsort[i][0]),0):

        deepsort_res[int(temp_deepsort[i][0])]=[[int(temp_deepsort[i][0]),x,y,w,h],
                     temp_deepsort[i][1],891.9,1]
        f=open(output+str(int(temp_deepsort[i][0]))+"_deepsort_box.txt","w")
        f.write(str(deepsort_res))
    #compare
    else:
        #considering deepsort to correct the results of pixel-level
        deepsort_time=temp_deepsort[i][1]
        pixel_time=pixel_res[int(temp_deepsort[i][0])][1]
        if deepsort_time>10 and (deepsort_time-80> pixel_time or
            (deepsort_time+80> pixel_time and deepsort_time< pixel_time)):

            deepsort_res[int(temp_deepsort[i][0])] = [[int(temp_deepsort[i][0]),x,y,w,h],
                    temp_deepsort[i][1], 891.9, 1]

            f = open(output + str(int(temp_deepsort[i][0])) + "_deepsort_box.txt","w")
            f.write(str(deepsort_res))