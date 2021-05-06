import numpy as np
import os
import sys

output='./pixel_ori.txt'
pixel_path='./coarse_filter/'

#load origin-pixel tracking

f_res=open(output,"w")
res=[]
for video_name in range(1, 151):
    video_name = int(video_name)

    try:
        result_txt = pixel_path+"%d_coarse_box.txt" % video_name

        f_pix = open(result_txt, 'r')
        video_result = eval(f_pix.read())

        for key in video_result:
            start_time = float(video_result[key][1])

            res.append([video_name, start_time])
            f_res.write(str(video_name)+" "+str(start_time)+"\n")
            break
        f_pix.close()

    except:
        pass

f_res.close()
