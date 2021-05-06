#3.20 初步筛选掉可疑结果 再进行timeback
import numpy as np
import os
import sys

merge_threshold=100

outpath='./coarse_filter/'

if not os.path.exists(outpath):
    os.makedirs(outpath)

#range here

for video_name in range(1,151):
    video_name = int(video_name)

    # load the results of pixel_level tracking branch
    pixel_coarse = {}
    try:
        result_txt = "./pixel_track/post_process/similar_txt/%d_box.txt" % video_name

        f_pre = open(result_txt, 'r')
        video_result = eval(f_pre.read())
        prediction = []
        for key in video_result:
            start_time = float(video_result[key][1])
            end_time = float(video_result[key][2])
            x1, y1, x2, y2 = float(video_result[key][0][1]), float(video_result[key][0][2]), float(video_result[key][0][3]), \
                             float(video_result[key][0][4])

            start_des = [x1 ,y1, x2-x1, y2-y1]
            prediction.append([start_time, end_time, video_result[key][3], start_des])

        sort_predicton = sorted(prediction, key=lambda prediction: float(prediction[0]), reverse=False)

        count=1   #merge count
        time=0    #circle count
        pass_flag=False
        if len(sort_predicton) > 1:
            for i in range(len(sort_predicton)):
                start = float(sort_predicton[i][0])
                end = float(sort_predicton[i][1])
                score = float(sort_predicton[i][2])
                start_detres = sort_predicton[i][3]

                if i >= 1:
                    if i==1 or pass_flag:
                        #last-time jugdement
                        if end-start < 120 and ((start > 50 and start < 640) and end < 860):
                            pass_flag=True
                            continue
                    #shape judgement
                    if start_detres[2]<=2 or start_detres[3]<=2:
                        continue

                    if sort_predicton[i][0] - sort_predicton[i - 1][0] < merge_threshold and time!=0:

                        #fuse judge
                        if end-start>3*(sort_predicton[i][1]-sort_predicton[i][0]):
                            pixel_coarse[time] = [start_detres, start, end, score]
                            continue

                        sort_predicton[i][0]=sort_predicton[i-1][0]
                        sort_predicton[i][1] = max(sort_predicton[i-1][1], sort_predicton[i][1])

                        #last update
                        if(i==len(sort_predicton)-1):
                            pixel_coarse[time-1][2]=sort_predicton[i][1]
                        count += 1
                        score = (score + float(sort_predicton[i][2])) / count
                    else:
                        count=1
                        start_detres.insert(0, time)
                        pixel_coarse[time] = [start_detres, start, end, score]
                        time+=1

                if i == 0:      #multi-anomalies  first
                    if start < 20:
                        continue
                    # last-time jugdement
                    if end - start < 120 and ((start > 50 and start < 640) and end < 860):
                        continue
                    start_detres.insert(0, 0)
                    pixel_coarse[time] = [start_detres, start, end, score]
                    time+=1

        elif len(sort_predicton) == 1:

            start = float(sort_predicton[0][0])
            end = float(sort_predicton[0][1])
            score = float(sort_predicton[0][2])
            start_detres = sort_predicton[0][3]

            # last-time jugdement
            if end - start < 120 and ((start > 50 and start < 640) and end < 860):
                continue
            # shape judgement
            if start_detres[2] <= 2 or start_detres[3] <= 2:
                continue

            start_detres.insert(0, 0)
            pixel_coarse[0]=[start_detres,start, end, score]

        if pixel_coarse!={}:
            f_writer=open(outpath+str(video_name)+'_coarse_box.txt',"w")
            f_writer.write(str(pixel_coarse))
            f_writer.close()

    except:
        pass

