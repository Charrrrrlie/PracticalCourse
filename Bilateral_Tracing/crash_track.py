import numpy as np
import os
import pickle
import cv2

from sklearn.cluster import MeanShift
from matplotlib import pyplot as plt

#anomaly list

# (AT * A)^(-1) * AT * y
def cal_slope(center_list):

    x_list=[]
    y_list=[]

    for i in range(len(center_list)):
        x_list.append([center_list[i][0],1])
        y_list.append(center_list[i][1])

    x_inv=np.linalg.pinv(np.dot(np.transpose(x_list),x_list))
    slope=np.dot(np.dot(x_inv,np.transpose(x_list)),y_list)

    delta=np.dot(x_list,slope)-y_list

    return slope[0],np.dot(np.transpose(delta),delta)

def cal_loc(center1,center2):
    return ((center1[0]-center2[0])**2+(center1[1]-center2[1])**2)**0.5

def backward_search(sharp_change,idx):

    for i in range(idx,2,-1):
        if sharp_change[i]<=sharp_change[i-1] and sharp_change[i]<=sharp_change[i-2]:
            return i+1
    # return to the reverse end
    return 25

def forward_search(sharp_change,idx):

    for i in range(idx+2, 24):
        if sharp_change[i] <= sharp_change[i+1] and sharp_change[i] <= sharp_change[i+2]:
            return i
    return -1

def count_plat(sharp_change,second_num,third_num):
    count=0
    count2=0
    for i in sharp_change:
        if i==second_num:
            count+=1
        if i==third_num:
            count2+=1

    return max(count,count2)

#analyze in statistics
def filter(list,video_num):

    sharp_change=np.array(list)

    peak=np.max(sharp_change)
    avg=np.average(sharp_change)
    sort=-np.sort(-sharp_change)   #DESC

    times=count_plat(sharp_change,sort[1],sort[2])

    if sort[0]!=sort[1]:
        peak_idx=np.argmax(sharp_change)
            #has peak?
        if sort[0]-sort[1]>=2 and \
                (peak-sharp_change[max(0,peak_idx-1)]>=2 or peak-sharp_change[min(peak_idx+1,25)]>=2):
            if peak<12:
                if times>=5:
                    print("Video {0}. Missed.".format(video_num))
                    return -1
                print("Video {0} Has Peak. Result {1} in Peak Search.".format(video_num,peak_idx))
                return peak_idx
            else:
                #find the point starts to increase
                print("Video {0} Has Peak. Result {1} in Backward Search.".format(video_num, backward_search(sharp_change,peak_idx)))
                return backward_search(sharp_change,peak_idx)
        #low values
        elif avg>4:
            print("Video {0} Hasn't Peak. Result {1} in Forward Search.".format(video_num, forward_search(sharp_change,peak_idx)))
            return forward_search(sharp_change,peak_idx)

        elif sort[0]>=4 and (sort[1]!=sort[2] or sort[1]>=4):
            if times >= 5 and times<=8:
                print("Video {0}. Missed1.".format(video_num))
                return -1
            for i in range(len(sharp_change)):
                if sharp_change[i]>=4:
                    print("Video {0} . Result {1} in Max Search.".format(video_num, i))
                    return i
        elif sort[0]>=3 and sort[1]!=sort[2]:
            count=0
            for i in range(len(sharp_change)):
                if sharp_change[i]>=3:
                    if count:
                        print("Video {0} . Result {1} in Second Max Search.".format(video_num, i))
                        return i+1
                    count += 1
        #platform
        if times > 20:
            return 1

    #reduce platform shape
    elif sort[1]!=sort[2]:
        #contains same max values
        if sort[0] >= 4:
            for i in range(len(sharp_change)):
                if sharp_change[i] >= 4:
                    print("Video {0} . Result {1} in Max Search.".format(video_num, i))
                    return i
        elif sort[0] >= 3:
            count = 0
            for i in range(len(sharp_change)):
                if sharp_change[i] >= 3:
                    if count:
                        print("Video {0} . Result {1} in Second Max Search.".format(video_num, i))
                        return i
                    count += 1
        print("Video {0}. Same Max.".format(video_num))
    print("Video {0}. Missed2.".format(video_num))
    return -1

def suspect(path,file):

    suspect_num = []
    suspect_track=[]

    plot_count=[]
    #
    for num in range(0,26):

        f=open(path+list[num],"rb")
        traj={}
        traj=pickle.load(f)
        f.close()

        #in one second
        track_slope={}

        similar=[]
        similar_count=[]
        plot_num=0
        for ii in traj:

            if len(traj[ii])<10:
                continue
            if cal_loc(traj[ii][0],traj[ii][-1])<8:
                continue

            track_slope[ii]=cal_slope(traj[ii])

            #Error
            if track_slope[ii][1]>30:
                similar.append(track_slope[ii][0])
                similar_count.append([2,ii])
                plot_num+=1
                continue

        plot_count.append(plot_num)

    return filter(plot_count,int(file.split(".mp4")[0]))


def offtrack(path,mask_path,file):
    mask=cv2.imread(mask_path+(file.split(".mp4")[0]).zfill(3)+".jpg",0)
    mask[mask<170] = 0
    mask[mask>=170] = 1

    for num in range(0,26):

        f=open(path+list[num],"rb")
        traj={}
        traj=pickle.load(f)
        f.close()

        for ii in traj:
            count=0

            for loc in traj[ii]:
                x=max(0,loc[0]-5)
                x2=min(800,loc[0]+5)
                y=max(0,loc[1]-5)
                y2=min(410,loc[1]+5)
                if sum(sum(mask[y:y2,x:x2]))<40 and cal_loc(traj[ii][0],traj[ii][-1])>10:
                    count+=1

            if count>8 and traj[ii][0][0]>300 and traj[ii][0][0]<600 \
                    and traj[ii][0][1]>200 and traj[ii][0][1]<600:
                print("Video {n} Num:{0} ID:{1} Box:{x}".format(
                        num,ii,x=traj[ii],n=file.split(".mp4")[0]
                    ))
                return num

    return -1

input_path='./Trajectory/'
pixel_path='../final_fuse_4_9.pkl'
mask_path='../baidu_test/intermediate_result/mask/mask-fuse/mask_'
sub_path='/traj/'

output_path='./curve_change.pkl'
unprocess_path='./unprocess2.npy'

#
files=os.listdir(input_path)

list=['0_30.pkl','30_60.pkl','60_90.pkl','90_120.pkl','120_150.pkl','150_180.pkl','180_210.pkl','210_240.pkl','240_270.pkl',
      '270_300.pkl','300_330.pkl','330_360.pkl','360_390.pkl','390_420.pkl','420_450.pkl','450_480.pkl','480_510.pkl','510_540.pkl',
      '540_570.pkl','570_600.pkl','600_630.pkl','630_660.pkl','660_690.pkl','690_720.pkl','720_750.pkl','750_779.pkl']

#read undo list
f_pixel=open(pixel_path,"rb")
pixel_res=pickle.load(f_pixel)
f_pixel.close()

undo_list1=[]
undo_list2=[]
for key in pixel_res:
    #time
    if pixel_res[key][0]<40:
        undo_list1.append(key)
    elif pixel_res[key][0]>620 and pixel_res[key][0]<700:
        undo_list2.append(key)

#search in [20,-6] before/after the anomaly

processed=[]
unprocessed=[]
res={}

for file in files:

    path=input_path+file+sub_path
    video_num=int(file.split(".mp4")[0])

    if video_num in undo_list1:
        continue

    off_num=offtrack(path,mask_path,file)
    if off_num!=-1:
        processed.append(video_num)
        res[video_num]=20-off_num
        continue

    if video_num in undo_list2:
        unprocessed.append(video_num)
        continue

    suspect_num=suspect(path,file)
    if suspect_num!=-1:
        processed.append(video_num)
        res[video_num]=20-suspect_num
    else:
        unprocessed.append(video_num)

print("Undo list1 (<40) :{x}".format(x=undo_list1))
print("Undo list2 (>620) :{x}".format(x=undo_list2))
print("Processed_Num:{x}".format(x=processed))
print("Result Num:{x}".format(x=res))
print("UnProcessed_Num:{x}".format(x=unprocessed))

#output
f_out=open(output_path,"wb")
pickle.dump(res,f_out)
f_out.close()

np.save(unprocess_path,unprocessed)