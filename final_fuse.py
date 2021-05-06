#update pixel&box timeback results

import numpy as np
import os
import sys

def load_deepsort(path):
    deep_res=[]
    num_count = []
    for video_name in range(1, 151):
        video_name = int(video_name)

        try:
            result_txt = path+"%d_time_back_box.txt" % video_name

            f_box = open(result_txt, 'r')
            video_result = eval(f_box.read())

            for key in video_result:
                start_time = float(video_result[key][1])
                x=float(video_result[key][0][1])
                y=float(video_result[key][0][2])
                w=float(video_result[key][0][3])
                h=float(video_result[key][0][4])

                deep_res.append([video_name, start_time, x,y,w,h,video_result[key][3]])
            num_count.append(video_name)
        except:
            pass
    f_box.close()

    return deep_res,num_count

output='./final_fuse_4_9.txt'

deepsort_path1="./baidu_test/pixel_track/post_process/deepsort_back_test_psnr_18.5_0/"
deepsort_path2="./baidu_test/pixel_track/post_process/deepsort_back_test_ssim_0.5_0.3/"

pixel_path='pixel_res2.txt'

f_final=open(output,"w")

final = []


# load deepsort tracking branch
deep_psnr,_=load_deepsort(deepsort_path1)
deep_ssim,num_count=load_deepsort(deepsort_path2)

weight=1.5
ratio=0.5
print(deep_psnr)
print("-------")
print(deep_ssim)
for i in range(len(deep_psnr)):
    psnr_time=deep_psnr[i][1]
    ssim_time=deep_ssim[i][1]
    video_num=deep_psnr[i][0]
    x=deep_psnr[i][2]
    y=deep_psnr[i][3]
    w=deep_psnr[i][4]
    h=deep_psnr[i][5]

    if deep_psnr[i][6]==0 and deep_ssim[i][6]==0:
        if min(psnr_time,ssim_time)<3:
            final.append([video_num,0,x,y,w,h,1])
        else:
            final.append([video_num,min(psnr_time, ssim_time),x,y,w,h,1])
        continue

    if psnr_time<=ssim_time:
        final.append([video_num,psnr_time,x,y,w,h,1])
    else:
        if psnr_time-ssim_time<2 and psnr_time-ssim_time>1 :
            final.append([video_num,ratio*(psnr_time+weight-ratio)+(1-ratio)*ssim_time,
                          x,y,w,h,1])

        elif psnr_time-ssim_time>30:
            final.append([video_num,ssim_time,x,y,w,h,1])
        elif psnr_time-ssim_time<1:
            final.append([video_num,psnr_time-weight,x,y,w,h,1])
        else:
            final.append([video_num,psnr_time+weight,x,y,w,h,1])

#print(final)


#load origin-pixel tracking branch
f_pix=open(pixel_path,"r")

pix_res=f_pix.readlines()
for r in pix_res:
    re=r.split(' ')
    if int(re[0]) in num_count:
        continue
    else:
        final.append([re[0],re[1],re[2],re[3],re[4],re[5],1])

f_pix.close()


#sort and output
sort_final = sorted(final, key=lambda final: float(final[0]), reverse=False)

for ii in range(len(sort_final)):
    f_final.write(str(sort_final[ii][0]) + " " + str(sort_final[ii][1]) + " " +
                  str(sort_final[ii][2]) + " " + str(sort_final[ii][3]) + " " + str(sort_final[ii][4]) + " "+str(sort_final[ii][5]) + " "+
                  str(sort_final[ii][6]) + " \n")

np.save("final_fuse_4_9.npy",sort_final)

##修改输出
import joblib

save_pickle={}
for i in sort_final:
    save_pickle[int(i[0])]=[float(i[1]),[float(i[2]),float(i[3]),float(i[2])+float(i[4]),float(i[3])+float(i[5])]]

joblib.dump(save_pickle,"final_fuse_4_9.pkl")


f_final.close()