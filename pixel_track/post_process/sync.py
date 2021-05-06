import numpy as np


output='./pixel_res2.txt'
output2='./pixel_res2.npy'

f=open(output,"w")
# load the results of pixel_level tracking branch

def load_data(path):
    res={}
    for video_name in range(1, 151):

        try:
            result_txt = path+"%d_time_back_box.txt" % video_name

            f_pre = open(result_txt, 'r')
            video_result = eval(f_pre.read())

            temp_end = -60
            temp_start = -120

            for key in video_result:
                start_time = float(video_result[key][1])
                end_time = float(video_result[key][2])
                score = float(video_result[key][3])
                bbox=video_result[key][0]

                if start_time < temp_end + 60:  # thred 1
                    continue
                if start_time < temp_start + 120:  # thred 2
                    continue

                temp_start = start_time
                temp_end = end_time

                res[video_name]=[start_time,bbox[1],bbox[2],bbox[3],bbox[4],score]

        except:
            pass

    return res

path_rate='./pixel_track/post_process/time_back_test_rate_0.7_0.65/'
path_ssim='./pixel_track/post_process/time_back_test_ssim_0.4_0.3/'
path_psnr='./pixel_track/post_process/time_back_test_psnr_13_10/'

#multi res
rate_res =load_data(path_rate)
psnr_res=load_data(path_psnr)
ssim_res=load_data(path_ssim)

#fuse
res=[]
for key in rate_res:
    rate_time=rate_res[key][0]
    psnr_time=psnr_res[key][0]
    ssim_time=ssim_res[key][0]

    res_time=0

    d1=abs(rate_time-psnr_time)
    d2=abs(rate_time-ssim_time)
    d3=abs(psnr_time-ssim_time)

    if rate_time>=psnr_time and ssim_time>=psnr_time:

        if max(d1,d3)<12 and min(d1,d3)>4:
            res_time=min(rate_time,ssim_time)
            res.append([key, res_time, rate_res[key][1], rate_res[key][2], rate_res[key][3],
                        rate_res[key][4], rate_res[key][5]])
            print('type1.1 :{0}'.format(key))
            continue
        else:
            res_time=psnr_time
            res.append([key,res_time,rate_res[key][1],rate_res[key][2],rate_res[key][3],
                        rate_res[key][4],rate_res[key][5]])
            print('type1.2 :{0}'.format(key))
            continue

    else:
        if max(d1,d3)<6.2:
            res_time=(min(rate_time,ssim_time)+psnr_time)/2
            res.append([key, res_time, rate_res[key][1], rate_res[key][2], rate_res[key][3],
                        rate_res[key][4], rate_res[key][5]])
            print('type2 :{0}'.format(key))
            continue
        else:
            res_time = (max(rate_time, ssim_time) + psnr_time) / 2
            res.append([key, res_time, rate_res[key][1], rate_res[key][2], rate_res[key][3],
                        rate_res[key][4], rate_res[key][5]])
            print('type3 :{0}'.format(key))
            continue

#output
for ii in range(len(res)):
    f.write(str(res[ii][0])+" "+str(res[ii][1])+" "
            +str(res[ii][2])+" "+str(res[ii][3])+" "+str(res[ii][4])+" "+str(res[ii][5])+" "
            +str(res[ii][6])+"\n")

np.save(output2,res,allow_pickle=True)

