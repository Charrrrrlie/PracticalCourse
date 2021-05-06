import pickle


final_path='./track4_4_3.txt'

fuse_path='./final_fuse.pkl'
curve_path='./car_crash/curve_change.pkl'
of_path='./car_crash/anomaly_by_of.pkl'

f_fuse=open(fuse_path,"rb")
fuse_res=pickle.load(f_fuse)
f_fuse.close()

f_curve=open(curve_path,"rb")
curve_res=pickle.load(f_curve)
f_curve.close()

f_of=open(of_path,"rb")
of_res=pickle.load(f_of)
f_of.close()

f_output=open(final_path,"w")

for key in fuse_res:
    if curve_res.get(key,0):
        fuse_res[key][0]-=curve_res[key]          #res in seconds
        print("video id {0} changed {1} s in curve detect".format(key,curve_res[key]))
    if of_res.get(key,0):
        fuse_res[key][0]-=float(of_res[key])/30   #res in frames
        print("video id {0} changed {1:.2f} s in optical flow detect".format(key,of_res[key]/30))

    f_output.write(str(key)+" "+str(fuse_res[key][0])+" "+str(1)+"\n")

f_output.close()

