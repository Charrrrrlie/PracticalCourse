
🥉News: our team got the 3rd place in the AICity 2021 Challenge on Track 4

## Dual-Modality Vehicle Anomaly Detection via Bilateral Trajectory Tracing (CVPRW 2021)


### Introduction
This is the source code for Team WHU_IIP for track 4 Anomaly Detection in AICity 2021 Chellenge.

Our experiments conducted on the Track 4 testset yielded a result of 0.9302 F1-Score and 3.4039 root mean square error (RMSE), which performed 3rd place in the challenge.

!{(https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/rank.jpg)}

More implementation details are displayed in the paper—— 
*Dual-Modality Vehicle Anomaly Detection via Bilateral Trajectory Tracing*. 

Here we only show the flow chart for better understanding of the following procedures.  

![Flow Chart]{(https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/abstract.jpg)}
#### NVIDIA AICity Challenge 2021 Track4

### Requirements

- Linux (tested on Ubuntu 16.04.5)
- Python 3.7
- PyTorch 
- OpenCV
- Scikit-Image

### Annotations
TBD for CJY

### Procedures

#### Background Modeling
##### Extract the background
``` 
cd bg_code
python ex_bg_mog.py
```

#### Preparation For Detection 
##### Structure of *PreData* Folder
The orginal videos and the their frames are put under `../PreData/Origin-Test` and `../PreData/Origin-Frame` folders, respectively. And the background modeling results are put under `../PreData/Forward-Bg-Frame` folder.

All these files are organized for the Detect Step later and then the detection results based on background modeling will be saved under  `../PreData/Bg-Detect-Result/Forward_full` for each videos while `../PreData/Bg-Detect-Result/Forward` is saved in frames seperated from full videos.

Detailed structure is shown as below.
``` 
├── Bg-Detect-Result
│   ├── Forward
│       └── 1
│          ├──test_1_00000.jpg.npy
│          ├──test_1_00001.jpg.npy
│          ├──test_1_00002.jpg.npy
│          └── ...
│       ├── 2
│       ├── 3
│       └── ...
│   └── Forward_full
│       ├── 1.npy
│       ├── 2.npy
│       ├── 3.npy
│       └── ...
├── Forward-Bg-Frame
│   ├── 1.mp4
│   ├── 2.mp4
│   ├── 3.mp4
│   └── ...
├── Origin-Frame
│   └── 1
│       ├──1_00001.jpg
│       ├──1_00002.jpg
│       ├──1_00003.jpg
│       └── ...
│   ├── 2
│   ├── 3
│   └── ...
├── Origin-Test
│   ├── 1.mp4
│   ├── 2.mp4
│   ├── 3.mp4
│   └── ...
```

#### Detection 
TBD for CJY

#### Road Mask Construction
##### Extract Motion-Based Mask
``` 
cd mask_code
python mask_frame_diff.py start_num end_num
```
##### Extract Trajectory-Based Mask
``` 
python mask_track.py video_num
```
##### Mask Fusion
``` 
python mask_fuse.py video_num
```

#### Pixel-Level Tracking
##### Coarse Detect
``` 
cd pixel_track/coarse_ddet
python pixel-level_tracking.py start_num end_num
```
##### Fuse Similar Results
``` 
cd pixel_track/post_process
python similar.py start_num end_num
```
##### Filter Suspicious Anomaly Results
``` 
python filter.py
```
##### Fuse Close Results
```
python pixel_fuse.py
```
##### ROI Backtracking for Pixel-Level
```
python timeback_pixel.py type_num start_num end_num
```
##### Fuse Tracking Results
```
python sync.py
```

#### Box-Level Tracking
TBD for CJY
##### 
##### ROI Backtracking for Box-Level

#### Dynamic Analysis Stage
We mainly contribute this to trace back the exact time of crashing while what's done before can only be used to locate the time when abnormal vehicles become static.

##### Multiple Vehicle Trajectory Tracing
```
cd car_crash
python crash_track.py
```
##### Singular Vehicle Trajectory Tracing
TBD for CJY


### Demo
#### Multiple Vehicle Trajectory Tracing
Statistically, vehicle crashes often come up with sharp turns, which is the primary reaction of drivers when encountering such anomalies. Here we list some typical scenarios to display that.

!{(https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/multi.jpg)}

#### Singular Vehicle Trajectory Tracing
TBD for CJY

<div align=center><img width="400" height="205" src="https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/0.gif"/></div>
<div align=center><img width="400" height="205" src="https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/1.gif"/></div>
<div align=center><img width="400" height="205" src="https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/2.gif"/></div>
<div align=center><img width="400" height="205" src="https://github.com/JingyuanChen1423/AICity_2021_Anomaly_Detection/figs/3.gif"/></div>



## 
If you have any questions or issues in using this code, please feel free to
contact us. (jchen157@u.rochester.edu mainly for Detection & Singular Tracing and yuchen_yang@whu.edu.cn for the last)
