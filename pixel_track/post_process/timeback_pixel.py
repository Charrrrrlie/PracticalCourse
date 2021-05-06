import math
import os
import sys
import cv2
import numpy as np
from PIL import Image, ImageStat
from skimage.metrics import structural_similarity


def compute_iou2(rec1, rec2):
    areas1 = (rec1[3] - rec1[1]) * (rec1[2] - rec1[0])
    areas2 = (rec2[3] - rec2[1]) * (rec2[2] - rec2[0])
    left = max(rec1[1], rec2[1])
    right = min(rec1[3], rec2[3])
    top = max(rec1[0], rec2[0])
    bottom = min(rec1[2], rec2[2])
    w = max(0, right - left)
    h = max(0, bottom - top)
    return float(w * h) / (areas2 + areas1 - w * h)


def brightness(im_file, x1, y1, x2, y2):
    im = Image.open(im_file)
    box = (x1, y1, x2, y2)
    im = im.crop(box)
    stat = ImageStat.Stat(im)
    r, g, b = stat.mean
    return math.sqrt(0.241 * (r ** 2) + 0.691 * (g ** 2) + 0.068 * (b ** 2))


def euclidean(img1, img2):
    img = cv2.subtract(img1, img2)
    mid_img = cv2.medianBlur(img, 3)
    _, mid_img = cv2.threshold(mid_img, 50, 1, cv2.THRESH_BINARY)
    return 1 - sum(sum(sum(mid_img))) / (img.shape[0] * img.shape[1])


def ssim(img1, img2):
    gray1 = cv2.cvtColor(img1, cv2.COLOR_RGB2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)

    if gray2.shape[0] <= 7 or gray2.shape[1] <= 7:
        score = structural_similarity(gray1, gray2, win_size=3)
    else:
        score = structural_similarity(gray1, gray2)
    return score


def psnr1(img1, img2):
    img1 = cv2.resize(img1, (20, 20))
    img2 = cv2.resize(img2, (20, 20))
    mse = np.mean((img1 / 1.0 - img2 / 1.0) ** 2)
    if mse < 1.0e-10:
        return 100
    return 10 * math.log10(255.0 ** 2 / mse)


# def change_rate(img1, img2, threshold):
#     dimg = abs(img1-img2)
#     return sum(sum(dimg>threshold))/img1.shape[0]/img1.shape[1]

def timeback(start_time, img_root, x1, y1, x2, y2, thred1, thred2, distance):
    for t in range(int(float(start_time) * 30), max(5, int(float(start_time - 34) * 30)), -5):
        dis_list = []
        dis_ = []
        comprare_img = cv2.imread(img_root + str(int(t)).zfill(5) + ".jpg")
        comprare_img_crop = comprare_img[y1:y2, x1:x2]
        dis_list.append(distance(comprare_img_crop, template_crop))
        dis_.append(distance(comprare_img_crop, template_crop) < thred1)
        comprare_img = cv2.imread(img_root + str(int(t - 1)).zfill(5) + ".jpg")
        comprare_img_crop = comprare_img[y1:y2, x1:x2]
        dis_list.append(distance(comprare_img_crop, template_crop))
        dis_.append(distance(comprare_img_crop, template_crop) < thred1)
        comprare_img = cv2.imread(img_root + str(int(t - 2)).zfill(5) + ".jpg")
        comprare_img_crop = comprare_img[y1:y2, x1:x2]
        dis_list.append(distance(comprare_img_crop, template_crop))
        dis_.append(distance(comprare_img_crop, template_crop) < thred1)
        comprare_img = cv2.imread(img_root + str(int(t - 3)).zfill(5) + ".jpg")
        comprare_img_crop = comprare_img[y1:y2, x1:x2]
        dis_list.append(distance(comprare_img_crop, template_crop))
        dis_.append(distance(comprare_img_crop, template_crop) < thred1)
        comprare_img = cv2.imread(img_root + str(int(t - 4)).zfill(5) + ".jpg")
        comprare_img_crop = comprare_img[y1:y2, x1:x2]
        dis_list.append(distance(comprare_img_crop, template_crop))
        dis_.append(distance(comprare_img_crop, template_crop) < thred1)
        # print(np.sum(psnr1_))
        # print(psnr1_list)
        # if np.mean(psnr1_list) < 21:
        img_file = img_root + str(int(t)).zfill(5) + ".jpg"
        bt = brightness(img_file, x1, y1, x2, y2)
        bt_all = brightness(img_file, 0, 0, 799, 409)
        # print(bt-bt_all)
        # print(np.sum(psnr1_))
        # print(np.mean(psnr1_list))

        if (bt - bt_all) < 20 and  np.sum(dis_) > 4:
            print("jump from judge1 |Bright:{0} Num:{1} Score:{x}".format(bt - bt_all, np.sum(dis_),x=dis_list))
            return str(t / 30)
        if (bt - bt_all) >= 20 and float(np.mean(dis_list)) <= thred2:
            print("jump from judge2 |Bright:{0} Score:{1}".format(bt - bt_all, np.mean(dis_list)))
            return str(t / 30)

    return max(0, start_time - 34)

type_num=int(sys.argv[1])
video_name_start = int(sys.argv[2])
video_name_end = int(sys.argv[3])

psnr_thred = 18
psnr_avg_thred = 0

euclidean_thred = 0.4
euclidean_avg_thred = 0.45

ssim_thred = 0.3
ssim_avg_thred = 0.2

type_str = ['psnr', 'ssim', 'rate']
thred = [[psnr_thred, psnr_avg_thred], [ssim_thred, ssim_avg_thred], [euclidean_thred, euclidean_avg_thred]]
func = [psnr1, ssim, euclidean]

output = "./deepsort_back_test_" + type_str[type_num] + "_" + str(thred[type_num][0]) + "_" + str(thred[type_num][1]) + '/'

if not os.path.exists(output):
    os.makedirs(output)

for video_name in range(video_name_start, video_name_end + 1):
    # if True:
    # if True:
    try:
        line = open("../../../need2timeback/" + str(video_name) + "_deepsort_box.txt", "r")

        # print(video_name)
        video_result = eval(line.read())
        # print(video_result)
        for key in video_result:
            start_time = float(video_result[key][1])

            if start_time < 5:
                video_result[key][1] = 0
                continue
            x1, y1, w, h = int(video_result[key][0][1]), int(video_result[key][0][2]), int(video_result[key][0][3]), \
                           int(video_result[key][0][4])
            x2 = x1 + w
            y2 = y1 + h
            img_root = "../../../PreData/Origin-Frame/" + str(video_name) + "/" + str(
                video_name) + '_'
            # print(img_root)
            template = cv2.imread(img_root + str(int(float(start_time) * 30) + 1).zfill(5) + ".jpg")
            template_crop = template[y1:y2, x1:x2]
            # parameters
            time = timeback(start_time, img_root, x1, y1, x2, y2
                            , thred[type_num][0], thred[type_num][1], func[type_num])

            video_result[key][1] = time

        f_out = open(output + str(video_name) + '_time_back_box.txt', 'w')
        print(video_name)
        print(video_result)
        line.close()
        f_out.write(str(video_result))
        f_out.close()

    except:
        pass




