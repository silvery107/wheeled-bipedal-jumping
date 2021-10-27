import cv2
import os
import numpy as np
import matplotlib.pyplot as plt

img_head = None
img_seq = []
for root,dirs,files in os.walk("screenshot"):
    # print(root)
    for file in files:
        # print(file)
        if file[-3:]=="jpg":
            continue
        img = cv2.imread(os.path.join(root,file))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = np.array(img, dtype=np.float32)
        if img_head is None:
            img_head = img
        else:
            img_seq.append(img)
            
img_head_gray = cv2.cvtColor(img_head,cv2.COLOR_RGB2GRAY).astype('uint8')
# plt.imshow(img_head.astype('uint8'))
# plt.show()

backSub = cv2.createBackgroundSubtractorMOG2()
backSub.apply(img_head)
kernel = cv2.getGaussianKernel(3,1)
fgmasks = []
fgmask_inv = []
for img in img_seq:
    fgMask = backSub.apply(img)
    fgMask = cv2.morphologyEx(fgMask,cv2.MORPH_CLOSE,kernel,iterations=10)
    _,fgMask = cv2.threshold(fgMask,125,255,cv2.THRESH_BINARY)
    fg_inv = cv2.bitwise_not(fgMask)
    fgmasks.append(fgMask)
    fgmask_inv.append(fg_inv)
    cv2.imshow("Mask", fgMask[::-1])
    cv2.waitKey(10)
    # plt.imshow(fgMask, cmap='gray')
    # plt.show()

img_composite = img_head
for frame, fg, fg_inv in zip(img_seq, fgmasks, fgmask_inv):
    img_composite = cv2.bitwise_and(img_composite, img_composite, mask=fg_inv)
    img_composite += cv2.bitwise_and(frame, frame, mask=fg)

img_composite_8u = img_composite.astype(np.uint8)
plt.imshow(img_composite_8u)
plt.show()