#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/ubuntu/237D/camera/cam_env/lib/python3.12/site-packages')

import cv2
import time
from ultralytics import YOLO


class YOLODetector():
    def __init__(self, model_name="yolo26n.pt"):
        """ Models:
Model	Size (px)	mAP@0.5:0.95   mAP@0.5:0.95 (e2e)	Speed (ONNX)   Speed (TensorRT)    Params (M)	FLOPs (B)	
YOLO26n	  640	        40.9	         40.1	          38.9 ± 0.7	   1.7 ± 0.0	     2.4	       5.4
YOLO26s	  640	        48.6	         47.8	          87.2 ± 0.9	   2.5 ± 0.0	     9.5	       20.7
YOLO26m	  640	        53.1	         52.5	          220.0 ± 1.4	   4.7 ± 0.1	     20.4	       68.2
YOLO26l	  640	        55.0	         54.4	          286.2 ± 2.0	   6.2 ± 0.2	     24.8	       86.4
YOLO26x	  640	        57.5	         56.9	          525.8 ± 4.0	   11.8 ± 0.2	     55.7	       193.9
        """
        self.model_name = model_name
        self.model = YOLO(self.model_name)
        self.init_done = False
        self.lock = False

    def detect(self, image_in, show=False):
        """Args:
            image_in: BGR image in
        """
        # lock until initialization is done
        # otherwise it will crash with segmentation fault.
        while self.lock == True:
            # print('waiting')
            time.sleep(0.01)
        if self.init_done == False:
            print("model initializing ...")
            self.lock = True

        # yolov8 takes BGR images directly
        outputs = self.model.predict(source=image_in, show=show)
        
        if self.init_done == False:
            print("Model initialization Finished.")
            self.init_done = True
            self.lock = False

        outputs = [output.cpu().numpy().boxes for output in outputs]

        return outputs


def draw_text(img, text,
          font=cv2.FONT_HERSHEY_PLAIN,
          pos=(0, 0),
          font_scale=3,
          font_thickness=2,
          text_color=(0, 255, 0),
          text_color_bg=(0, 0, 0)
          ):

    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    cv2.rectangle(img, pos, (int(x + text_w), int(y + text_h)), text_color_bg, -1)
    cv2.putText(img, text, (int(x), int(y + text_h + font_scale - 1)), font, font_scale, text_color, font_thickness)

    return text_size       


def main():
    yolo_detector = YOLODetector()


if __name__ == "__main__":
    main()



