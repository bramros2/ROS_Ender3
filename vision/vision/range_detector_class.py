import cv2
import argparse
from operator import xor


class RangeDetector:
    def __init__(self, range_filter, image=None, webcam=False, preview=False):
        self.range_filter = range_filter.upper()
        self.image_path = image
        self.use_webcam = webcam
        self.show_preview = preview
        self.camera = None

    def callback(self, value):
        pass

    def setup_trackbars(self):
        cv2.namedWindow("Trackbars", 0)

        for i in ["MIN", "MAX"]:
            v = 0 if i == "MIN" else 255

            for j in self.range_filter:
                cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, self.callback)

    def get_trackbar_values(self):
        values = []

        for i in ["MIN", "MAX"]:
            for j in self.range_filter:
                v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
                values.append(v)

        return values

    def run(self):
        if not xor(bool(self.image_path), bool(self.use_webcam)):
            raise ValueError("Please specify only one image source")

        if not self.range_filter in ['RGB', 'HSV']:
            raise ValueError("Please speciy a correct filter.")

        if self.image_path:
            image = cv2.imread(self.image_path)

            if self.range_filter == 'RGB':
                frame_to_thresh = image.copy()
            else:
                frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        else:
            self.camera = cv2.VideoCapture(0)

        self.setup_trackbars()

        while True:
            if self.use_webcam:
                ret, image = self.camera.read()

                if not ret:
                    break

                if self.range_filter == 'RGB':
                    frame_to_thresh = image.copy()
                else:
                    frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = self.get_trackbar_values()

            thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            if self.show_preview:
                preview = cv2.bitwise_and(image, image, mask=thresh)
                cv2.imshow("Preview", preview)
            else:
                cv2.imshow("Original", image)
                cv2.imshow("Thresh", thresh)

            if cv2.waitKey(1) & 0xFF is ord('q'):
                break


range = RangeDetector(range_filter= 'HSV', webcam=True)
range.run()