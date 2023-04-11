from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QMessageBox, \
    QFileDialog
from PyQt5.QtGui import QPixmap
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
import re

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self):
        super().__init__()
        self._run_flag = True

    def run(self):
        # capture from webcam
        cap = cv2.VideoCapture(0)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                self.change_pixmap_signal.emit(cv_img)

        # shut down capture system
        cap.release()

    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.wait()

class App(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Range Detector")
        self.disply_width = 640
        self.display_height = 480
        # create the label that holds the image
        self.image_label = QLabel(self)
        self.image_label.resize(self.disply_width, self.display_height)
        self.image_label2 = QLabel(self)
        self.image_label2.resize(self.disply_width,self.display_height)
        # create a text label
        self.textLabel = QLabel('Webcam')
        self.textLabel2 = QLabel('Threshold')

        self.hmin = 0
        self.smin = 0
        self.vmin = 0
        self.hmax = 255
        self.smax = 255
        self.vmax = 255

        hmin_label = QLabel('Minimum Hue:')
        self.hmin_slider = QSlider()
        self.hmin_slider.setMinimum(0)
        self.hmin_slider.setMaximum(255)
        self.hmin_slider.setValue(self.hmin)
        self.hmin_slider.setOrientation(Qt.Horizontal)
        self.hmin_slider.valueChanged.connect(self.on_hmin_slider_changed)
        self.hmin_val_label = QLabel(str(self.hmin))

        smin_label = QLabel('Minimum Saturation:')
        self.smin_slider = QSlider()
        self.smin_slider.setMinimum(0)
        self.smin_slider.setMaximum(255)
        self.smin_slider.setValue(self.smin)
        self.smin_slider.setOrientation(Qt.Horizontal)
        self.smin_slider.valueChanged.connect(self.on_smin_slider_changed)
        self.smin_val_label = QLabel(str(self.smin))

        vmin_label = QLabel('Minimum Value:')
        self.vmin_slider = QSlider()
        self.vmin_slider.setMinimum(0)
        self.vmin_slider.setMaximum(255)
        self.vmin_slider.setValue(self.vmin)
        self.vmin_slider.setOrientation(Qt.Horizontal)
        self.vmin_slider.valueChanged.connect(self.on_vmin_slider_changed)
        self.vmin_val_label = QLabel(str(self.vmin))

        hmax_label = QLabel('Maximum Hue:')
        self.hmax_slider = QSlider()
        self.hmax_slider.setMinimum(0)
        self.hmax_slider.setMaximum(255)
        self.hmax_slider.setValue(self.hmax)
        self.hmax_slider.setOrientation(Qt.Horizontal)
        self.hmax_slider.valueChanged.connect(self.on_hmax_slider_changed)
        self.hmax_val_label = QLabel(str(self.hmax))

        smax_label = QLabel('Maximum Saturation:')
        self.smax_slider = QSlider()
        self.smax_slider.setMinimum(0)
        self.smax_slider.setMaximum(255)
        self.smax_slider.setValue(self.smax)
        self.smax_slider.setOrientation(Qt.Horizontal)
        self.smax_slider.valueChanged.connect(self.on_smax_slider_changed)
        self.smax_val_label = QLabel(str(self.smax))

        vmax_label = QLabel('Maximum Value:')
        self.vmax_slider = QSlider()
        self.vmax_slider.setMinimum(0)
        self.vmax_slider.setMaximum(255)
        self.vmax_slider.setValue(self.vmax)
        self.vmax_slider.setOrientation(Qt.Horizontal)
        self.vmax_slider.valueChanged.connect(self.on_vmax_slider_changed)
        self.vmax_val_label = QLabel(str(self.vmax))

        reset_button = QPushButton('Reset')
        reset_button.setFixedWidth(100)
        reset_button.clicked.connect(self.on_reset_button_clicked)

        save_button = QPushButton('Save')
        save_button.setFixedWidth(100)
        save_button.clicked.connect(self.on_save_button_clicked)

        load_button = QPushButton('Load')
        load_button.setFixedWidth(100)
        load_button.clicked.connect(self.on_load_button_clicked)

        # create a vertical box layout and add the two labels
        top_layout1 = QVBoxLayout()
        top_layout2 = QVBoxLayout()

        top_layout1.addWidget(self.textLabel)
        top_layout1.addWidget(self.image_label)

        # set the vbox layout as the widgets layout
        top_layout2.addWidget(self.textLabel2)
        top_layout2.addWidget(self.image_label2)

        # set the vbox layout as the widgets layout
        button_layout = QHBoxLayout()
        button_layout.addWidget(save_button, alignment= Qt.AlignCenter)
        button_layout.addWidget(load_button, alignment=Qt.AlignCenter)
        button_layout.addWidget(reset_button, alignment=Qt.AlignCenter)

        bottom_layout = QVBoxLayout()
        hmin_layout = QHBoxLayout()
        hmin_layout.addWidget(hmin_label, alignment=Qt.AlignRight)
        hmin_layout.addWidget(self.hmin_slider, alignment=Qt.AlignRight)
        hmin_layout.addWidget(self.hmin_val_label, alignment=Qt.AlignRight)
        bottom_layout.addLayout(hmin_layout)

        smin_layout = QHBoxLayout()
        smin_layout.addWidget(smin_label, alignment=Qt.AlignRight)
        smin_layout.addWidget(self.smin_slider, alignment=Qt.AlignRight)
        smin_layout.addWidget(self.smin_val_label, alignment=Qt.AlignRight)
        bottom_layout.addLayout(smin_layout)

        vmin_layout = QHBoxLayout()
        vmin_layout.addWidget(vmin_label, alignment=Qt.AlignRight)
        vmin_layout.addWidget(self.vmin_slider, alignment=Qt.AlignRight)
        vmin_layout.addWidget(self.vmin_val_label, alignment=Qt.AlignRight)
        bottom_layout.addLayout(vmin_layout)

        hmax_layout = QHBoxLayout()
        hmax_layout.addWidget(hmax_label, alignment=Qt.AlignRight)
        hmax_layout.addWidget(self.hmax_slider, alignment=Qt.AlignRight)
        hmax_layout.addWidget(self.hmax_val_label, alignment=Qt.AlignRight)
        bottom_layout.addLayout(hmax_layout)

        smax_layout = QHBoxLayout()
        smax_layout.addWidget(smax_label, alignment=Qt.AlignRight)
        smax_layout.addWidget(self.smax_slider, alignment=Qt.AlignRight)
        smax_layout.addWidget(self.smax_val_label, alignment=Qt.AlignRight)
        bottom_layout.addLayout(smax_layout)

        vmax_layout = QHBoxLayout()
        vmax_layout.addWidget(vmax_label, alignment=Qt.AlignRight)
        vmax_layout.addWidget(self.vmax_slider, alignment=Qt.AlignRight)
        vmax_layout.addWidget(self.vmax_val_label, alignment=Qt.AlignRight)
        bottom_layout.addLayout(vmax_layout)

        bottom_layout.addLayout(button_layout)
        #bottom_layout.addWidget(save_button)
        #bottom_layout.addWidget(load_button)

        top_layout = QHBoxLayout()
        top_layout.addLayout(top_layout1)
        top_layout.addLayout(top_layout2)

        layout = QVBoxLayout()
        layout.addLayout(top_layout)
        layout.addLayout(bottom_layout)
        self.setLayout(layout)

        # create the video capture thread
        self.thread = VideoThread()
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)
        frame_to_thresh = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(frame_to_thresh, (self.hmin, self.smin, self.vmin), (self.hmax, self.smax, self.vmax))
        qt_img2 = self.convert_cv_qt2(thresh)
        self.image_label2.setPixmap(qt_img2)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def convert_cv_qt2(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        gray_image = cv_img
        h, w = gray_image.shape
        bytes_per_line = 1 * w
        convert_to_Qt_format = QtGui.QImage(gray_image.data, w, h, bytes_per_line, QtGui.QImage.Format_Grayscale8)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def on_hmin_slider_changed(self, value):
        if self.hmax <= value:
            value = self.hmax -1
        self.hmin = value
        self.hmin_slider.setValue(self.hmin)
        self.hmin_val_label.setText(str(value))

    def on_smin_slider_changed(self, value):
        if self.smax <= value:
            value = self.smax -1
        self.smin = value
        self.smin_slider.setValue(self.smin)
        self.smin_val_label.setText(str(value))

    def on_vmin_slider_changed(self, value):
        if self.vmax <= value:
            value = self.vmax -1
        self.vmin = value
        self.vmin_slider.setValue(self.vmin)
        self.vmin_val_label.setText(str(value))

    def on_hmax_slider_changed(self, value):
        if self.hmin >= value:
            value = self.hmin +1
        self.hmax = value
        self.hmax_slider.setValue(self.hmax)
        self.hmax_val_label.setText(str(value))

    def on_smax_slider_changed(self, value):
        if self.smin >= value:
            value = self.smin +1
        self.smax = value
        self.smax_slider.setValue(self.smax)
        self.smax_val_label.setText(str(value))

    def on_vmax_slider_changed(self, value):
        if self.vmin >= value:
            value = self.vmin +1
        self.vmax = value
        self.vmax_slider.setValue(self.vmax)
        self.vmax_val_label.setText(str(value))

    def on_reset_button_clicked(self):
        self.reset_sliders()

    def reset_sliders(self):
        self.hmin = 0
        self.hmin_slider.setValue(self.hmin)
        self.hmin_val_label.setText(str(self.hmin))
        self.smin = 0
        self.smin_slider.setValue(self.smin)
        self.smin_val_label.setText(str(self.smin))
        self.vmin = 0
        self.vmin_slider.setValue(self.vmin)
        self.vmin_val_label.setText(str(self.vmin))
        self.hmax = 255
        self.hmax_slider.setValue(self.hmax)
        self.hmax_val_label.setText(str(self.hmax))
        self.smax = 255
        self.smax_slider.setValue(self.smax)
        self.smax_val_label.setText(str(self.smax))
        self.vmax = 255
        self.vmax_slider.setValue(self.vmax)
        self.vmax_val_label.setText(str(self.vmax))

    def on_save_button_clicked(self):
        if self.hmin >= self.hmax or self.smin >= self.smax or self.vmin >= self.vmax:
            QMessageBox.warning(self, 'Error', 'Minimum value should be lower than maximum value')
        else:
            # get file name to save the settings
            file_name, _ = QFileDialog.getSaveFileName(self, 'Save Settings', '', 'Text files (*.txt)')

            # save settings to file
            if file_name:
                with open(file_name, 'w') as f:
                    f.write(f'hmin={self.hmin}\n')
                    f.write(f'smin={self.smin}\n')
                    f.write(f'vmin={self.vmin}\n')
                    f.write(f'hmax={self.hmax}\n')
                    f.write(f'smax={self.smax}\n')
                    f.write(f'vmax={self.vmax}')
                QMessageBox.information(self, 'Saved', 'Settings saved successfully.')

    def on_load_button_clicked(self):

        # Use file dialog to allow user to select a file
        file_path, _ = QFileDialog.getOpenFileName(self, 'Open Settings File', '', 'Text Files (*.txt)')

        # If a file was selected, read the values from the file and update the sliders
        if file_path:
            with open(file_path, 'r') as f:
                values = f.read().split('\n')
                if len(values) == 6:
                    try:
                        hmin = int(re.findall("\d+", values[0])[0])
                        smin = int(re.findall("\d+", values[1])[0])
                        vmin = int(re.findall("\d+", values[2])[0])
                        hmax = int(re.findall("\d+", values[3])[0])
                        smax = int(re.findall("\d+", values[4])[0])
                        vmax = int(re.findall("\d+", values[5])[0])

                        # Check that min values are lower than max values
                        if hmin < hmax and smin < smax and vmin < vmax:
                            self.hmin = hmin
                            self.smin = smin
                            self.vmin = vmin
                            self.hmax = hmax
                            self.smax = smax
                            self.vmax = vmax

                            # Update sliders
                            self.hmin_slider.setValue(hmin)
                            self.smin_slider.setValue(smin)
                            self.vmin_slider.setValue(vmin)
                            self.hmax_slider.setValue(hmax)
                            self.smax_slider.setValue(smax)
                            self.vmax_slider.setValue(vmax)

                            # Update labels
                            self.hmin_val_label.setText(str(hmin))
                            self.smin_val_label.setText(str(smin))
                            self.vmin_val_label.setText(str(vmin))
                            self.hmax_val_label.setText(str(hmax))
                            self.smax_val_label.setText(str(smax))
                            self.vmax_val_label.setText(str(vmax))
                        else:
                            QMessageBox.warning(self, "Invalid Values", "The values in the file are not valid.")
                    except ValueError:
                        QMessageBox.warning(self, "Invalid File", "The file does not contain valid integer values.")
                else:
                    QMessageBox.warning(self, "Invalid File", "The file does not contain the correct number of values.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())