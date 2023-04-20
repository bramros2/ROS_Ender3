import sys
import cv2
import numpy as np
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QPushButton, QGridLayout
from PyQt5.QtCore import Qt, QTimer


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenCV Video Stream")
        self.setGeometry(100, 100, 1000, 800)

        # Create the central widget and layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        grid_layout = QGridLayout(central_widget)
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 2)
        grid_layout.setColumnStretch(2, 1)

        # Create the video frame label and add it to the layout
        self.video_frame = QLabel(self)
        self.video_frame.setFixedSize(640, 480)
        self.video_frame.setAlignment(Qt.AlignCenter)
        grid_layout.addWidget(self.video_frame, 0, 0, 1, 3)

        # Create the buttons and add them to the layout
        self.rectangle_button = QPushButton("Draw Rectangle", self)
        self.rectangle_button.clicked.connect(self.draw_rectangle)
        grid_layout.addWidget(self.rectangle_button, 1, 0)

        self.save_frame_button = QPushButton("Save Frame", self)
        self.save_frame_button.clicked.connect(self.save_frame)
        grid_layout.addWidget(self.save_frame_button, 1, 1)

        self.save_rect_button = QPushButton("Save Rect", self)
        self.save_rect_button.clicked.connect(self.save_rect)
        grid_layout.addWidget(self.save_rect_button, 1, 2)

        self.subtraction_button = QPushButton("Subtraction", self)
        self.subtraction_button.setCheckable(True)
        self.subtraction_button.clicked.connect(self.toggle_subtraction)
        grid_layout.addWidget(self.subtraction_button, 2, 0, 1, 3)

        # Create the labels and add them to the layout
        self.drawing_label = QLabel(self)
        grid_layout.addWidget(self.drawing_label, 3, 0, Qt.AlignCenter)

        self.start_label = QLabel(self)
        grid_layout.addWidget(self.start_label, 3, 1, Qt.AlignCenter)

        self.end_label = QLabel(self)
        grid_layout.addWidget(self.end_label, 3, 2, Qt.AlignCenter)

        self.rect_label = QLabel(self)
        grid_layout.addWidget(self.rect_label, 4, 0, Qt.AlignCenter)

        self.stored_label = QLabel(self)
        grid_layout.addWidget(self.stored_label, 4, 1, Qt.AlignCenter)

        self.subtraction_label = QLabel(self)
        grid_layout.addWidget(self.subtraction_label, 4, 2, Qt.AlignCenter)

        self.coords_label = QLabel(self)
        grid_layout.addWidget(self.coords_label, 5, 0, Qt.AlignCenter)

        self.background_label = QLabel(self)
        grid_layout.addWidget(self.background_label, 5, 1, Qt.AlignCenter)

        # Initialize instance variables
        self.is_drawing = False
        self.rect_start = None
        self.rect_end = None
        self.rectangle = None
        self.stored_frame = None
        self.subtraction_on = False
        self.rect_coords = None
        self.background = None

        # Start the video stream
        self.capture = cv2.VideoCapture(0)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(5)

        # Update labels
        self.update_labels()

    def update_labels(self):
        self.drawing_label.setText(f"Drawing: {self.is_drawing}")
        self.start_label.setText(f"Rect start: {self.rect_start}")
        self.end_label.setText(f"Rect end: {self.rect_end}")
        self.rect_label.setText(f"Rectangle: {self.rectangle}")
        self.stored_label.setText(f"Stored frame: {self.stored_frame is not None}")
        self.subtraction_label.setText(f"Subtraction on: {self.subtraction_on}")
        self.coords_label.setText(f"Rect coords: {self.rect_coords}")
        self.background_label.setText(f"Background: {self.background is not None}")


    def update_frame(self):
        ret, frame = self.capture.read()
        if ret:
            if self.subtraction_on and self.background is not None:
                frame = cv2.absdiff(frame, self.background)
            q_image = QImage(frame.data, frame.shape[1], frame.shape[0], QImage.Format_BGR888)
            painter = QPainter(q_image)
            painter.setPen(QPen(Qt.blue, 2, Qt.SolidLine))
            if self.rectangle is not None:
                x, y, w, h = self.rectangle
                painter.drawRect(x, y, w, h)
            painter.end()
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            self.video_frame.setPixmap(QPixmap.fromImage(q_image).scaled(w, h, Qt.KeepAspectRatio))
            self.update_labels()
                
    def save_frame(self):
        ret, frame = self.capture.read()
        if ret:
            self.background = frame.copy()
    
    def save_rect(self):
        self.rectangle_coords = self.get_rect_coords()

    def get_rect_coords(self):
        if self.rectangle is None:
            return None
        x, y, w, h = self.rectangle
        x_min, y_min, x_max, y_max = x / self.capture.get(cv2.CAP_PROP_FRAME_WIDTH), y / self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT), (x + w) / self.capture.get(cv2.CAP_PROP_FRAME_WIDTH), (y + h) / self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        return [x_min, y_min, x_max, y_max]
    
    def toggle_subtraction(self):
        if self.subtraction_on:
            self.subtraction_on = False    
        else:
            self.subtraction_on = True

    def draw_rectangle(self):
        if self.is_drawing:
            self.is_drawing = False
            self.rectangle = self.get_rect()
            self.rect_start = None
            self.rect_end = None
        else:
            self.is_drawing = True

    def mousePressEvent(self, event):
        if self.is_drawing:
            if self.rect_start is None:
                self.rect_start = (event.x(), event.y())
            else:
                self.rect_end = (event.x(), event.y())
                self.draw_rectangle()

    def get_rect(self):
        if self.rect_start is None or self.rect_end is None:
            return None
        x, y, w, h = cv2.boundingRect(np.array([self.rect_start, self.rect_end]))
        return (x, y, w, h)

    def closeEvent(self, event):
        self.capture.release()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
