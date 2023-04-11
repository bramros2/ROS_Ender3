from range_filter import App, VideoThread
from keycontrol_gui import KeyControlWidget
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QAction, QStackedWidget


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setGeometry(0,0,1360,800)
        # Create a menu bar
        menubar = self.menuBar()

        # Create a "Home" action and add it to the menu bar
        home_action = QAction("Home", self)
        home_action.triggered.connect(self.show_home)
        menubar.addAction(home_action)

        # Create a "Range Filter" action and add it to the menu bar
        range_widget_action = QAction("Range filter", self)
        range_widget_action.triggered.connect(self.show_range_widget)
        menubar.addAction(range_widget_action)

        # Create a "Keycontrol" action and add it to the menu bar
        keyctrl_widget_action = QAction("Key Control", self)
        keyctrl_widget_action.triggered.connect(self.show_keyctrl_widget)
        menubar.addAction(keyctrl_widget_action)

        # Create a stacked widget to hold the different pages
        self.stacked_widget = QStackedWidget(self)

        # Create a label for the home page and add it to the stacked widget
        self.home_label = QLabel("This is the home page")
        self.stacked_widget.addWidget(self.home_label)

        # Create an instance of MyWidget and add it to the stacked widget
        self.range_widget = App()
        self.stacked_widget.addWidget(self.range_widget)

        #Create an instance of KeyControl widget and add it to the stacked widget
        self.keyctrl_widget = KeyControlWidget()
        self.stacked_widget.addWidget(self.keyctrl_widget)

        self.setCentralWidget(self.stacked_widget)

    def show_home(self):
        # Show the home page
        self.stacked_widget.setCurrentWidget(self.home_label)

    def show_range_widget(self):
        # Show the range widget page
        self.stacked_widget.setCurrentWidget(self.range_widget)

    def show_keyctrl_widget(self):
        #show the keycontrol widget
        self.stacked_widget.setCurrentWidget(self.keyctrl_widget)

if __name__ == "__main__":
    app = QApplication([])
    main_window = MainWindow()
    main_window.show()
    app.exec_()