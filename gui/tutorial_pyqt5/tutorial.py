import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QMessageBox, QPushButton, QToolTip, QLineEdit, QLabel
from PyQt5 import QtGui


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # window position
        self.top = 100
        self.left = 100

        # window geometry
        self.width = 800
        self.height = 600

        # window title
        self.title = "Guindaste Teleoperado"

        # create text box
        self.name_label = QLabel(self)
        self.name_label.move(10, 0)
        self.name_label.setText('Text box:')
        self.textbox = QLineEdit(self)
        self.textbox.move(10, 30)
        self.textbox.resize(280, 40)

        # create button
        self.test_button = QPushButton('Button One', self)
        self.test_button.move(10, 80)
        self.test_button.resize(280, 30)
        # self.test_button.setStyleSheet('QPushButton {background-color:#3F6F8D; font: bold; font-size: 15px}')

        # show window
        self.test_button.clicked.connect(self.on_click)

        # add label
        self.label_one = QLabel(self)
        self.label_one.setText('Label')
        self.label_one.move(200, 0)
        #self.label_one.resize(200, 18)
        #self.label_one.setStyleSheet('QLabel {background-color:#3F6F8D; font: bold; font-size: 15px}')

        # add label image
        self.seinao = QLabel(self)
        self.seinao.move(10, 400)
        self.seinao.resize(200, 200)
        self.seinao.setPixmap(QtGui.QPixmap('smallbluecar2.jpg'))

        # load main window
        self.load_window()

    def on_click(self):
        text_value = self.textbox.text()
        QMessageBox.question(self, 'Message', "It was typed: " + text_value, QMessageBox.Ok, QMessageBox.Ok)
        self.textbox.setText("")
        # text label
        self.label_one.setText('Clicou =)')
        self.label_one.setStyleSheet('QLabel {color: "green"}')
        # image
        self.seinao.setPixmap(QtGui.QPixmap('smallgrey2.jpg'))


    def load_window(self):
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setWindowTitle(self.title)
        self.show()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app_window = MainWindow()
    sys.exit(app.exec())
