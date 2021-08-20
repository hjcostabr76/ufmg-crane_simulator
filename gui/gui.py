import sys
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QMessageBox, QPushButton, QToolTip, QLineEdit, QLabel, QTextBrowser, QLCDNumber
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5 import QtGui

import sim
import time
import numpy as np
from PIL import Image


class Worker(QThread):
    image_update = pyqtSignal(QImage)
    gen_image_update = pyqtSignal(QImage)
    restart_simulation = False

    def run(self):
        self.ThreadActive = True
        self.simulation_started = False

        sim.simxFinish(-1)
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        robot_video = 'Vision_sensor0'
        simulation_video = 'Vision_sensor'

        if self.clientID != -1:
            # Colect camera handle
            _, video = sim.simxGetObjectHandle(self.clientID, robot_video, sim.simx_opmode_oneshot_wait)
            _, general_video = sim.simxGetObjectHandle(self.clientID, simulation_video, sim.simx_opmode_oneshot_wait)

            # First call to camera
            err_video, video_resolution, video_img = sim.simxGetVisionSensorImage(self.clientID, video, 0, sim.simx_opmode_streaming)
            err_gen_video, general_video_resolution, general_video_img = sim.simxGetVisionSensorImage(self.clientID, general_video, 0, sim.simx_opmode_streaming)
            time.sleep(0.01)


            while self.ThreadActive:
                if self.simulation_started:
                    sim.simxStartSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
                    time.sleep(0.02)
                    self.simulation_started = False
                    restart_simulation = True

                # control arm velocity
                self.set_arm_velocity()
                # Following calls to camera
                err_video, video_resolution, video_img = sim.simxGetVisionSensorImage(self.clientID, video, 0, sim.simx_opmode_buffer)
                err_gen_video, general_video_resolution, general_video_img = sim.simxGetVisionSensorImage(self.clientID, general_video, 0, sim.simx_opmode_buffer)

                if err_video == sim.simx_return_ok and err_gen_video == sim.simx_return_ok:
                    # Arm video
                    image_byte_array = np.uint8(video_img)
                    image_buffer = Image.frombuffer("RGB", (video_resolution[0], video_resolution[1]), image_byte_array, "raw")
                    img2 = np.asarray(image_buffer)
                    qt_image = QImage(img2, video_resolution[0], video_resolution[1], 3 * video_resolution[0], QtGui.QImage.Format_RGB888)
                    aa = qt_image.transformed(QtGui.QTransform().rotate(180))
                    aa = aa.transformed(QtGui.QTransform().scale(-1, 1))
                    self.image_update.emit(aa)

                    # General video
                    gen_image_byte_array = np.uint8(general_video_img)
                    gen_image_buffer = Image.frombuffer("RGB", (general_video_resolution[0],general_video_resolution[1]), gen_image_byte_array, "raw")
                    gen_img2 = np.asarray(gen_image_buffer)
                    gen_qt_image = QImage(gen_img2, general_video_resolution[0],general_video_resolution[1], 3*general_video_resolution[0], QtGui.QImage.Format_RGB888)
                    bb = gen_qt_image.transformed(QtGui.QTransform().rotate(180))
                    bb = bb.transformed(QtGui.QTransform().scale(-1, 1))
                    self.gen_image_update.emit(bb)

    def set_arm_velocity(self):
        robot_arm_act = 'Arm_actuator'
        _, arm_act = sim.simxGetObjectHandle(self.clientID, robot_arm_act, sim.simx_opmode_blocking)
        self.arm_velocity = 0.04
        sim.simxSetJointTargetVelocity(self.clientID, arm_act, self.arm_velocity, sim.simx_opmode_continuous)

    def get_arm_position(self):
        _, robot_arm = 'Arm'

    def get_mass_information(self):
        mass_name = 'Mass'
        _, mass_handle = sim.simxGetObjectHandle(self.clientID, mass_name, sim.simx_opmode_blocking)
        sim.simxGetObjectPosition(self.clientID, mass_handle, 0.0, )

    def pause_simulation(self):
        sim.simxPauseSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
        # sim.simxFinish(self.clientID)

    def stop(self):
        self.ThreadActive = False
        self.quit()


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

        # add label
        self.lcd_label = QLabel(self)
        self.lcd_label.move(10, 175)
        self.lcd_label.resize(255, 30)
        self.lcd_label.setText('Status da Simulação')

        self.lcd1_label = QLabel(self)
        self.lcd1_label.move(10, 200)
        self.lcd1_label.resize(255, 30)
        self.lcd1_label.setText('\t     --')
        self.lcd1_label.setStyleSheet('QLabel {background-color:#dbdbdb; font: bold; font-size: 15px; font-family: Courier}')

        # create start button
        self.test_button = QPushButton('Start', self)
        self.test_button.move(10, 235)
        self.test_button.resize(120, 30)
        # pop up window
        self.test_button.clicked.connect(self.on_click_start)

        # create stop button
        self.test_button = QPushButton('Stop', self)
        self.test_button.move(145, 235)
        self.test_button.resize(120, 30)
        # pop up window
        self.test_button.clicked.connect(self.on_click_stop)

        # start streaming video thread
        self.worker = Worker()
        self.worker.start()

        # video from camera in arm
        self.arm_camera_view = QLabel(self)
        self.arm_camera_view.move(10, 260)
        self.arm_camera_view.resize(400, 400)
        # show video
        self.worker.image_update.connect(self.arm_image_update_slot)

        # video from general view camera
        self.gen_camera_view = QLabel(self)
        self.gen_camera_view.move(300, 260)
        self.gen_camera_view.resize(400, 400)
        # show video
        self.worker.gen_image_update.connect(self.gen_image_update_slot)

        # load main window
        self.load_window()

    def arm_image_update_slot(self, image):
        self.arm_camera_view.setPixmap(QPixmap.fromImage(image))

    def gen_image_update_slot(self, image):
        self.gen_camera_view.setPixmap(QPixmap.fromImage(image))

    def on_click_start(self):
        QMessageBox.question(self, 'Message', "Simulation started.", QMessageBox.Ok, QMessageBox.Ok)
        self.worker.simulation_started = True
        self.lcd1_label.setText('\t     ON')
        self.lcd1_label.setStyleSheet('QLabel {background-color:#e3fadc; font: bold; font-size: 15px; font-family: Courier}')

    def on_click_stop(self):
        self.worker.pause_simulation()
        QMessageBox.question(self, 'Message', "Simulation stopped.", QMessageBox.Ok, QMessageBox.Ok)
        self.lcd1_label.setText('\t     OFF')
        self.lcd1_label.setStyleSheet('QLabel {background-color:#ffa8a8; font: bold; font-size: 15px; font-family: Courier;}')

    def load_window(self):
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setWindowTitle(self.title)
        self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app_window = MainWindow()
    sys.exit(app.exec())
