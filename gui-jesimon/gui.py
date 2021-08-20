# -*- coding: utf-8 -*-
from connection import Connection
import time
import numpy as np
# Form implementation generated from reading ui file 'C:\Users\JB\Documents\lab_3\guindaste.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!
#
# Convertido para PyQt5

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtWidgets.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtWidgets.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtWidgets.QApplication.translate(context, text, disambig)

class Ui_Guindaste(object):

    def setupUi(self, Guindaste):
        #create connection
        control = Connection()

        # Cores da GUI
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Background, QtGui.QColor(225,230,245))
        palette.setColor(QtGui.QPalette.Light, QtGui.QColor(190,190,235))

        # Janela principal da GUI
        Guindaste.setObjectName(_fromUtf8("Guindaste"))
        Guindaste.resize(875, 650)
        Guindaste.setPalette(palette)
        Guindaste.setFont(QtGui.QFont("Helvetica"))

        # Titulo central "Controle do Guindaste"
        self.label_5 = QtWidgets.QLabel(Guindaste)
        self.label_5.setGeometry(QtCore.QRect(245, 20, 400, 41))
        self.label_5.setStyleSheet(_fromUtf8("font: 18pt \"Helvetica\";"))
        self.label_5.setObjectName(_fromUtf8("label_5"))

        # Imagem guindaste
        self.pixmap = QtGui.QPixmap('crane.png')
        self.label_13 = QtWidgets.QLabel(Guindaste)
        self.label_13.setPixmap(self.pixmap)
        self.label_13.setGeometry(QtCore.QRect(30, 100, 150, 230))
        self.label_13.setScaledContents(True)

        # Telas centrais
        self.graphicsView = QtWidgets.QGraphicsView(Guindaste)
        self.graphicsView.setGeometry(QtCore.QRect(207, 125, 220, 220))
        self.graphicsView.setObjectName(_fromUtf8("graphicsView"))

        self.view = QtWidgets.QLabel(Guindaste)
        self.view.setGeometry(QtCore.QRect(207, 125, 220, 220))
        self.pixmap = QtGui.QPixmap()
        self.view.setPixmap(self.pixmap)

        self.label_14 = QtWidgets.QLabel(Guindaste)
        self.label_14.setGeometry(QtCore.QRect(207, 90, 220, 41))
        self.label_14.setObjectName(_fromUtf8("label_14"))

        self.graphicsView2 = QtWidgets.QGraphicsView(Guindaste)
        self.graphicsView2.setGeometry(QtCore.QRect(448, 125, 220, 220))
        self.graphicsView2.setObjectName(_fromUtf8("graphicsView2"))

        self.view2 = QtWidgets.QLabel(Guindaste)
        self.view2.setGeometry(QtCore.QRect(448, 125, 220, 220))
        self.pixmap2 = QtGui.QPixmap()
        self.view2.setPixmap(self.pixmap2)

        self.label_15 = QtWidgets.QLabel(Guindaste)
        self.label_15.setGeometry(QtCore.QRect(448, 90, 220, 41))
        self.label_15.setObjectName(_fromUtf8("label_15"))

        #LCDs colors
        self.off_color = QtGui.QPalette()
        self.off_color.setColor(QtGui.QPalette.Light, QtGui.QColor(255,145,145))
        self.on_color = QtGui.QPalette()
        self.on_color.setColor(QtGui.QPalette.Light, QtGui.QColor(145,225,145))

        # LCDs superiores direitos
        self.lcdNumber_4 = QtWidgets.QLCDNumber(Guindaste)
        self.lcdNumber_4.setGeometry(QtCore.QRect(741, 160, 60, 30))
        self.lcdNumber_4.setObjectName(_fromUtf8("lcdNumber_4"))
        self.lcdNumber_4.setPalette(self.off_color)

        self.label_10 = QtWidgets.QLabel(Guindaste)
        self.label_10.setGeometry(QtCore.QRect(710, 130, 121, 31))
        self.label_10.setObjectName(_fromUtf8("label_10"))

        self.lcdNumber_5 = QtWidgets.QLCDNumber(Guindaste)
        self.lcdNumber_5.setGeometry(QtCore.QRect(741, 230, 60, 30))
        self.lcdNumber_5.setObjectName(_fromUtf8("lcdNumber_5"))
        self.lcdNumber_5.setPalette(self.off_color)

        self.label_12 = QtWidgets.QLabel(Guindaste)
        self.label_12.setGeometry(QtCore.QRect(710, 200, 131, 31))
        self.label_12.setObjectName(_fromUtf8("label_12"))

        self.lcdNumber_6 = QtWidgets.QLCDNumber(Guindaste)
        self.lcdNumber_6.setGeometry(QtCore.QRect(741, 300, 60, 30))
        self.lcdNumber_6.setObjectName(_fromUtf8("lcdNumber_6"))
        self.lcdNumber_6.setPalette(self.off_color)

        self.label_8 = QtWidgets.QLabel(Guindaste)
        self.label_8.setGeometry(QtCore.QRect(710, 270, 131, 31))
        self.label_8.setObjectName(_fromUtf8("label_8"))

        # LCDs inferiores direitos
        self.groupBox_5 = QtWidgets.QGroupBox(Guindaste)
        self.groupBox_5.setGeometry(QtCore.QRect(679, 380, 181, 256))
        self.groupBox_5.setObjectName(_fromUtf8("groupBox_4"))

        self.lcdNumber_2 = QtWidgets.QLCDNumber(Guindaste)
        self.lcdNumber_2.setGeometry(QtCore.QRect(741, 468, 60, 30))
        self.lcdNumber_2.setObjectName(_fromUtf8("lcdNumber_2"))

        self.label_7 = QtWidgets.QLabel(Guindaste)
        self.label_7.setGeometry(QtCore.QRect(700, 438, 160, 31))
        self.label_7.setObjectName(_fromUtf8("label_7"))

        self.lcdNumber = QtWidgets.QLCDNumber(Guindaste)
        self.lcdNumber.setGeometry(QtCore.QRect(741, 548, 60, 30))
        self.lcdNumber.setObjectName(_fromUtf8("lcdNumber"))

        self.label = QtWidgets.QLabel(Guindaste)
        self.label.setGeometry(QtCore.QRect(700, 518, 151, 31))
        self.label.setObjectName(_fromUtf8("label"))

        # Quadro inferior esquerdo "Controle do Guindaste"
        self.groupBox_4 = QtWidgets.QGroupBox(Guindaste)
        self.groupBox_4.setGeometry(QtCore.QRect(15, 380, 181, 151))
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))

        self.pushButton_7 = QtWidgets.QPushButton(self.groupBox_4)
        self.pushButton_7.setGeometry(QtCore.QRect(50, 35, 80, 30))
        self.pushButton_7.setObjectName(_fromUtf8("pushButton_7"))

        self.pushButton_2 = QtWidgets.QPushButton(self.groupBox_4)
        self.pushButton_2.setGeometry(QtCore.QRect(50, 90, 80, 30))
        self.pushButton_2.setObjectName(_fromUtf8("pushButton_2"))

        # Quadro inferior esquerdo "Controle do imã"
        self.groupBox_3 = QtWidgets.QGroupBox(Guindaste)
        self.groupBox_3.setGeometry(QtCore.QRect(15, 540, 181, 96))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))

        self.pushButton_6 = QtWidgets.QPushButton(self.groupBox_3)
        self.pushButton_6.setGeometry(QtCore.QRect(50, 40, 80, 30))
        self.pushButton_6.setObjectName(_fromUtf8("pushButton_6"))

        # Quadro inferior central "Controle Direcional com Botões"
        self.groupBox = QtWidgets.QGroupBox(Guindaste)
        self.groupBox.setGeometry(QtCore.QRect(207, 380, 461, 256))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))

        self.lineEdit = QtWidgets.QLineEdit(self.groupBox)
        self.lineEdit.setGeometry(QtCore.QRect(190, 35, 80, 30))
        self.lineEdit.setObjectName(_fromUtf8("lineEdit"))

        self.label_9 = QtWidgets.QLabel(self.groupBox)
        self.label_9.setGeometry(QtCore.QRect(50, 35, 130, 31))
        self.label_9.setObjectName(_fromUtf8("label_9"))

        self.pushButton_5 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_5.setGeometry(QtCore.QRect(190, 85, 80, 45))
        self.pushButton_5.setObjectName(_fromUtf8("pushButton_5"))

        self.pushButton_3 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_3.setGeometry(QtCore.QRect(190, 195, 80, 45))
        self.pushButton_3.setObjectName(_fromUtf8("pushButton_3"))

        self.pushButton = QtWidgets.QPushButton(self.groupBox)
        self.pushButton.setGeometry(QtCore.QRect(103, 140, 80, 45))
        self.pushButton.setObjectName(_fromUtf8("pushButton"))

        self.pushButton_4 = QtWidgets.QPushButton(self.groupBox)
        self.pushButton_4.setGeometry(QtCore.QRect(277, 140, 80, 45))
        self.pushButton_4.setObjectName(_fromUtf8("pushButton_4"))

        # Janelas de confirmacao
        self.confirmWindow = QtWidgets.QMessageBox()
        self.confirmWindow.setIcon(QtWidgets.QMessageBox.Question)
        self.confirmWindow.setWindowTitle('Controle do Imã')
        self.confirmWindow.setText('Desligar imã?')
        self.confirmWindow.setStandardButtons(QtWidgets.QMessageBox.Yes|QtWidgets.QMessageBox.No)
        self.buttonY = self.confirmWindow.button(QtWidgets.QMessageBox.Yes)
        self.buttonY.setText('Sim')
        self.buttonN = self.confirmWindow.button(QtWidgets.QMessageBox.No)
        self.buttonN.setText('Não')
        self.confirmWindow.setDefaultButton(self.buttonN)

        self.confirmWindow_2 = QtWidgets.QMessageBox()
        self.confirmWindow_2.setIcon(QtWidgets.QMessageBox.Question)
        self.confirmWindow_2.setWindowTitle('Controle do Guindaste')
        self.confirmWindow_2.setText("AVISO: Imã ligado.\nDesligue-o antes de concluir esta ação.")
        self.confirmWindow_2.setStandardButtons(QtWidgets.QMessageBox.Ok)


        self.groupBox_5.raise_()
        self.groupBox_4.raise_()
        self.groupBox_3.raise_()
        self.groupBox.raise_()
        self.lcdNumber.raise_()
        self.label.raise_()
        self.graphicsView.raise_()
        self.graphicsView2.raise_()
        self.view.raise_()
        self.view2.raise_()
        self.label_5.raise_()
        self.lcdNumber_2.raise_()
        self.label_7.raise_()
        self.label_8.raise_()
        self.label_10.raise_()
        self.lcdNumber_4.raise_()
        self.label_12.raise_()
        self.lcdNumber_5.raise_()
        self.lcdNumber_6.raise_()
        self.lcdNumber_4.display('Off')
        self.lcdNumber_6.display('Off')
        self.lcdNumber_5.display('Off')
        self.lcdNumber_2.display(0)

        #Start conection Connection
        def connect():
            print('UI: Start Connection')
            _, status = control.init_connection(ip = '127.0.0.1' , port = 19997)
            if status:
                self.lcdNumber_4.display('UP')
                self.lcdNumber_4.setPalette(self.on_color)
            else:
                self.lcdNumber_4.display('Off')
                self.lcdNumber_4.setPalette(self.off_color)

        self.pushButton_7.clicked.connect(connect)
        
        # Transforma os algulos negativos para positivos
        def getAngle360(angle):
            if angle < 0:
                angle += 360
            return angle

        # Left
        def left():
            self.pushButton.blockSignals(True)
            self.pushButton_4.blockSignals(True)
            self.pushButton_5.blockSignals(True)
            self.pushButton_3.blockSignals(True)
            cod = self.lineEdit.text()
            try:
                old_angle = control.getCurrentAngleClaw()
                step = float(cod)
                status = control.commandLeft(step)
            except:
                status = False
            if status:
                desired_angle = getAngle360(old_angle + step)
                cnt = 0
                angle = getAngle360(control.getCurrentAngleClaw())
                while(abs(angle - desired_angle) > 0.1 and cnt < 500):
                    self.lcdNumber_2.display(round(angle))
                    app.processEvents()
                    angle = getAngle360(control.getCurrentAngleClaw())
                    control.getStatusDist()
                    cnt+=1
                    im1 = control.getCamImage(save=True,number=1)
                    im2 = control.getCamImage(save=True,number=2)
                    if im1 is not None:
                        self.pixmap = QtGui.QPixmap(im1)
                        self.pixmap = self.pixmap.scaled(220,220)
                        self.view.setPixmap(self.pixmap)
                    if im2 is not None:
                        self.pixmap2 = QtGui.QPixmap(im2)
                        self.pixmap2 = self.pixmap2.scaled(220,220)
                        self.view2.setPixmap(self.pixmap2)
            else:
                print('Não pode ser alterado por não estar ligado ou Falta valor de passo no campo acima')
            self.pushButton.blockSignals(False)
            self.pushButton_4.blockSignals(False)
            self.pushButton_5.blockSignals(False)
            self.pushButton_3.blockSignals(False)

        self.pushButton.clicked.connect(left)

        # Right
        def right():
            self.pushButton_4.blockSignals(True)
            self.pushButton.blockSignals(True)
            self.pushButton_5.blockSignals(True)
            self.pushButton_3.blockSignals(True)
            cod = self.lineEdit.text()
            try:
                old_angle = control.getCurrentAngleClaw()
                step = float(cod)
                status = control.commandRight(step)
            except Exception as error:
                status = False
                print(error)

            if status:
                desired_angle = getAngle360(old_angle - step)
                cnt = 0
                angle = getAngle360(control.getCurrentAngleClaw())
                while(abs(angle - desired_angle) > 0.1 and cnt < 500):
                    self.lcdNumber_2.display(round(angle))
                    app.processEvents()
                    angle = getAngle360(control.getCurrentAngleClaw())
                    cnt+=1
                    im1 = control.getCamImage(save=True,number=1)
                    im2 = control.getCamImage(save=True,number=2)
                    if im1 is not None:
                        self.pixmap = QtGui.QPixmap(im1)
                        self.pixmap = self.pixmap.scaled(220,220)
                        self.view.setPixmap(self.pixmap)
                    if im2 is not None:
                        self.pixmap2 = QtGui.QPixmap(im2)
                        self.pixmap2 = self.pixmap2.scaled(220,220)
                        self.view2.setPixmap(self.pixmap2)

            else:
                print('Não pode ser alterado por não estar ligado ou Falta valor de passo no campo acima')
            self.pushButton_4.blockSignals(False)
            self.pushButton.blockSignals(False)
            self.pushButton_5.blockSignals(False)
            self.pushButton_3.blockSignals(False)

        self.pushButton_4.clicked.connect(right)

        # Up
        def Up():
            self.pushButton_5.blockSignals(True)
            self.pushButton_3.blockSignals(True)
            self.pushButton_4.blockSignals(True)
            self.pushButton.blockSignals(True)
            cod = self.lineEdit.text()
            try:
                old_height = control.getCurrentHeightHook()
                step = np.clip(float(cod),0.0,6.6747)
                status = control.commandUp(step)
            except:
                status = False 
            if status:
                desired_height = np.clip(old_height + step,0.0,6.6747)
                cnt = 0
                height = control.getCurrentHeightHook()
                while(abs(height - desired_height) > 0.05 and cnt < 200):
                    app.processEvents()
                    height = control.getCurrentHeightHook()
                    cnt+=1
                    im1 = control.getCamImage(save=True,number=1)
                    im2 = control.getCamImage(save=True,number=2)
                    if im1 is not None:
                        self.pixmap = QtGui.QPixmap(im1)
                        self.pixmap = self.pixmap.scaled(220,220)
                        self.view.setPixmap(self.pixmap)
                    if im2 is not None:
                        self.pixmap2 = QtGui.QPixmap(im2)
                        self.pixmap2 = self.pixmap2.scaled(220,220)
                        self.view2.setPixmap(self.pixmap2)
                    dist = control.getStatusDist()
                    self.lcdNumber.display(dist)
            else:
                print('Não pode ser alterado por não estar ligado ou Falta valor de passo no campo acima')
            time.sleep(0.5)
            dist = control.getStatusDist()
            self.lcdNumber.display(dist)
            self.pushButton_5.blockSignals(False)
            self.pushButton_3.blockSignals(False)
            self.pushButton_4.blockSignals(False)
            self.pushButton.blockSignals(False)

        self.pushButton_5.clicked.connect(Up)

        # Down
        def Down():
            self.pushButton_5.blockSignals(True)
            self.pushButton_3.blockSignals(True)
            self.pushButton_4.blockSignals(True)
            self.pushButton.blockSignals(True)
            cod = self.lineEdit.text()
            try:
                old_height = control.getCurrentHeightHook()
                step = np.clip(float(cod),0.0,6.6747)
                status = control.commandDown(step)
            except:
                status = False
            if status:
                desired_height = np.clip(old_height - step,0.0,6.6747)
                cnt = 0
                height = control.getCurrentHeightHook()
                while(abs(height - desired_height) > 0.05 and cnt < 200):
                    app.processEvents()
                    height = control.getCurrentHeightHook()
                    cnt+=1
                    im1 = control.getCamImage(save=True,number=1)
                    im2 = control.getCamImage(save=True,number=2)
                    if im1 is not None:
                        self.pixmap = QtGui.QPixmap(im1)
                        self.pixmap = self.pixmap.scaled(220,220)
                        self.view.setPixmap(self.pixmap)
                    if im2 is not None:
                        self.pixmap2 = QtGui.QPixmap(im2)
                        self.pixmap2 = self.pixmap2.scaled(220,220)
                        self.view2.setPixmap(self.pixmap2)
                    dist = control.getStatusDist()
                    self.lcdNumber.display(dist)
            else:
                print('Não pode ser alterado por não estar ligado ou Falta valor de passo no campo acima')
            time.sleep(0.5)
            dist = control.getStatusDist()
            self.lcdNumber.display(dist)
            self.pushButton_5.blockSignals(False)
            self.pushButton_3.blockSignals(False)
            self.pushButton_4.blockSignals(False)
            self.pushButton.blockSignals(False)

        self.pushButton_3.clicked.connect(Down)

        # Ima
        def magnet():
            if control.craneStatus and control.connectionStatus:
                status = control.getMagnetStatus()
                if status:
                    self.confirmWindow.exec_()
                    if self.confirmWindow.clickedButton() == self.buttonY:
                        control.commandMagnetOnOff()
                        print('Imã Desligado')
                        self.lcdNumber_6.display('Off')
                        self.lcdNumber_6.setPalette(self.off_color)
                        for _ in np.arange(0.0,control.getCurrentHeightHook(),0.25):
                            app.processEvents()
                            im1 = control.getCamImage(save=True,number=1)
                            im2 = control.getCamImage(save=True,number=2)
                            if im1 is not None:
                                self.pixmap = QtGui.QPixmap(im1)
                                self.pixmap = self.pixmap.scaled(220,220)
                                self.view.setPixmap(self.pixmap)
                            if im2 is not None:
                                self.pixmap2 = QtGui.QPixmap(im2)
                                self.pixmap2 = self.pixmap2.scaled(220,220)
                                self.view2.setPixmap(self.pixmap2)
                            dist = control.getStatusDist()
                            self.lcdNumber.display(dist)
                else:
                    control.commandMagnetOnOff()
                    print('Imã Ligado')
                    self.lcdNumber_6.display('UP')
                    self.lcdNumber_6.setPalette(self.on_color)
            else:
                print('Sem conexão ou guindaste desligado.')

        self.pushButton_6.clicked.connect(magnet)

        # On/Off Crane
        def crane():
            if control.connectionStatus:
                status = control.getCraneStatus()
                if status:
                    if control.getMagnetStatus():
                        self.confirmWindow_2.exec_()
                        print("Não é possível desligar o guindaste com o imã ainda ligado.")
                    else:
                        print('Desligado')
                        control.commandCraneOnOff()
                        self.view.clear()
                        self.view2.clear()
                        self.lcdNumber_5.display('OFF')
                        self.lcdNumber_5.setPalette(self.off_color)
                        self.lcdNumber_2.display(0)
                else:
                    print('Ligado')
                    control.commandCraneOnOff()
                    self.lcdNumber_5.display('UP')
                    self.lcdNumber_5.setPalette(self.on_color)
                    
                    angle = getAngle360(control.getCurrentAngleClaw())
                    self.lcdNumber_2.display(round(angle))
                
                    dist = control.getStatusDist()
                    self.lcdNumber.display(dist)
                    im1 = control.getCamImage(save=True,number=1)
                    im2 = control.getCamImage(save=True,number=2)
                    if im1 is not None:
                        self.pixmap = QtGui.QPixmap(im1)
                        self.pixmap = self.pixmap.scaled(220,220)
                        self.view.setPixmap(self.pixmap)
                    if im2 is not None:
                        self.pixmap2 = QtGui.QPixmap(im2)
                        self.pixmap2 = self.pixmap2.scaled(220,220)
                        self.view2.setPixmap(self.pixmap2)
            else:
                print('Sem conexão.')
        self.pushButton_2.clicked.connect(crane)

        self.retranslateUi(Guindaste)
        QtCore.QMetaObject.connectSlotsByName(Guindaste)

    def retranslateUi(self, Guindaste):
        Guindaste.setWindowTitle(_translate("Guindaste", "Laboratório de Projeto III - Grupo D", None))
        self.label.setText(_translate("Guindaste", "Sensor de Proximidade:", None))
        self.label_5.setText(_translate("Guindaste", "GUINDASTE TELEOPERADO", None))
        self.label_7.setText(_translate("Guindaste", "Ângulo da Lança [Graus]:", None))
        self.label_8.setText(_translate("Guindaste", "Estado do imã:", None))
        self.groupBox.setTitle(_translate("Guindaste", "Controle Direcional com Botões", None))
        self.label_9.setText(_translate("Guindaste", "Movimento por Clique:", None))
        self.pushButton_5.setText(_translate("Guindaste", "Subir\nGuincho", None))
        self.pushButton_4.setText(_translate("Guindaste", "Lança p/\nDireita", None))
        self.pushButton_3.setText(_translate("Guindaste", "Baixar\nGuincho", None))
        self.pushButton.setText(_translate("Guindaste", "Lança p/\nEsquerda", None))
        self.groupBox_3.setTitle(_translate("Guindaste", "Controle do Imã", None))
        self.pushButton_6.setText(_translate("Guindaste", "On/Off", None))
        self.groupBox_4.setTitle(_translate("Guindaste", "Sistema do Guindaste", None))
        self.groupBox_5.setTitle(_translate("Guindaste", "Sensores", None))
        self.pushButton_2.setText(_translate("Guindaste", "On/Off", None))
        self.pushButton_7.setText(_translate("Guindaste", "Conectar", None))
        self.label_10.setText(_translate("Guindaste", "Estado de Conexão:", None))
        self.label_12.setText(_translate("Guindaste", "Estado do Guindaste:", None))
        self.label_14.setText(_translate("Guindaste", "Câmera Gancho:", None))
        self.label_15.setText(_translate("Guindaste", "Câmera Braço:", None))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Guindaste = QtWidgets.QMainWindow()
    ui = Ui_Guindaste()
    ui.setupUi(Guindaste)
    Guindaste.show()
    sys.exit(app.exec_())
