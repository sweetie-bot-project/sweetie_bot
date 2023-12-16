# -*- coding: utf-8 -*-
import pytest
import warnings

import os
import sys
import math

from PyQt5.QtCore import QPointF, QRect, QRectF, Qt, QTimer
from PyQt5.QtGui import (QBrush, QColor, QFont, QLinearGradient, QPainter,
        QPen, QSurfaceFormat)
from PyQt5.QtWidgets import (QApplication, QGridLayout, QLabel, QOpenGLWidget,
        QWidget)

class Helper(object):
    def __init__(self):
        gradient = QLinearGradient(QPointF(50, -20), QPointF(80, 20))
        gradient.setColorAt(0.0, Qt.white)
        gradient.setColorAt(1.0, QColor(0xa6, 0xce, 0x39))

        self.background = QBrush(QColor(64, 32, 64))
        self.circleBrush = QBrush(gradient)
        self.circlePen = QPen(Qt.black)
        self.circlePen.setWidth(1)
        self.textPen = QPen(Qt.white)
        self.textFont = QFont()
        self.textFont.setPixelSize(50)

    def paint(self, painter, event, elapsed):
        painter.fillRect(event.rect(), self.background)
        painter.translate(400, 400)

        painter.save()
        painter.setBrush(self.circleBrush)
        painter.setPen(self.circlePen)
        painter.rotate(elapsed * 0.030)

        r = elapsed / 1000.0
        n = 30
        for i in range(n):
            painter.rotate(30)
            radius = 0 + 120.0*((i+r)/n)
            circleRadius = 1 + ((i+r)/n)*20
            painter.drawEllipse(QRectF(radius, -circleRadius,
                    circleRadius*2, circleRadius*2))

        painter.restore()

        painter.setPen(self.textPen)
        painter.setFont(self.textFont)
        painter.drawText(QRect(-50, -50, 100, 100), Qt.AlignCenter, "Qt")


class Widget(QWidget):
    def __init__(self, helper, parent):
        super(Widget, self).__init__(parent)

        self.helper = helper
        self.elapsed = 0
        self.setFixedSize(800, 800)

    def animate(self):
        self.elapsed = (self.elapsed + self.sender().interval()) % 1000
        self.repaint()

    def paintEvent(self, event):
        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)
        self.helper.paint(painter, event, self.elapsed)
        painter.end()


class GLWidget(QOpenGLWidget):
    def __init__(self, helper, parent):
        super(GLWidget, self).__init__(parent)

        self.helper = helper
        self.elapsed = 0
        self.setFixedSize(800, 800)
        self.setAutoFillBackground(False)

    def animate(self):
        self.elapsed = (self.elapsed + self.sender().interval()) % 1000
        self.update()

    def paintEvent(self, event):
        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)
        self.helper.paint(painter, event, self.elapsed)
        painter.end()


class Window(QWidget):
    def __init__(self):
        super(Window, self).__init__()

        self.setWindowTitle("2D Painting on Native and OpenGL Widgets")
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)

        helper = Helper()
        native = Widget(helper, self)
        openGL = GLWidget(helper, self)
        nativeLabel = QLabel("Native")
        nativeLabel.setAlignment(Qt.AlignHCenter)
        openGLLabel = QLabel("OpenGL")
        openGLLabel.setAlignment(Qt.AlignHCenter)

        layout = QGridLayout()
        layout.addWidget(native, 0, 0)
        layout.addWidget(openGL, 0, 1)
        layout.addWidget(nativeLabel, 1, 0)
        layout.addWidget(openGLLabel, 1, 1)
        self.setLayout(layout)

        timer = QTimer(self)
        timer.timeout.connect(native.animate)
        timer.timeout.connect(openGL.animate)
        timer.start(50)

class TestGPU:
    @pytest.mark.parametrize(
        ("timeout"),
        [
            (1000)
        ],
        ids=['egl']
    )
    def test_egl(self, timeout):
        os.environ["QT_QPA_PLATFORM"] = "eglfs"
        os.environ["QT_QPA_EGLFS_INTEGRATION"] = "eglfs_kms"
        os.environ["QT_QPA_EGLFS_ALWAYS_SET_MODE"] = "1"
        #os.environ["QT_QPA_EGLFS_KMS_CONFIG"] = "./eglfs.json"
        #os.environ["QT_LOGGING_RULES"] = "qt.*=true"
        #os.environ["QSG_INFO"] = "1"
        #os.environ["QT_QPA_EGLFS_DEBUG"] = "1"
        #os.environ["QT_DEBUG_PLUGINS"] = "1"

        app = QApplication(sys.argv)

        fmt = QSurfaceFormat()
        fmt.setSamples(4)
        QSurfaceFormat.setDefaultFormat(fmt)

        window = Window()
        window.show()
        QTimer.singleShot(timeout, window.close)
        try:
            app.exec_()
        except:
            assert False
        return True
