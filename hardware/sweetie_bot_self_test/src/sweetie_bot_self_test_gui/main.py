import sys
from junitparser import JUnitXml
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QLabel,
    QMainWindow,
    QPushButton,
    QStackedLayout,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QWidget,
    QTextEdit,
    QSizePolicy,
)

class Widget(QWidget):
    log = ""
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)

        self.timer=QTimer()
        self.timer.timeout.connect(self.exit)

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.page_layout = QHBoxLayout(self)
        self.btn_layout = QVBoxLayout()
        self.grid_layout = QGridLayout()

        label = QLabel('Sweetie Bot self diagnostics test')
        label.setAlignment(Qt.AlignCenter)
        self.label_layout = QVBoxLayout()
        self.label_layout.addStretch()
        self.label_layout.addWidget(label)
        self.label_layout.addStretch()

        self.btn_layout.addStretch()
        self.btn_layout.addWidget(label)
        self.btn_layout.addLayout(self.grid_layout)
        self.btn_layout.addStretch()

        self.memo_layout = QVBoxLayout()
        self.memo_layout.addStretch()
        self.memo = QTextEdit(self)
        self.memo_layout.addWidget(self.memo)
        self.memo_layout.addStretch()

        self.page_layout.addStretch()
        self.page_layout.addLayout(self.memo_layout, 5)
        self.page_layout.addLayout(self.btn_layout, 5)
        self.page_layout.addStretch()

    def exit(self):
        sys.exit()

    def add_button(self, name, color):
        i = self.grid_layout.count()
        button = QPushButton(name)
        button.setStyleSheet(f"background-color: {color}");
        self.grid_layout.addWidget(button, 1 + i // 6, i % 6)

def main():
    app = QApplication(sys.argv)
    gui = Widget()
    xml = JUnitXml.fromfile('/tmp/result.xml')
    for suite in xml:
        # handle suites
        for case in suite:
            n = case.name
            name = n[n.find("[")+1:n.rfind("]")]
            gui.add_button(name, "green" if case.is_passed else "red" )
            if not case.system_out == None:
                gui.memo.append(case.system_out) 

    gui.show()
    gui.timer.start(1000)
    sys.exit(app.exec_())

