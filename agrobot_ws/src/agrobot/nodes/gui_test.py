import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QLabel, QPlainTextEdit
from PyQt5.QtCore import Qt

width = 500
height = 500
class ControlWindow(QMainWindow):

    def __init__(self):
        super().__init__()

        # mySlider = QSlider(Qt.Horizontal, self)
        # mySlider.setGeometry(30, 40, 200, 30)
        # mySlider.valueChanged[int].connect(self.changeValue)
        self.setGeometry(50,50,width,height)
        self.setWindowTitle("Parameter Tuning")
        self.slider_pos = 40
    
    def add_slider(self, name, low, high, inc):

        label = QLabel(self)
        label.setText(name)
        label.setGeometry(30, self.slider_pos, width-50, 30)
        label.move(10,self.slider_pos-30)
        # label.show()
        print('here', self.slider_pos)
        slider = QSlider(Qt.Horizontal, self)
        slider.setGeometry(30, self.slider_pos, width-50, 30)
        slider.valueChanged[int].connect(self.changeValue)
        slider.setMinimum(low)
        slider.setMaximum(high)
        slider.setTickInterval(inc)
        slider.setTickPosition(2)
        self.slider_pos+= 80

        name_label = QLabel(self)
        # name_label.setGeometry(30, self.slider_pos, width-50, 30)
        name_label.setText('1')
        name_label.move(1, self.slider_pos)
        

    def changeValue(self, value):
        # label.setText(str(value))

        print(value)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    cw = ControlWindow()
    cw.add_slider('parameter 1',low =0,high=100,inc=1)
    cw.add_slider('the other parameter',low=0,high=50,inc=1)
    cw.add_slider('the third parameter',low=50,high=1000,inc=1)
    cw.show()
    # print(cw.get)
    sys.exit(app.exec_())