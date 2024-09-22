import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import pyqtgraph as pg

import rospy
from sweetie_bot_plot.msg import Plot, Curve, Subplot


class PlotNodeWindow(QMainWindow):
    _new_msg_signal = pyqtSignal(Plot)
    
    def __init__(self):
        super(PlotNodeWindow, self).__init__()
        # init ROS node
        rospy.init_node('plot_node')
        self._plot_sub = rospy.Subscriber('plot', Plot, self.on_plot)
        rospy.on_shutdown(self.close)
        # create plot widget
        self._curves = {}
        self._subplots = {}
        self._plot_layout = pg.GraphicsLayoutWidget()
        self.setCentralWidget(self._plot_layout)
        # plot layout style
        self._plot_layout.setBackground("w")
        # communication
        self._new_msg_signal.connect(self.plot)

    #
    # ovwerride
    #
    def closeEvent(self, event):
        rospy.signal_shutdown('close')
        event.accept()

    #
    # message handler
    #
    def on_plot(self, plot_msg):
        self._new_msg_signal.emit(plot_msg)

    #
    # plot hook
    #
    def plot(self, plot_msg):
        self.setWindowTitle(plot_msg.title)
        if plot_msg.action == Plot.CREATE:
            # recreate plots
            self._update_layout(plot_msg)
        elif plot_msg.action == Plot.UPDATE:
            # attempt to update exxicting plots
            try:
                self._update(plot_msg)
            except Exception as e:
                rospy.logwarn('Update action has failed: %s\n.', e)
                rospy.logwarn('Attempt to recreate plot layout.')
                # attemt recreate layout
                self._update_layout(plot_msg)

    def _update_layout(self, plot_msg):
        # clear axes and plots
        self._plot_layout.clear()
        self._subplots.clear()
        self._curves.clear()
        # construct plots
        row = 0
        for subplot_msg in plot_msg.subplots:
            # check correctness
            if subplot_msg.title in self._subplots:
                rospy.logerr(f'Dublicate subplot name {subplot_msg.title}. Subplot is skipped.')
                continue
            if len(subplot_msg.curves) == 0:
                rospy.logerr(f'Empty subplot {subplot_msg.title}.')
                continue
            # create subplot
            subplot = self._plot_layout.addPlot(row = row, col = 0)
            row += 1
            # configure subplot
            subplot.setTitle(subplot_msg.title, style = 'k')
            subplot.setLabel("bottom", subplot_msg.xlabel, style = 'k')
            subplot.setLabel("left", subplot_msg.ylabel, style = 'k')
            subplot.addLegend(labelTextColor = 'k')
            subplot.showGrid(x=True, y=True)
            if len(subplot_msg.xlim) == 2:
                subplot.setXRange(subplot_msg.xlim[0], subplot_msg.xlim[1])
            elif len(subplot_msg.xlim) != 0:
                rospy.logerr(f'Incorrect xlim field size for subplot "{subplot_msg.title}". xlim field must be empty or contain two elements.')
            if len(subplot_msg.ylim) == 2:
                subplot.setYRange(subplot_msg.ylim[0], subplot_msg.ylim[1])
            elif len(subplot_msg.ylim) != 0:
                rospy.logerr(f'Incorrect xlim field size for subplot "{subplot_msg.title}". ylim field must be empty or contain two elements.')
            # add curves
            colors = ['b', 'r', 'g', 'm']
            for curve_msg in subplot_msg.curves:
                # curve name
                if len(curve_msg.name) > 0:
                    name = curve_msg.name
                    full_name = subplot_msg.title + ' ' + name
                else:
                    name = None
                    full_name = subplot_msg.title
                if full_name in self._curves:
                    rospy.logerr(f'Incorrect curve {curve_msg.name} on  subplot "{subplot_msg.title}": dublicate name.')
                    continue
                # curve type
                if curve_msg.type not in (Curve.LINE, Curve.HISTOGRAM, Curve.SCATTER):
                    rospy.logerr(f'Unknovwn subplot {subplot_msg.title} type: {subplot_msg.type}.')
                    continue
                if curve_msg.type != Curve.HISTOGRAM:
                    expected_x_len = lambda y: len(y) 
                else:
                    expected_x_len = lambda y: len(y)+1
                # curve color
                color = curve_msg.color if len(curve_msg.color) != 0 else colors[0]
                if color not in colors:
                    rospy.logerr(f'Incorrect curve {curve_msg.name} on  subplot "{subplot_msg.title}": Unknow or dubplicate color {curve_msg.color}.')
                    continue
                # curve data 
                if ((len(curve_msg.x) != expected_x_len(curve_msg.y)) and len(curve_msg.x) != 0) or len(curve_msg.y) == 0:
                    rospy.logerr(f'Incorrect curve {curve_msg.name} on  subplot "{subplot_msg.title}": The size of x field must be same as th size of y field or zero. y must not be empty: len(x) = {len(curve_msg.x)},  len(y) = {len(curve_msg.y)},  expected_len(x) = {expected_x_len(curve_msg.y)}')
                    continue
                if len(curve_msg.x) != 0:
                    data = [ curve_msg.x, curve_msg.y ]
                else:
                    data = [ curve_msg.y ]
                # plotting
                if curve_msg.type == Curve.LINE:
                    # draw line
                    self._curves[full_name] = subplot.plot(*data, name = name, pen = pg.mkPen(color))
                elif curve_msg.type == Curve.SCATTER:
                    # add points 
                    self._curves[full_name] = subplot.plot(*data, name = name, pen = None, symbolBrush = pg.mkBrush(color), symbol = 'o')
                elif curve_msg.type == Curve.HISTOGRAM:
                    self._curves[full_name] = subplot.plot(*data, stepMode=True, fillLevel=0, brush = pg.mkBrush(color))
                # remove color
                colors.remove(color)

    def _update(self, plot_msg):
        ''' REturn False on layout mismatch error. '''
        for subplot_msg in plot_msg.subplots:
            for curve_msg in subplot_msg.curves:
                # get name 
                if len(curve_msg.name) != 0:
                    full_name = subplot_msg.title + ' ' + curve_msg.name
                else:
                    full_name = subplot_msg.title
                # get curve 
                curve = self._curves.get(full_name)
                if curve is None:
                    raise RuntimeError(f'Update action failed: "{curve_msg.name}" on subplot "{subplot_msg.title}" is not present in layout.')
                # update plot
                if len(curve_msg.x) != 0:
                    curve.setData(curve_msg.x, curve_msg.y)
                else:
                    curve.setData(curve_msg.y)

def main():
    # init PyQt application
    app = QApplication([])
    plot_window = PlotNodeWindow()
    plot_window.show()
    app.exec()

