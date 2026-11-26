"""P7 operator GUI: launch the stack, start/finish exploration, mission timer, flight heatmap."""

import os
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure
from mpl_toolkits.axes_grid1 import make_axes_locatable
from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSplitter, QTabWidget,
    QVBoxLayout, QWidget,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from . import heatmap, topdown
from .ros_node import GuiNode

STATE_COLORS = {
    'WAITING': '#7f7f7f',
    'COMPUTE': '#1f77b4',
    'SELECT_GOAL': '#9467bd',
    'NAVIGATING': '#2ca02c',
    'REPLAN': '#ff7f0e',
    'GOAL_REACHED': '#17becf',
    'FINISHED': '#d62728',
}
STATE_STALE_SECONDS = 2.0
ODOM_STALE_SECONDS = 1.0


def format_duration(seconds):
    if seconds is None:
        return '--:--:--.-'
    hours, rest = divmod(seconds, 3600.0)
    minutes, secs = divmod(rest, 60.0)
    return f'{int(hours):02d}:{int(minutes):02d}:{secs:04.1f}'


class MainWindow(QMainWindow):
    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.process = None
        self.auto_exported = False
        self.last_state = None
        self.last_map = None  # newest consistent top-down map (image and grid can arrive apart)
        self.setWindowTitle('P7 drone - mission control')
        self.resize(1400, 900)

        splitter = QSplitter(Qt.Horizontal)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._build_controls())
        scroll.setMinimumWidth(480)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        splitter.addWidget(scroll)
        splitter.addWidget(self._build_tabs())
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        self.fast_timer = QTimer(self)
        self.fast_timer.timeout.connect(self.refresh_status)
        self.fast_timer.start(100)
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.plot_timer.timeout.connect(self.refresh_map)
        self.plot_timer.timeout.connect(self.refresh_coverage)
        self.plot_timer.start(1000)
        self.refresh_plot()

    # ------------------------------------------------------------------ layout
    def _build_controls(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)

        system = QGroupBox('System')
        form = QVBoxLayout(system)
        self.sim_time_box = QCheckBox('use_sim_time (timer follows /clock)')
        self.sim_time_box.setChecked(self.node.use_sim_time())
        self.sim_time_box.toggled.connect(self.on_sim_time_toggled)
        form.addWidget(self.sim_time_box)
        row = QHBoxLayout()
        self.launch_button = QPushButton(f'Launch {self.node.launch_file}')
        self.launch_button.clicked.connect(self.launch_stack)
        self.stop_stack_button = QPushButton('Stop stack')
        self.stop_stack_button.clicked.connect(self.stop_stack)
        row.addWidget(self.launch_button)
        row.addWidget(self.stop_stack_button)
        form.addLayout(row)
        self.stack_label = QLabel()
        form.addWidget(self.stack_label)
        layout.addWidget(system)

        exploration = QGroupBox('Frontier exploration')
        form = QVBoxLayout(exploration)
        self.state_label = QLabel('no data')
        self.state_label.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setPointSize(18)
        font.setBold(True)
        self.state_label.setFont(font)
        self.state_label.setMinimumHeight(48)
        form.addWidget(self.state_label)
        self.state_info = QLabel()
        self.state_info.setWordWrap(True)
        form.addWidget(self.state_info)
        row = QHBoxLayout()
        self.start_button = QPushButton('Start exploration')
        self.start_button.setStyleSheet('font-weight: bold;')
        self.start_button.clicked.connect(self.node.call_start)
        self.finish_button = QPushButton('Finish (hold)')
        self.finish_button.clicked.connect(self.on_finish)
        row.addWidget(self.start_button)
        row.addWidget(self.finish_button)
        form.addLayout(row)
        layout.addWidget(exploration)

        mission = QGroupBox('Mission')
        form = QFormLayout(mission)
        self.timer_label = QLabel(format_duration(None))
        font = QFont('Monospace')
        font.setStyleHint(QFont.Monospace)
        font.setPointSize(22)
        font.setBold(True)
        self.timer_label.setFont(font)
        form.addRow('Mission time', self.timer_label)
        self.wall_label = QLabel()
        form.addRow('Wall time', self.wall_label)
        self.rtf_label = QLabel()
        form.addRow('Real-time factor', self.rtf_label)
        self.goals_label = QLabel()
        form.addRow('Goals reached', self.goals_label)
        self.replans_label = QLabel()
        form.addRow('Replans', self.replans_label)
        self.batches_label = QLabel()
        form.addRow('Frontier batches', self.batches_label)
        self.viewpoints_label = QLabel()
        form.addRow('Viewpoints (last batch)', self.viewpoints_label)
        layout.addWidget(mission)

        flight = QGroupBox('Flight')
        form = QFormLayout(flight)
        self.position_label = QLabel()
        form.addRow('Position', self.position_label)
        self.yaw_label = QLabel()
        form.addRow('Yaw / speed', self.yaw_label)
        self.altitude_label = QLabel()
        form.addRow('Max altitude', self.altitude_label)
        self.distance_label = QLabel()
        form.addRow('Distance (xy / 3D)', self.distance_label)
        self.area_label = QLabel()
        form.addRow('Area flown over', self.area_label)
        self.odom_label = QLabel()
        self.odom_label.setWordWrap(True)
        form.addRow('Odometry', self.odom_label)
        layout.addWidget(flight)

        heat = QGroupBox('Heatmaps (grid, σ and scale also apply to camera coverage)')
        form = QVBoxLayout(heat)
        row = QHBoxLayout()
        row.addWidget(QLabel('Grid [m]'))
        self.resolution_box = QDoubleSpinBox()
        self.resolution_box.setRange(0.02, 2.0)
        self.resolution_box.setSingleStep(0.05)
        self.resolution_box.setValue(self.node.heatmap_resolution)
        self.resolution_box.setToolTip('Heatmap grid resolution (coarsened automatically for very large areas).')
        self.resolution_box.valueChanged.connect(self.refresh_plot)
        self.resolution_box.valueChanged.connect(lambda _: self.refresh_coverage())
        row.addWidget(self.resolution_box)
        row.addWidget(QLabel('Spread σ [m]'))
        self.sigma_box = QDoubleSpinBox()
        self.sigma_box.setRange(0.0, 10.0)
        self.sigma_box.setSingleStep(0.1)
        self.sigma_box.setValue(self.node.heatmap_sigma)
        self.sigma_box.setToolTip('Gaussian applied around every drone position.')
        self.sigma_box.valueChanged.connect(self.refresh_plot)
        self.sigma_box.valueChanged.connect(lambda _: self.refresh_coverage())
        row.addWidget(self.sigma_box)
        form.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel('Intensity'))
        self.weight_box = QComboBox()
        self.weight_box.addItem('time spent (hovering is brighter)', 'time')
        self.weight_box.addItem('path flown (even along the line)', 'path')
        self.weight_box.currentIndexChanged.connect(self.refresh_plot)
        row.addWidget(self.weight_box, 1)
        form.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel('Colour scale'))
        self.scale_box = QComboBox()
        self.scale_box.addItem('square root', 'sqrt')
        self.scale_box.addItem('linear', 'linear')
        self.scale_box.addItem('logarithmic', 'log')
        self.scale_box.currentIndexChanged.connect(self.refresh_plot)
        self.scale_box.currentIndexChanged.connect(lambda _: self.refresh_coverage())
        row.addWidget(self.scale_box, 1)
        form.addLayout(row)
        self.path_box = QCheckBox('Show flight path')
        self.path_box.setChecked(True)
        self.viewpoint_box = QCheckBox('Show frontier viewpoints')
        self.viewpoint_box.setChecked(True)
        self.follow_box = QCheckBox('Show drone pose')
        self.follow_box.setChecked(True)
        self.autofit_box = QCheckBox('Auto-fit view (untick to keep your zoom)')
        self.autofit_box.setChecked(True)
        self.mission_only_box = QCheckBox('Record only while mission timer runs')
        self.mission_only_box.toggled.connect(self.on_mission_only_toggled)
        self.auto_export_box = QCheckBox('Auto-export PNGs + CSV when FINISHED')
        self.auto_export_box.setChecked(True)
        self.follow_box.toggled.connect(lambda _: (self.refresh_map(), self.refresh_coverage()))
        for box in (self.path_box, self.viewpoint_box, self.follow_box):
            box.toggled.connect(self.refresh_plot)
        for box in (self.path_box, self.viewpoint_box, self.follow_box, self.autofit_box,
                    self.mission_only_box, self.auto_export_box):
            form.addWidget(box)
        row = QHBoxLayout()
        export_png = QPushButton('Export PNG...')
        export_png.clicked.connect(self.export_png_dialog)
        export_csv = QPushButton('Export CSV...')
        export_csv.clicked.connect(self.export_csv_dialog)
        reset = QPushButton('Reset')
        reset.clicked.connect(self.reset_heatmap)
        row.addWidget(export_png)
        row.addWidget(export_csv)
        row.addWidget(reset)
        form.addLayout(row)
        layout.addWidget(heat)

        explored = QGroupBox('Explored map (camera, from octomap)')
        form = QFormLayout(explored)
        self.explored_label = QLabel('-')
        form.addRow('Seen by camera', self.explored_label)
        self.floor_label = QLabel('-')
        form.addRow('Floor observed', self.floor_label)
        self.trees_label = QLabel('-')
        form.addRow('Trees found', self.trees_label)
        self.map_age_label = QLabel('-')
        self.map_age_label.setWordWrap(True)
        form.addRow('Map', self.map_age_label)
        self.map_trees_box = QCheckBox('Mark trees')
        self.map_trees_box.setChecked(True)
        self.map_path_box = QCheckBox('Show flight path')
        self.map_path_box.setChecked(True)
        for box in (self.map_trees_box, self.map_path_box):
            box.toggled.connect(self.refresh_map)
            form.addRow(box)
        export_map = QPushButton('Export map PNG...')
        export_map.clicked.connect(self.export_map_dialog)
        form.addRow(export_map)
        layout.addWidget(explored)

        coverage = QGroupBox('Camera coverage (camera-filtered LiDAR cloud)')
        form = QFormLayout(coverage)
        self.coverage_area_label = QLabel('-')
        form.addRow('In camera view', self.coverage_area_label)
        self.coverage_threshold_box = QDoubleSpinBox()
        self.coverage_threshold_box.setRange(0.0, 600.0)
        self.coverage_threshold_box.setSingleStep(0.5)
        self.coverage_threshold_box.setValue(1.0)
        self.coverage_threshold_box.setSuffix(' s')
        self.coverage_threshold_box.valueChanged.connect(lambda _: self.refresh_coverage())
        form.addRow('Seen at least', self.coverage_threshold_box)
        self.coverage_threshold_label = QLabel('-')
        form.addRow('Area seen that long', self.coverage_threshold_label)
        self.coverage_mean_label = QLabel('-')
        form.addRow('Time in view (mean / max)', self.coverage_mean_label)
        self.coverage_age_label = QLabel('-')
        self.coverage_age_label.setWordWrap(True)
        form.addRow('Coverage', self.coverage_age_label)
        self.coverage_path_box = QCheckBox('Show flight path')
        self.coverage_path_box.setChecked(True)
        self.coverage_path_box.toggled.connect(lambda _: self.refresh_coverage())
        form.addRow(self.coverage_path_box)
        row = QHBoxLayout()
        export_coverage = QPushButton('Export coverage PNG...')
        export_coverage.clicked.connect(self.export_coverage_dialog)
        reset_coverage = QPushButton('Reset coverage')
        reset_coverage.clicked.connect(self.on_reset_coverage)
        row.addWidget(export_coverage)
        row.addWidget(reset_coverage)
        form.addRow(row)
        layout.addWidget(coverage)
        layout.addStretch(1)
        return panel

    def _build_tabs(self):
        tabs = QTabWidget()

        plot_page = QWidget()
        plot_layout = QVBoxLayout(plot_page)
        self.figure = Figure(figsize=(8, 7), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.cax = make_axes_locatable(self.ax).append_axes('right', size='4%', pad=0.1)
        plot_layout.addWidget(NavigationToolbar2QT(self.canvas, self))
        plot_layout.addWidget(self.canvas)
        tabs.addTab(plot_page, 'Heatmap (x, y)')

        map_page = QWidget()
        map_layout = QVBoxLayout(map_page)
        self.map_figure = Figure(figsize=(8, 7), tight_layout=True)
        self.map_canvas = FigureCanvasQTAgg(self.map_figure)
        self.map_ax = self.map_figure.add_subplot(111)
        map_layout.addWidget(NavigationToolbar2QT(self.map_canvas, self))
        map_layout.addWidget(self.map_canvas)
        tabs.addTab(map_page, 'Explored map (top-down)')

        coverage_page = QWidget()
        coverage_layout = QVBoxLayout(coverage_page)
        self.coverage_figure = Figure(figsize=(8, 7), tight_layout=True)
        self.coverage_canvas = FigureCanvasQTAgg(self.coverage_figure)
        self.coverage_ax = self.coverage_figure.add_subplot(111)
        self.coverage_cax = make_axes_locatable(self.coverage_ax).append_axes('right', size='4%', pad=0.1)
        coverage_layout.addWidget(NavigationToolbar2QT(self.coverage_canvas, self))
        coverage_layout.addWidget(self.coverage_canvas)
        tabs.addTab(coverage_page, 'Camera coverage')
        self.tabs = tabs
        tabs.currentChanged.connect(lambda _: (self.refresh_plot(), self.refresh_map(), self.refresh_coverage()))

        self.history_list = QListWidget()
        tabs.addTab(self.history_list, 'State history')

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        self.log_view.setFont(QFont('Monospace', 9))
        tabs.addTab(self.log_view, 'Log')
        return tabs

    # ------------------------------------------------------------------ actions
    def on_sim_time_toggled(self, enabled):
        self.node.set_use_sim_time(enabled)
        self.append_log(f'use_sim_time set to {enabled}.')

    def on_mission_only_toggled(self, enabled):
        with self.node.lock:
            self.node.record_only_during_mission = enabled

    def on_finish(self):
        answer = QMessageBox.question(
            self, 'Finish exploration',
            'Finish frontier exploration? The selector enters FINISHED (terminal), commands a hold at the '
            'current pose and the mission timer stops.')
        if answer == QMessageBox.Yes:
            self.node.call_finish()

    def launch_stack(self):
        if self.process is not None:
            return
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_process_output)
        self.process.finished.connect(self.process_finished)
        arguments = ['launch', self.node.launch_package, self.node.launch_file,
                     f'use_sim_time:={"true" if self.sim_time_box.isChecked() else "false"}']
        self.append_log('$ ros2 ' + ' '.join(arguments))
        self.process.start('ros2', arguments)
        if not self.process.waitForStarted(3000):
            self.append_log('Failed to start ros2 launch: ' + self.process.errorString())
            self.process = None

    def stop_stack(self):
        if self.process is None:
            return
        self.append_log('Stopping stack (SIGINT)...')
        os.kill(int(self.process.processId()), signal.SIGINT)
        process = self.process
        QTimer.singleShot(15000, lambda: process.state() != QProcess.NotRunning and process.kill())

    def read_process_output(self):
        if self.process is None:
            return
        text = bytes(self.process.readAllStandardOutput()).decode(errors='replace').rstrip('\n')
        if text:
            self.log_view.appendPlainText(text)

    def process_finished(self, code, _status):
        self.append_log(f'Stack exited with code {code}.')
        self.process = None

    def reset_heatmap(self):
        self.node.reset_trajectory()
        self.refresh_plot()

    def append_log(self, text):
        self.log_view.appendPlainText(f'[{time.strftime("%H:%M:%S")}] {text}')

    # ------------------------------------------------------------------ export
    def summary(self):
        mission, _ = self.node.mission_elapsed()
        with self.node.lock:
            state = self.node.state or 'unknown'
            distance = self.node.distance_2d
            goals = self.node.goals_reached
        area = heatmap.covered_area(self.node.trajectory())
        clock = 'sim' if self.node.use_sim_time() else 'wall'
        return (f'{time.strftime("%Y-%m-%d %H:%M")} | state {state} | mission {format_duration(mission)} ({clock}) | '
                f'xy distance {distance:.1f} m | flown over {area:.0f} m² | goals {goals}')

    def export_png(self, path):
        figure = Figure(figsize=(10, 8.5), tight_layout=True)
        FigureCanvasAgg(figure)
        ax = figure.add_subplot(111)
        cax = make_axes_locatable(ax).append_axes('right', size='4%', pad=0.1)
        self.draw_into(figure, ax, cax, title=self.summary())
        figure.savefig(path, dpi=200)
        self.append_log(f'Heatmap exported to {path}')

    def export_map_png(self, path):
        map_data = self.current_map()
        figure = Figure(figsize=(10, 8.5), tight_layout=True)
        FigureCanvasAgg(figure)
        ax = figure.add_subplot(111)
        stats = topdown.statistics(map_data)
        title = self.summary()
        if stats:
            title += (f'\nseen by camera {stats["explored"]:.0f} m² | floor observed {stats["floor"]:.0f} m² | '
                      f'trees {stats["trees"]}')
        self.draw_map_into(ax, map_data, title=title)
        figure.savefig(path, dpi=200)
        self.append_log(f'Explored map exported to {path}')

    def coverage_snapshot(self):
        with self.node.lock:
            coverage = None if self.node.coverage is None else self.node.coverage.copy()
            frame, received = self.node.coverage_frame, self.node.coverage_received
        return coverage, frame, received

    def coverage_statistics(self, coverage):
        if coverage is None or coverage.shape[0] == 0:
            return None
        cell = heatmap.coverage_cell_size(coverage) ** 2
        seconds = coverage[:, 2]
        threshold = self.coverage_threshold_box.value()
        return {
            'area': coverage.shape[0] * cell,
            'area_threshold': float(np.count_nonzero(seconds >= threshold)) * cell,
            'threshold': threshold,
            'mean': float(seconds.mean()),
            'max': float(seconds.max()),
        }

    def draw_coverage_into(self, figure, ax, cax, title=None):
        coverage, frame, _ = self.coverage_snapshot()
        with self.node.lock:
            pose = self.node.pose
        heatmap.draw_coverage(
            figure, ax, cax, coverage, self.resolution_box.value(), self.sigma_box.value(),
            scale=self.scale_box.currentData(), trajectory=self.node.trajectory(),
            show_path=self.coverage_path_box.isChecked(), pose=pose if self.follow_box.isChecked() else None,
            title=title, frame=frame)

    def export_coverage_png(self, path):
        figure = Figure(figsize=(10, 8.5), tight_layout=True)
        FigureCanvasAgg(figure)
        ax = figure.add_subplot(111)
        cax = make_axes_locatable(ax).append_axes('right', size='4%', pad=0.1)
        title = self.summary()
        stats = self.coverage_statistics(self.coverage_snapshot()[0])
        if stats:
            title += (f'\nin camera view {stats["area"]:.0f} m² | seen ≥ {stats["threshold"]:.1f} s '
                      f'{stats["area_threshold"]:.0f} m² | mean {stats["mean"]:.1f} s')
        self.draw_coverage_into(figure, ax, cax, title=title)
        figure.savefig(path, dpi=200)
        self.append_log(f'Camera coverage exported to {path}')

    def export_coverage_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export camera coverage', self.default_name('png').replace(
            'p7_flight_', 'p7_coverage_'), 'PNG image (*.png)')
        if path:
            if not path.lower().endswith('.png'):
                path += '.png'
            self.export_coverage_png(path)

    def on_reset_coverage(self):
        answer = QMessageBox.question(self, 'Reset camera coverage',
                                      'Clear the accumulated camera coverage in topdown_map_node?')
        if answer == QMessageBox.Yes:
            self.node.call_reset_coverage()

    def export_csv(self, path):
        np.savetxt(path, self.node.trajectory(), delimiter=',', header='x,y,z,dt,stamp', comments='', fmt='%.4f')
        self.append_log(f'Trajectory exported to {path}')

    def default_name(self, extension):
        return f'p7_flight_{time.strftime("%Y%m%d_%H%M%S")}.{extension}'

    def export_png_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export heatmap', self.default_name('png'), 'PNG image (*.png)')
        if path:
            if not path.lower().endswith('.png'):
                path += '.png'
            self.export_png(path)

    def export_map_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export explored map', self.default_name('png').replace(
            'p7_flight_', 'p7_explored_'), 'PNG image (*.png)')
        if path:
            if not path.lower().endswith('.png'):
                path += '.png'
            self.export_map_png(path)

    def export_csv_dialog(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export trajectory', self.default_name('csv'), 'CSV (*.csv)')
        if path:
            self.export_csv(path)

    def auto_export(self):
        directory = os.path.join(os.getcwd(), 'p7_gui_exports')
        os.makedirs(directory, exist_ok=True)
        stem = os.path.join(directory, self.default_name('')[:-1])
        self.export_png(stem + '.png')
        self.export_csv(stem + '.csv')
        if self.current_map() is not None:
            self.export_map_png(stem.replace('p7_flight_', 'p7_explored_') + '.png')
        if self.coverage_snapshot()[0] is not None:
            self.export_coverage_png(stem.replace('p7_flight_', 'p7_coverage_') + '.png')

    # ------------------------------------------------------------------ refresh
    def draw_into(self, figure, ax, cax, title=None):
        with self.node.lock:
            pose = self.node.pose
            viewpoints = self.node.viewpoints.copy()
            frame = self.node.odom_frame
        heatmap.draw(
            figure, ax, cax, self.node.trajectory(), self.resolution_box.value(), self.sigma_box.value(),
            weighting=self.weight_box.currentData(), scale=self.scale_box.currentData(), show_path=self.path_box.isChecked(),
            pose=pose if self.follow_box.isChecked() else None,
            viewpoints=viewpoints if self.viewpoint_box.isChecked() else None,
            title=title, frame=frame)

    def current_map(self):
        map_data = self.node.topdown_map()
        if map_data is not None:
            self.last_map = map_data
        return self.last_map

    def draw_map_into(self, ax, map_data, title=None):
        with self.node.lock:
            pose = self.node.pose
        topdown.draw(ax, map_data, self.node.trajectory(), pose if self.follow_box.isChecked() else None,
                     show_path=self.map_path_box.isChecked(), show_trees=self.map_trees_box.isChecked(),
                     title=title, frame=map_data['frame'] if map_data else '')

    def refresh_map(self):
        map_data = self.current_map()
        stats = topdown.statistics(map_data)
        if stats:
            self.explored_label.setText(f'{stats["explored"]:.0f} m²')
            self.floor_label.setText(f'{stats["floor"]:.0f} m²')
            self.trees_label.setText(f'{stats["trees"]}  ({stats["trunk"]:.1f} m² trunk cells)')
            age = time.monotonic() - map_data['received']
            with self.node.lock:
                odom_frame = self.node.odom_frame
            warning = (f'  - frame {map_data["frame"]} differs from odometry {odom_frame}!'
                       if odom_frame and map_data['frame'] != odom_frame else '')
            self.map_age_label.setText(f'updated {age:.0f} s ago, {map_data["resolution"]:.2f} m cells{warning}')
        else:
            self.map_age_label.setText(f'no data on {self.node.topdown_prefix}/image + /grid')
        if self.tabs.currentIndex() != 1:
            return
        keep = self.map_canvas.toolbar is not None and self.map_canvas.toolbar.mode != ''
        keep = keep or not self.autofit_box.isChecked()
        limits = (self.map_ax.get_xlim(), self.map_ax.get_ylim()) if keep and self.map_ax.has_data() else None
        self.draw_map_into(self.map_ax, map_data)
        if limits:
            self.map_ax.set_xlim(limits[0])
            self.map_ax.set_ylim(limits[1])
        self.map_canvas.draw_idle()

    def refresh_coverage(self):
        if not hasattr(self, 'coverage_canvas'):
            return
        coverage, _, received = self.coverage_snapshot()
        stats = self.coverage_statistics(coverage)
        if stats:
            self.coverage_area_label.setText(f'{stats["area"]:.0f} m²')
            self.coverage_threshold_label.setText(f'{stats["area_threshold"]:.0f} m²')
            self.coverage_mean_label.setText(f'{stats["mean"]:.1f} s / {stats["max"]:.1f} s')
            self.coverage_age_label.setText(f'updated {time.monotonic() - received:.0f} s ago, '
                                            f'{coverage.shape[0]} cells')
        else:
            self.coverage_age_label.setText(f'no data on {self.node.topdown_prefix}/coverage')
        if self.tabs.currentIndex() != 2:
            return
        keep = self.coverage_canvas.toolbar is not None and self.coverage_canvas.toolbar.mode != ''
        keep = keep or not self.autofit_box.isChecked()
        ax = self.coverage_ax
        limits = (ax.get_xlim(), ax.get_ylim()) if keep and ax.has_data() else None
        self.draw_coverage_into(self.coverage_figure, ax, self.coverage_cax)
        if limits:
            ax.set_xlim(limits[0])
            ax.set_ylim(limits[1])
        self.coverage_canvas.draw_idle()

    def refresh_plot(self):
        if hasattr(self, 'tabs') and self.tabs.currentIndex() != 0:
            self.area_label.setText(f'{heatmap.covered_area(self.node.trajectory()):.0f} m² (1 m cells)'
                                    f'  ({len(self.node.samples)} samples)')
            return
        # Keep the user's zoom/pan while a toolbar tool is active or auto-fit is off.
        toolbar_active = self.canvas.toolbar is not None and self.canvas.toolbar.mode != ''
        keep = toolbar_active or not self.autofit_box.isChecked()
        limits = (self.ax.get_xlim(), self.ax.get_ylim()) if keep and self.ax.has_data() else None
        self.draw_into(self.figure, self.ax, self.cax)
        self.area_label.setText(f'{heatmap.covered_area(self.node.trajectory()):.0f} m² (1 m cells)'
                                f'  ({len(self.node.samples)} samples)')
        if limits:
            self.ax.set_xlim(limits[0])
            self.ax.set_ylim(limits[1])
        self.canvas.draw_idle()

    def refresh_status(self):
        node = self.node
        for message in node.take_messages():
            self.log_view.appendPlainText(message)
        now = time.monotonic()
        with node.lock:
            state = node.state
            state_age = None if node.state_received is None else now - node.state_received
            history = list(node.state_history)
            goals, replans, batches = node.goals_reached, node.replans, len(node.batch_ids)
            n_viewpoints = len(node.viewpoints)
            pose, speed = node.pose, node.speed
            odom_age = None if node.odom_received is None else now - node.odom_received
            max_altitude = node.max_altitude
            d2, d3 = node.distance_2d, node.distance_3d
            pending = node.mission_start_pending

        stale = state_age is None or state_age > STATE_STALE_SECONDS
        if state is None or stale:
            text = 'no data' if state is None else f'{state} (stale)'
            self.state_label.setText(text)
            self.state_label.setStyleSheet('color: white; background: #444444; border-radius: 6px;')
        else:
            self.state_label.setText(state)
            self.state_label.setStyleSheet(
                f'color: white; background: {STATE_COLORS.get(state, "#444444")}; border-radius: 6px;')
        self.state_info.setText(
            f'topic {node.state_topic}' + ('' if state_age is None else f'  (last {state_age:.1f} s ago)'))

        running = node.mission_running()
        self.start_button.setEnabled(not running and (state in (None, 'WAITING') or stale))
        self.finish_button.setEnabled(state not in (None, 'FINISHED') and not stale)
        self.sim_time_box.setEnabled(not running)
        self.launch_button.setEnabled(self.process is None)
        self.stop_stack_button.setEnabled(self.process is not None)
        self.stack_label.setText('stack: running (launched from GUI)' if self.process is not None
                                 else 'stack: not launched from GUI')

        mission, wall = node.mission_elapsed()
        if pending:
            self.timer_label.setText('waiting /clock')
        else:
            self.timer_label.setText(format_duration(mission))
        self.timer_label.setStyleSheet('color: #2ca02c;' if running else ('color: #d62728;' if mission else ''))
        self.wall_label.setText(format_duration(wall))
        self.rtf_label.setText(f'{mission / wall:.2f}' if mission and wall and wall > 1.0 else '-')
        self.goals_label.setText(str(goals))
        self.replans_label.setText(str(replans))
        self.batches_label.setText(str(batches))
        self.viewpoints_label.setText(str(n_viewpoints))

        if pose is None:
            self.position_label.setText('-')
            self.yaw_label.setText('-')
        else:
            self.position_label.setText(f'x {pose[0]:7.2f}  y {pose[1]:7.2f}  z {pose[2]:6.2f} m')
            self.yaw_label.setText(f'{np.degrees(pose[3]):6.1f} deg   {speed:4.2f} m/s')
        self.altitude_label.setText('-' if max_altitude == -np.inf else f'{max_altitude:.2f} m')
        self.distance_label.setText(f'{d2:.1f} m / {d3:.1f} m')
        if odom_age is None:
            self.odom_label.setText(f'no data on {node.odom_topic}')
            self.odom_label.setStyleSheet('color: #d62728;')
        else:
            self.odom_label.setText(f'{node.odom_topic}  ({odom_age:.1f} s ago)')
            self.odom_label.setStyleSheet('color: #d62728;' if odom_age > ODOM_STALE_SECONDS else '')

        if len(history) != self.history_list.count():
            self.history_list.clear()
            for mission_time, clock, entry in history:
                stamp = format_duration(mission_time) if mission_time is not None else '--'
                self.history_list.addItem(f'{clock}   T+{stamp}   {entry}')
            self.history_list.scrollToBottom()

        if state != self.last_state:
            if state == 'FINISHED' and self.auto_export_box.isChecked() and not self.auto_exported:
                self.auto_exported = True
                self.auto_export()
            elif state != 'FINISHED':
                self.auto_exported = False
            self.last_state = state

    def closeEvent(self, event):
        if self.process is not None:
            answer = QMessageBox.question(self, 'Quit', 'The stack launched from this GUI is still running. '
                                                        'Stop it and quit?')
            if answer != QMessageBox.Yes:
                event.ignore()
                return
            os.kill(int(self.process.processId()), signal.SIGINT)
            self.process.waitForFinished(15000)
            if self.process is not None and self.process.state() != QProcess.NotRunning:
                self.process.kill()
        event.accept()


def main():
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = GuiNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(remove_ros_args(sys.argv))
    window = MainWindow(node)
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    # Let the Python interpreter run periodically so Ctrl+C is handled.
    keep_alive = QTimer()
    keep_alive.timeout.connect(lambda: None)
    keep_alive.start(200)
    code = app.exec_()

    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
