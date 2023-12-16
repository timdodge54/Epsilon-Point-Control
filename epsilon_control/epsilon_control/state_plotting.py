#!/bin/usr/env python3

import numpy as np
import threading
import numpy.typing as npt
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from controller_msgs.msg import StatePlotting


class PlottingNode(Node):
    """Node for plotting state data."""

    def __init__(self):
        super().__init__('StatePlotter')  # type: ignore
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        # Initial arrow, you can customize the length and color
        # create Thread lock to prevent multiaccess threading errors
        self._lock = threading.Lock()
        self._lock2 = threading.Lock()
        # create initial values to plot
        self.states: npt.NDArray[np.float64]
        self.t_span: npt.NDArray[np.float64]
        self.haveState = False
        # create subscriber
        self.state_sub: Subscription = self.create_subscription(
            StatePlotting, 'state_plotting', self._callback, 10)
        self.t = 0.0

    def _callback(self, msg: StatePlotting):
        """Callback for subscriber

        Args:
            msg: message from subscriber
                Message format
                ----------------
                int32 num
        """
        # lock thread
        with self._lock:
            # update values
            states = []
            for i in range(len(msg.tspan)):
                state = np.array(
                    [msg.x[i], msg.y[i], msg.phi[i], msg.w_r[i], msg.w_l[i]])
                states.append(state)
            self.states = np.array(states)
            self.t_span = np.array(msg.tspan)
            self.ax.clear()  # type: ignore
            plt.Arrow(0, 0, 1, 0, color='r', width=3)
            self.haveState = True

    def plt_func(self, frame):
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.

        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            self.ax.clear()  # type: ignore
            self.ax.set_xlim(-75, 75)
            self.ax.set_ylim(-75, 75)
            state = self.states[frame][0:3]
            dx, dy = 4*np.cos(state[2]), 4*np.sin(state[2])
            arrow = plt.Arrow(state[0], state[1], dx, dy, color='r', width=5)
            self.ax.add_patch(arrow)
            self.t += 0.01
            self.get_logger().info(f'{self.t}')
            # self.get_logger().info(f'{state}')
            # self.get_logger().info(f'{dx} {dy}')
            return self.ax,

    def plot_poller(self):
        while True:
            if self.haveState:
                self._plt()

    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = FuncAnimation(
            self.fig, self.plt_func, frames=len(self.t_span), blit=True, interval=1)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = PlottingNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node.plot_poller()


if __name__ == "__main__":
    main()
