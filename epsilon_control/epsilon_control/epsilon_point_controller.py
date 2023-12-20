#!/bin/usr/env python3

"""
MIT License

Copyright (c) [2023] [Timothy Dodge]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


import numpy as np
import numpy.typing as npt
import control as ct
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from controller_msgs.msg import StatePlotting
from geometry_msgs.msg import Twist


class EpsilonPointController(Node):
    """Class that to calculate approximate diffomorphism epsilon point control.

    Utilizes velocity control state space with the following state:
    [x, y, phi, w_r, w_l].T
    with the dynamics: [x_dot, y_dot, phi_dot, u_r, u_l].T

    Attributes:
        A: feedback linearized state matrix
        B: feedback linearized input matrix
        r: wheel radius
        L: wheelbase
        t_span: time span for odeint
        epsilon: epsilon value for epsilon point control
    """

    def __init__(
            self,
            t_f: float,
            epsilon,
            r: float,
            L: float,
            x_0: npt.NDArray[np.float64],
            plot_sim: bool = False
    ) -> None:
        """Initialize.

        Args:
            t_f: final time for simulation
            epsilon: epsilon value for epsilon point control
            r: wheel radius
            L: wheelbase
            x_0: initial state vector
        """
        super().__init__("epsilon_point_controller")  # type: ignore
        self.state_pub = self.create_publisher(
            StatePlotting, "state_plotting", 10)
        # defined feedback linearized state space matrices
        self.A = np.array([[0, 0, 1, 0],
                           [0, 0, 0, 1],
                           [0, 0, 0, 0],
                           [0, 0, 0, 0]])
        self.B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])
        # define LQR gain matrices
        Q = np.diag([1, 1, 1/(1.86**2), 1/(.26)])
        R = np.eye(2)
        self.r = r
        self.L = L
        # calculate LQR gain matrix
        self.k = ct.lqr(self.A, self.B, Q, R)[0]
        # define the time span for the simulation
        self.t_span = np.arange(0, t_f, 0.01)
        # define epsilon value
        self.epsilon = epsilon
        # the control history found after the simulation
        self.controls_history: list[tuple[float, float]] = []
        # call the simulation to calculate all states over the entire time span
        sol = self.ode_int_wrapper(x_0)
        # get the controls input
        controls = self.get_all_control_inputs(sol)
        # toggle between plotting the simulation or sending the commands to 
        # the robot
        if plot_sim:
            # create the plotting message and send to the plotter
            states = StatePlotting()
            states.tspan = self.t_span.tolist()
            states.x = sol.T[0, :].tolist()
            states.y = sol.T[1, :].tolist()
            states.phi = sol.T[2, :].tolist()
            states.w_r = sol.T[3, :].tolist()
            states.w_l = sol.T[4, :].tolist()
            self.state_pub.publish(states)
        else:
            # calculate the velocity control inputs over the entire time span
            self.v = (self.r / 2) * (sol.T[3, :] + sol.T[4, :])
            self.w = (self.r / self.L) * (sol.T[3, :] - sol.T[4, :])
            # wait for subscriber to create cmd_vel topic
            self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
            # wait for subscriber count to be greater or equal to 1
            while self.cmd_pub.get_subscription_count() < 1:
                self.get_logger().info("Waiting for subscriber to cmd_vel")
            # create a cmd pointer that decides which command to send
            self.command_pointer = 0
            self.cmd_timer = self.create_timer(0.01, self.send_cmd_vel)
        plot_results(np.array(self.t_span),
                     sol.T, np.array(controls).T, "r")

    def calc_epsilon_states(
        self, t: float, x: npt.NDArray[np.float64]
    ) -> tuple[
        npt.NDArray[np.float64],
        npt.NDArray[np.float64],
        npt.NDArray[np.float64],
        npt.NDArray[np.float64],
        npt.NDArray[np.float64],
    ]:
        """Calculate desired states and epsilon vectors.

        Calculates the desired state vectors:
            q: [sin(t), t].T
            q_dot: [cos(t), 1].T
            q_d_dot: [-sin(t), 0].T
        Calculates the epsilon vectors:
            y_epsilon: [x_1, x_2].T + epsilon * [cos(phi), sin(phi)].T
            y_epsilon_dot: R_epsilon @ v_bar

        Args:
            t: time
            x: state vector

        Returns:
            q, q_dot, q_d_dot, y_epsilon, y_epsilon_dot
        """
        # unpack and define variables needed for calculations
        x_1 = x.item(0)
        x_2 = x.item(1)
        phi = x.item(2)
        v = (self.r / 2) * (x.item(3) + x.item(4))
        omega = (self.r / self.L) * (x.item(3) - x.item(4))
        # define the desired state vectors
        q: npt.NDArray[np.float64] = np.array([[np.sin(t)], [t]])
        q_dot: npt.NDArray[np.float64] = np.array([[np.cos(t)], [1]])
        q_d_dot: npt.NDArray[np.float64] = np.array([[-np.sin(t)], [0]])
        # calulate y_epsilon 
        y_epsilon: npt.NDArray[np.float64] = (np.array([[x_1], [x_2]])
                                              + self.epsilon * np.array(
            [[np.cos(phi)], [np.sin(phi)]]
        ))
        # precalculate needed vectors for y_epsilon_dot
        R_epsilon: npt.NDArray[np.float64] = np.array(
            [
                [np.cos(phi), -self.epsilon * np.sin(phi)],
                [np.sin(phi), self.epsilon * np.cos(phi)],
            ]
        )
        v_bar: npt.NDArray[np.float64] = np.array([[v], [omega]])
        # calculate y_epsilon_dot
        y_epsilon_dot: npt.NDArray[np.float64] = R_epsilon @ v_bar
        return q, q_dot, q_d_dot, y_epsilon, y_epsilon_dot

    def send_cmd_vel(self) -> None:
        """Callback for sending command at time command pointer."""
        # check if the command pointer has exceeded the simulation time
        if self.command_pointer < len(self.v):
            cmd_vel = Twist()
            cmd_vel.linear.x = self.v[self.command_pointer]
            cmd_vel.angular.z = self.w[self.command_pointer]
            self.cmd_pub.publish(cmd_vel)
            self.command_pointer += 1
        # if not stop the robot and cancel the timer
        else:
            self.get_logger().info("Stopping robot")
            self.cmd_pub.publish(Twist())
            self.cmd_timer.cancel()
            plt.show()

    def calc_abar(
        self, x: npt.NDArray[np.float64], u: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Calculate a_bar for epsilon point control.

        a_bar = R_inv @ u - omega_hat @ v_bar.

        Args:
            x: state vector
            u: input vector

        Returns:
            a_bar
        """
        phi = x.item(2)
        omega = (self.r / self.L) * (x.item(3) - x.item(4))
        v = (self.r / 2) * (x.item(3) + x.item(4))
        R_inv = np.array([[1, 0], [0, 1 / self.epsilon]]) @ np.array(
            [[np.cos(phi), np.sin(phi)], [-np.sin(phi), np.cos(phi)]]
        )
        omega_hat = np.array(
            [[0, self.epsilon * omega], [omega / self.epsilon, 0]])
        v_bar = np.array([[v], [omega]])
        a_bar = (R_inv @ u) - (omega_hat @ v_bar)
        return a_bar

    def calculate_input(
        self, t: float, x: npt.NDArray[np.float64]
    ) -> npt.NDArray[np.float64]:
        """Calculate input with respect to the smooth velocity control model.

        Args:
            t: time
            x: state vector

        Returns:
            a_bar: [a, alpha]
        """
        y_d, y_d_dot, y_d_ddot, y_e, y_e_dot = self.calc_epsilon_states(t, x)
        y: npt.NDArray[np.float64] = np.array(
            [[y_e.item(0)], [y_e.item(1)], [
                y_e_dot.item(0)], [y_e_dot.item(1)]]
        )
        u_ff = y_d_ddot
        y_des: npt.NDArray[np.float64] = np.array(
            [[y_d.item(0)], [y_d.item(1)], [
                y_d_dot.item(0)], [y_d_dot.item(1)]]
        )
        u_hat = -self.k @ (y - y_des)
        u = u_ff + u_hat
        a_bar = self.calc_abar(x, u)
        return a_bar

    def dynamics(
        self,
        x: npt.NDArray[np.float64],
        t: float
    ) -> npt.NDArray[np.float64]:
        """Calculate the dynamics of the system for a given time.

        Args:
            x: state vector
            t: time

        Returns:
            x_dot: state vector derivative
        """
        # unpack variables for clarity
        u = self.calculate_input(t, x)
        a = u.item(0)
        alpha = u.item(1)
        w_r = x.item(3)
        w_l = x.item(4)
        theta = x.item(2)
        # define accelerations
        u_l = (a / self.r) - (self.L / 2 / self.r) * alpha
        u_r = (self.L / self.r / 2) * alpha + (a / self.r)
        # define the derivative of the state vector
        x_dot = np.array(
            [
                [(self.r / 2) * (w_r + w_l) * np.cos(theta)],
                [(self.r / 2) * (w_r + w_l) * np.sin(theta)],
                [(self.r / self.L) * (w_r - w_l)],
                [u_r],
                [u_l],
            ]
        )
        # flatten the array for odeint
        x_dot = x_dot.flatten()
        return x_dot

    def get_control_input(
            self, x: npt.NDArray[np.float64], t: float) -> list[float]:
        """Get the control vector for a given state and time.

        Args:
            x: state vector
            t: time

        Returns:
            control vector
        """
        u = self.calculate_input(t, x)
        a = u.item(0)
        alpha = u.item(1)
        self.controls_history.append((a, alpha))
        u_r = a / self.r + (self.L / 2 / self.r) * alpha
        u_l = a / self.r - (self.L / 2 / self.r) * alpha
        return [u_r, u_l]

    def get_all_control_inputs(
            self,
            x_sol: npt.NDArray
    ) -> list[list[float]]:
        """Wrapper for getting all control inputs.

        Args:
            x_sol: solution from odeint

        Returns:
            control_inputs: list of control inputs
        """
        control_inputs = []
        for i, t in enumerate(self.t_span):
            x = x_sol[i]
            control_inputs.append(self.get_control_input(x, t))
        return control_inputs

    def ode_int_wrapper(
            self, x_0: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """Odient wrapper for the dynamics.

        Args:
            x_0: initial state vector

        Returns:
            sol: solution from odeint
        """
        x = x_0.reshape(-1, 1)
        x = x.flatten()
        # print(x.shape)
        sol = odeint(self.dynamics, x, self.t_span)
        return sol


def plot_results(
    tvec: npt.NDArray,
    xvec: npt.NDArray,
    uvec: npt.NDArray,
    color: str,
    fig=None,
    ax=None,
) -> tuple[plt.Figure, plt.Axes]:
    """Plot the results.

    Args:
        tvec (npt.NDArray): The time vector
        xvec (npt.NDArray): The state vector
        uvec (npt.NDArray): The control vector
        color (str): The color of the plot
        fig ([type], optional): The figure. Defaults to None.
        ax ([type], optional): The axis. Defaults to None.

    Returns:
        tuple[plt.Figure, plt.Axes]: The figure and axis
    """
    if fig is None:
        fig, ax = plt.subplots(len(xvec) + len(uvec))  # type: ignore
        linestyle = "solid"
    elif ax is None:
        fig, ax = plt.subplots(9)  # type: ignore
        linestyle = "solid"
    else:
        color = "g"
        linestyle = "dashed"
    for i in range(len(xvec)):
        ax[i].plot(tvec, xvec[i, :], color=color, linestyle=linestyle)
        if i == 0:
            ax[i].plot(tvec, [np.sin(t)
                       for t in tvec], color="g", linestyle="dashed")
        if i == 1:
            ax[i].plot(tvec, tvec, color="g", linestyle="dashed")
        ax[i].set_ylabel(f"x_{i+1}(t)")
    for i in range(len(uvec)):
        current_index = i + len(xvec)
        ax[current_index].plot(
            tvec, uvec[i, :], color=color, linestyle=linestyle)
        ax[current_index].set_ylabel(f"u_{i+1}(t)")
    ax[-1].set_xlabel("time (s)")
    return (fig, ax)  # type: ignore


def main(args=None):
    rclpy.init(args=args)
    node = EpsilonPointController(
        30, .01, 65/1000, 140/1000, np.array([0, 0, 0, 0, 0]))
    rclpy.spin(node)


if __name__ == "__main__":
    main()
