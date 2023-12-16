#!/bin/usr/env python3

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
            x_0: npt.NDArray[np.float64]
    ) -> None:
        """Initialize.

        Args:
            t_f: final time for simulation
            epsilon: epsilon value for epsilon point control
            r: wheel radius
            L: wheelbase
        """
        super().__init__("epsilon_point_controller")  # type: ignore
        self.state_pub = self.create_publisher(
            StatePlotting, "state_plotting", 10)
        self.A = np.array([[0, 0, 1, 0], [0, 0, 0, 1],
                          [0, 0, 0, 0], [0, 0, 0, 0]])
        self.B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])
        Q = np.diag([1, 2, 3, 4])
        R = np.diag([1, 2])
        self.r = r
        self.L = L
        self.k = ct.lqr(self.A, self.B, Q, R)[0]
        print(f"K: {self.k}")
        self.t_span = np.arange(0, t_f, 0.01)
        self.epsilon = epsilon
        self.controls_history: list[tuple[float, float]] = []
        self.control_tspan: list[float] = []
        sol = self.ode_int_wrapper(x_0)
        controls, tspan = self.get_all_control_inputs(sol)
        self.control_tspan = tspan
        plot_results(np.array(tspan),
                     sol.T, np.array(controls).T, "r")
        states = StatePlotting()
        print(tspan)
        states.tspan = self.control_tspan.tolist()
        states.x = sol.T[0, :].tolist()
        states.y = sol.T[1, :].tolist()
        states.phi = sol.T[2, :].tolist()
        states.w_r = sol.T[3, :].tolist()
        states.w_l = sol.T[4, :].tolist()
        self.v = (self.r / 2) * (sol.T[3, :] + sol.T[4, :])
        self.w = (self.r / self.L) * (sol.T[3, :] - sol.T[4, :])
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_pub.publish(states)
        self.get_logger().info("Published states")
        self.command_pointer = 0
        self.cmd_timer = self.create_timer(0.01, self.send_cmd_vel)
        plt.show(block=False)

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
        x_1 = x.item(0)
        x_2 = x.item(1)
        phi = x.item(2)
        v = (self.r / 2) * (x.item(3) + x.item(4))
        omega = (self.r / self.L) * (x.item(3) - x.item(4))
        q: npt.NDArray[np.float64] = np.array([[np.sin(t)], [t]])
        q_dot: npt.NDArray[np.float64] = np.array([[np.cos(t)], [1]])
        q_d_dot: npt.NDArray[np.float64] = np.array([[-np.sin(t)], [0]])
        y_epsilon: npt.NDArray[np.float64] = (np.array([[x_1], [x_2]])
                                              + self.epsilon * np.array(
            [[np.cos(phi)], [np.sin(phi)]]
        ))
        R_epsilon: npt.NDArray[np.float64] = np.array(
            [
                [np.cos(phi), -self.epsilon * np.sin(phi)],
                [np.sin(phi), self.epsilon * np.cos(phi)],
            ]
        )
        v_bar: npt.NDArray[np.float64] = np.array([[v], [omega]])
        y_epsilon_dot: npt.NDArray[np.float64] = R_epsilon @ v_bar
        return q, q_dot, q_d_dot, y_epsilon, y_epsilon_dot

    def send_cmd_vel(self) -> None:
        self.get_logger().info(f"command_pointer: {self.command_pointer}")
        if self.command_pointer < len(self.v):
            cmd_vel = Twist()
            cmd_vel.linear.x = self.v[self.command_pointer]
            cmd_vel.angular.z = self.w[self.command_pointer]
            self.cmd_pub.publish(cmd_vel)
            self.get_logger().info(
                f"v: {cmd_vel.linear.x}, w: {cmd_vel.angular.z}")
            self.command_pointer += 1

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
        u = self.calculate_input(t, x)
        a = u.item(0)
        alpha = u.item(1)
        w_r = x.item(3)
        w_l = x.item(4)
        theta = x.item(2)
        u_l = (a / self.r) - (self.L / 2 / self.r) * alpha
        u_r = (self.L / self.r / 2) * alpha + (a / self.r)
        x_dot = np.array(
            [
                [(self.r / 2) * (w_r + w_l) * np.cos(theta)],
                [(self.r / 2) * (w_r + w_l) * np.sin(theta)],
                [(self.r / self.L) * (w_r - w_l)],
                [u_r],
                [u_l],
            ]
        )
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
        u_r = a / self.r - (self.L / 2 / self.r) * alpha
        u_l = (self.L / self.r) * alpha + a / self.r
        return [u_r, u_l]

    def get_all_control_inputs(
            self,
            x_sol: npt.NDArray
    ) -> tuple[list[float], npt.NDArray[np.float64]]:
        """Wrapper for getting all control inputs.

        Args:
            x_sol: solution from odeint

        Returns:
            control_inputs: list of control inputs
            t_span: time span
        """
        control_inputs = []
        print(x_sol[1].shape)
        for i, t in enumerate(self.t_span):
            x = x_sol[i]
            control_inputs.append(self.get_control_input(x, t))
        return control_inputs, self.t_span

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
        print(x.shape)
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
    node = EpsilonPointController(30, .01, 1, 1, np.array([0, 0, 0, 0, 0]))
    rclpy.spin(node)


if __name__ == "__main__":
    main()
