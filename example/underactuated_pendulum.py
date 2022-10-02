import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv
from scipy.linalg import solve_continuous_are

from mujoco_base import MuJoCoBase


class Acrobot(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 2.5])

        # Compute LQR gain
        A, B = self.linearization()
        Q = np.diag([10, 10, 10, 10])
        R = np.diag([0.1])
        P = solve_continuous_are(A, B, Q, R)
        self.K = -inv(B.T @ P @ B + R) @ B.T @ P @ A

        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a LQR controller for balancing.
        """
        state = np.array([
            [data.qpos[0]],
            [data.qvel[0]],
            [data.qpos[1]],
            [data.qvel[1]],
        ])
        data.ctrl[0] = (self.K @ state)[0, 0]

        # Apply noise to shoulder
        noise = mj.mju_standardNormal(0.0)
        data.qfrc_applied[0] = noise

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                # Step simulation environment
                mj.mj_step(self.model, self.data)

            if self.data.time >= self.simend:
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()

    def get_dx(self, inputs):
        """
        The state is [q1, dq1, q2, dq2]
        The inputs are [q1, dq1, q2, dq2, u]

        The function outputs [dq1, ddq1, dq2, ddq2]
        """
        # Apply inputs
        self.data.qpos[0] = inputs[0]
        self.data.qvel[0] = inputs[1]
        self.data.qpos[1] = inputs[2]
        self.data.qvel[1] = inputs[3]
        self.data.ctrl[0] = inputs[4]

        mj.mj_forward(self.model, self.data)

        # Record outputs
        dq1 = self.data.qvel[0]
        dq2 = self.data.qvel[1]

        # Convert sparse inertia matrix M into full (i.e. dense) matrix.
        # M is filled with the data from data.qM
        M = np.zeros((2, 2))
        mj.mj_fullM(self.model, M, self.data.qM)

        # Calculate f = ctrl - qfrc_bias
        f = np.array([
            [0 - self.data.qfrc_bias[0]],
            [self.data.ctrl[0] - self.data.qfrc_bias[1]]
        ])

        # Calculate qacc
        ddq = inv(M) @ f

        outputs = np.array([dq1, ddq[0, 0], dq2, ddq[1, 0]])

        return outputs

    def linearization(self, pert=0.001):
        f0 = self.get_dx(np.zeros(5))

        Jacobians = []
        for i in range(5):
            inputs_i = np.zeros(5)
            inputs_i[i] = pert
            jac = (self.get_dx(inputs_i) - f0) / pert
            Jacobians.append(jac[:, np.newaxis])

        A = np.concatenate(Jacobians[:4], axis=1)
        B = Jacobians[-1]

        return A, B


def main():
    xml_path = "./xml/acrobot.xml"
    sim = Acrobot(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
