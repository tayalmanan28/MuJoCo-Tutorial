import mujoco as mj
import numpy as np
from mujoco.glfw import glfw

from mujoco_base import MuJoCoBase


class DoublePendulum(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 5.0

    def reset(self):
        # Set initial angle of pendulum
        self.data.qpos[0] = 0.1

        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 2.0])

        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a Feedback Linearization controller
        """
        # Evaluate position-dependent energy (potential).
        mj.mj_energyPos(model, data)

        # Evaluate velocity-dependent energy (kinetic).
        mj.mj_energyVel(model, data)

        # Convert sparse inertia matrix M into full (i.e. dense) matrix.
        # M is filled with the data from data.qM
        M = np.zeros((2, 2))
        mj.mj_fullM(model, M, data.qM)

        # Defines PD gain and reference angles
        Kp = 100 * np.eye(2)
        Kd = 10 * np.eye(2)
        qref = np.array([[-0.5], [-1.6]])

        # f compensates Coriolis and gravitational forces
        f = data.qfrc_bias[:, np.newaxis]

        # τ = M * ddqref
        ddqref = Kp @ (qref - data.qpos[:2][:, np.newaxis]) + \
            Kd @ (0 - data.qvel[:2][:, np.newaxis])
        tau = M @ ddqref

        data.qfrc_applied = (tau + f)[:, 0]

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


def main():
    xml_path = "./xml/doublependulum.xml"
    sim = DoublePendulum(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
