import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv

from mujoco_base import MuJoCoBase


class ManipulatorDrawing(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 20.0

        # radius of drawn circle
        self.r = 0.5

    def reset(self):
        # Set initial angle of pendulum
        self.data.qpos[0] = -0.5
        self.data.qpos[1] = 1.0

        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        mj.mj_forward(self.model, self.data)

        # center of drawn circle
        self.x_0 = self.data.sensordata[0] - self.r
        self.z_0 = self.data.sensordata[2]

        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a P controller for tracking
        the reference motion.
        """
        # End-effector position
        end_eff_pos = data.sensordata[:3]

        # Compute end-effector Jacobian
        jacp = np.zeros((3, 2))
        mj.mj_jac(model, data, jacp, None, end_eff_pos, 2)

        # Δq = Jinv * Δx
        J = jacp[[0, 2], :]
        dx = np.array([
            [self.x_0 + self.r * np.cos(data.time) - self.data.sensordata[0]],
            [self.z_0 + self.r * np.sin(data.time) - self.data.sensordata[2]]
        ])
        dq = inv(J) @ dx

        # Target position is q + Δq
        data.ctrl[0] = data.qpos[0] + dq[0, 0]
        data.ctrl[2] = data.qpos[1] + dq[1, 0]

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
    xml_path = "./xml/doublependulum_manipulator.xml"
    sim = ManipulatorDrawing(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
