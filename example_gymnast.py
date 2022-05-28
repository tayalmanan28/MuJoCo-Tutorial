import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv

from mujoco_base import MuJoCoBase

FSM_SWING = 0
FSM_FREE = 1

class Gymnast(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 5.0
        self.fsm = None

    def reset(self):
        # Set initial configuration
        self.data.qpos[2] = -np.pi/2
        self.data.qpos[5] = -np.pi/2

        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 7.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        self.fsm = FSM_SWING
        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a controller that 
        mimics the forces of a fixed joint before release
        """
        # Get constraint Jacobian
        J = data.efc_J[:3, :3]

        # Get constraint force
        F0 = data.efc_force[:3][:, np.newaxis]

        # Get constrained joint torque
        JT_F = J.T @ F0

        # Release condition check
        if self.fsm == FSM_SWING and data.qpos[5] > 1.0:
            self.fsm = FSM_FREE
        
        if self.fsm == FSM_SWING:
            data.qfrc_applied[2] = -1 * (data.qvel[2] - 5.0)
            data.qfrc_applied[3] = JT_F[0, 0]
            data.qfrc_applied[4] = JT_F[1, 0]
            data.qfrc_applied[5] = JT_F[2, 0] + data.qfrc_applied[2]
        elif self.fsm == FSM_FREE:
            data.qfrc_applied[3] = 0.0
            data.qfrc_applied[4] = 0.0
            data.qfrc_applied[5] = 0.0

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

            # Show joint frames
            self.opt.flags[mj.mjtVisFlag.mjVIS_JOINT] = 1

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
    xml_path = "./xml/gymnast.xml"
    sim = Gymnast(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
