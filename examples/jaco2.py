import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R

from mujoco_base import MuJoCoBase


class Jaco2(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 120.89  # 89.608063
        self.cam.elevation = -15.81  # -11.588379
        self.cam.distance = 3.0  # 5.0
        self.cam.lookat = np.array([0.0, 0.0, 0.0])

        self.data.qpos[4] = 0.5
        self.data.ctrl[0] = self.data.qpos[4]

    def controller(self, model, data):
        """
        This function implements a controller that 
        mimics the forces of a fixed joint before release
        """
        # self.data.ctrl[4] = 0.5
        data.ctrl[0] = 10.1
        data.ctrl[1] = 10.5
        data.ctrl[2] = 10.5
        data.ctrl[3] = 10.5
        data.ctrl[4] = 5#100000000000+data.qpos[4]*1000 + data.qvel[4]*(-100)
        data.ctrl[5] = 10.7
        data.ctrl[6] = 40
        

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                # Step simulation environment
                mj.mj_step(self.model, self.data)

                # Apply control
                self.controller(self.model, self.data)

            if self.data.time >= self.simend:
                break

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Show joint frames
            self.opt.flags[mj.mjtVisFlag.mjVIS_JOINT] = 1

            # Update scene and render
            # self.cam.lookat[0] = self.data.qpos[0]
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()

    def quat2euler(self, quat):
        # SciPy defines quaternion as [x, y, z, w]
        # MuJoCo defines quaternion as [w, x, y, z]
        _quat = np.concatenate([quat[1:], quat[:1]])
        r = R.from_quat(_quat)

        # roll-pitch-yaw is the same as rotating w.r.t
        # the x, y, z axis in the world frame
        euler = r.as_euler('xyz', degrees=False)

        return euler


def main():
    xml_path = "./xml/jaco2/jaco2.xml"
    sim = Jaco2(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
