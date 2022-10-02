import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R

from mujoco_base import MuJoCoBase

FSM_LEG1_SWING = 0
FSM_LEG2_SWING = 1

FSM_KNEE1_STANCE = 0
FSM_KNEE1_RETRACT = 1

FSM_KNEE2_STANCE = 0
FSM_KNEE2_RETRACT = 1


class Biped(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0

        self.fsm_hip = None
        self.fsm_knee1 = None
        self.fsm_knee2 = None

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 120.89  # 89.608063
        self.cam.elevation = -15.81  # -11.588379
        self.cam.distance = 8.0  # 5.0
        self.cam.lookat = np.array([0.0, 0.0, 2.0])

        self.data.qpos[4] = 0.5
        self.data.ctrl[0] = self.data.qpos[4]

        self.model.opt.gravity[0] = 9.81 * np.sin(0.1)
        self.model.opt.gravity[2] = -9.81 * np.cos(0.1)

        self.fsm_hip = FSM_LEG2_SWING
        self.fsm_knee1 = FSM_KNEE1_STANCE
        self.fsm_knee2 = FSM_KNEE2_STANCE

    def controller(self, model, data):
        """
        This function implements a controller that 
        mimics the forces of a fixed joint before release
        """
        # State Estimation
        quat_leg1 = data.xquat[1, :]
        euler_leg1 = self.quat2euler(quat_leg1)
        abs_leg1 = -euler_leg1[1]
        pos_foot1 = data.xpos[2, :]

        quat_leg2 = data.xquat[3, :]
        euler_leg2 = self.quat2euler(quat_leg2)
        abs_leg2 = -euler_leg2[1]
        pos_foot2 = data.xpos[4, :]

        # Transition check
        if self.fsm_hip == FSM_LEG2_SWING and pos_foot2[2] < 0.05 and abs_leg1 < 0.0:
            self.fsm_hip = FSM_LEG1_SWING
        if self.fsm_hip == FSM_LEG1_SWING and pos_foot1[2] < 0.05 and abs_leg2 < 0.0:
            self.fsm_hip = FSM_LEG2_SWING

        if self.fsm_knee1 == FSM_KNEE1_STANCE and pos_foot2[2] < 0.05 and abs_leg1 < 0.0:
            self.fsm_knee1 = FSM_KNEE1_RETRACT
        if self.fsm_knee1 == FSM_KNEE1_RETRACT and abs_leg1 > 0.1:
            self.fsm_knee1 = FSM_KNEE1_STANCE

        if self.fsm_knee2 == FSM_KNEE2_STANCE and pos_foot1[2] < 0.05 and abs_leg2 < 0.0:
            self.fsm_knee2 = FSM_KNEE2_RETRACT
        if self.fsm_knee2 == FSM_KNEE2_RETRACT and abs_leg2 > 0.1:
            self.fsm_knee2 = FSM_KNEE2_STANCE

        # Control
        if self.fsm_hip == FSM_LEG1_SWING:
            self.data.ctrl[0] = -0.5
        if self.fsm_hip == FSM_LEG2_SWING:
            self.data.ctrl[0] = 0.5

        if self.fsm_knee1 == FSM_KNEE1_STANCE:
            self.data.ctrl[2] = 0.0
        if self.fsm_knee1 == FSM_KNEE1_RETRACT:
            self.data.ctrl[2] = -0.25

        if self.fsm_knee2 == FSM_KNEE2_STANCE:
            self.data.ctrl[4] = 0.0
        if self.fsm_knee2 == FSM_KNEE2_RETRACT:
            self.data.ctrl[4] = -0.25

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
            self.cam.lookat[0] = self.data.qpos[0]
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
    xml_path = "./xml/biped.xml"
    sim = Biped(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
