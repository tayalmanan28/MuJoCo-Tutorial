import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from numpy.linalg import inv

from mujoco_base import MuJoCoBase

FSM_AIR1 = 0
FSM_STANCE1 = 1
FSM_STANCE2 = 2
FSM_AIR2 = 3

class Hopper(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0
        self.fsm = None
        self.step_no = 0

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        self.fsm = FSM_AIR1
        self.step_no = 0

        # pservo-hip
        self.set_position_servo(0, 100)

        # vservo-hip
        self.set_velocity_servo(1, 10)

        # pservo-knee
        self.set_position_servo(2, 1000)

        # vservo-knee
        self.set_velocity_servo(3, 0)

        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a controller that 
        mimics the forces of a fixed joint before release
        """
        body_no = 3
        z_foot = data.xpos[body_no, 2]
        vz_torso = data.qvel[1]

        # Lands on the ground
        if self.fsm == FSM_AIR1 and z_foot < 0.05:
            self.fsm = FSM_STANCE1
        
        # Moving upward
        if self.fsm == FSM_STANCE1 and vz_torso > 0.0:
            self.fsm = FSM_STANCE2

        # Take off
        if self.fsm == FSM_STANCE2 and z_foot > 0.05:
            self.fsm = FSM_AIR2
        
        # Moving downward
        if self.fsm == FSM_AIR2 and vz_torso < 0.0:
            self.fsm = FSM_AIR1
            self.step_no += 1
        
        if self.fsm == FSM_AIR1:
            self.set_position_servo(2, 100)
            self.set_velocity_servo(3, 10)

        if self.fsm == FSM_STANCE1:
            self.set_position_servo(2, 1000)
            self.set_velocity_servo(3, 0)

        if self.fsm == FSM_STANCE2:
            self.set_position_servo(2, 1000)
            self.set_velocity_servo(3, 0)
            data.ctrl[0] = -0.2

        if self.fsm == FSM_AIR2:
            self.set_position_servo(2, 100)
            self.set_velocity_servo(3, 10)
            data.ctrl[0] = 0.0

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
            self.cam.lookat[0] = self.data.qpos[0]
            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                               mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

        glfw.terminate()
    
    def set_position_servo(self, actuator_no, kp):
        self.model.actuator_gainprm[actuator_no, 0] = kp
        self.model.actuator_biasprm[actuator_no, 1] = -kp
    
    def set_velocity_servo(self, actuator_no, kv):
        self.model.actuator_gainprm[actuator_no, 0] = kv
        self.model.actuator_biasprm[actuator_no, 2] = -kv



def main():
    xml_path = "./xml/hopper.xml"
    sim = Hopper(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
