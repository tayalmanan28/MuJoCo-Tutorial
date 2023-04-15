import mujoco as mj
import numpy as np
from mujoco.glfw import glfw
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv
from scipy.linalg import solve_continuous_are
from controllers.lqr import lqr
from mujoco_base import MuJoCoBase


class Iiwa(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 30.0

    def reset(self, model, data):
        # Set camera configuration
        self.cam.azimuth = 120.89  # 89.608063
        self.cam.elevation = -15.81  # -11.588379
        self.cam.distance = 2.0  # 5.0
        self.cam.lookat = np.array([0.0, 0.0, 0.50])

        qpos0, ctrl0 = self.stand_two_legs(model, data)

        # Compute LQR gain
        self.K, dq = lqr(model, data, qpos0, ctrl0)
        
        #mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a controller that 
        mimics the forces of a fixed joint before release
        """

        state = np.array([
            [data.qpos[0]],
            [data.qvel[0]],
            [data.qpos[1]],
            [data.qvel[1]],
            [data.qpos[2]],
            [data.qvel[2]],
            [data.qpos[3]],
            [data.qvel[3]],
            [data.qpos[4]],
            [data.qvel[4]],
            [data.qpos[5]],
            [data.qvel[5]],
            [data.qpos[6]],
            [data.qvel[6]],
        ])
        data.ctrl = (self.K @ state)
        # data.ctrl[0] = 10.1
        # data.ctrl[1] = 10.5
        # data.ctrl[2] = 10.5
        # data.ctrl[3] = 10.5
        # data.ctrl[4] = 5#100000000000+data.qpos[4]*1000 + data.qvel[4]*(-100)
        # data.ctrl[5] = 10.7
        # data.ctrl[6] = 40
        
        # print(data.qpos, data.ctrl[4])
        
        # Apply noise to shoulder
        noise = mj.mju_standardNormal(0.0)
        data.qfrc_applied[0] = noise
        

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
            #self.cam.lookat[0] = self.data.qpos[0]
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
        The state is [q1, dq1, q2, dq2, q3, dq3, q4, dq4, q5, dq5, q6, dq6, q7, dq7]
        The inputs are [u]

        The function outputs [dq1, ddq1, dq2, ddq2]
        """
        # Apply inputs
        self.data.ctrl[0] = inputs[0]
        self.data.ctrl[1] = inputs[1]
        self.data.ctrl[2] = inputs[2]
        self.data.ctrl[3] = inputs[3]
        self.data.ctrl[4] = inputs[4]
        self.data.ctrl[5] = inputs[5]
        self.data.ctrl[6] = inputs[6]

        mj.mj_forward(self.model, self.data)

        # Record outputs
        dq1 = self.data.qvel[0]
        dq2 = self.data.qvel[1]
        dq3 = self.data.qvel[2]
        dq4 = self.data.qvel[3]
        dq5 = self.data.qvel[4]
        dq6 = self.data.qvel[5]
        dq7 = self.data.qvel[6]

        # Convert sparse inertia matrix M into full (i.e. dense) matrix.
        # M is filled with the data from data.qM
        M = np.zeros((7, 7))
        mj.mj_fullM(self.model, M, self.data.qM)

        # Calculate f = ctrl - qfrc_bias
        f = np.array([
            [self.data.ctrl[0] - self.data.qfrc_bias[0]],
            [self.data.ctrl[1] - self.data.qfrc_bias[1]],
            [self.data.ctrl[2] - self.data.qfrc_bias[2]],
            [self.data.ctrl[3] - self.data.qfrc_bias[3]],
            [self.data.ctrl[4] - self.data.qfrc_bias[4]],
            [self.data.ctrl[5] - self.data.qfrc_bias[5]],
            [self.data.ctrl[6] - self.data.qfrc_bias[6]]
        ])

        # Calculate qacc
        ddq = inv(M) @ f

        outputs = np.array([dq1, ddq[0, 0], dq2, ddq[1, 0], dq3, ddq[2, 0], dq4, ddq[3, 0], dq5, ddq[4, 0], dq6, ddq[5, 0], dq7, ddq[6, 0]])

        return outputs

    def linearization(self, pert=0.001):
        f0 = self.get_dx(np.zeros(7))

        Jacobians = []
        for i in range(7):
            inputs_i = np.zeros(7)
            inputs_i[i] = pert
            jac = (self.get_dx(inputs_i) - f0) / pert
            Jacobians.append(jac[:, np.newaxis])

        A = np.concatenate(Jacobians[:6], axis=1)
        B = Jacobians[-1]

        return A, B

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
    xml_path = "./xml/iiwa/iiwa.xml"
    sim = Iiwa(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
