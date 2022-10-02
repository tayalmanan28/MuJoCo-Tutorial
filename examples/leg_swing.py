import mujoco as mj
import numpy as np
from mujoco.glfw import glfw

from mujoco_base import MuJoCoBase

# Define states for the finite state machine
FSM_HOLD = 0
FSM_SWING1 = 1
FSM_SWING2 = 2
FSM_STOP = 3


class LegSwing(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 5.0
        self.fsm_state = FSM_HOLD

        # Define durations of each state
        self.t_hold = 0.5
        self.t_swing1 = 1.0
        self.t_swing2 = 1.0

        # Define setpoints
        self.q_init = np.array([[-1.0], [0.0]])
        self.q_mid = np.array([[0.5], [-2.0]])
        self.q_end = np.array([[1.0], [0.0]])

        # Define setpoint times
        self.t_init = self.t_hold
        self.t_mid = self.t_hold + self.t_swing1
        self.t_end = self.t_hold + self.t_swing1 + self.t_swing2

        # Get trajectories
        self.a_swing1 = self.generate_trajectory(
            self.t_init, self.t_mid, self.q_init, self.q_mid)
        self.a_swing2 = self.generate_trajectory(
            self.t_mid, self.t_end, self.q_mid, self.q_end)

    def reset(self):
        # Set initial angle of pendulum
        self.data.qpos[0] = -1

        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        self.fsm_state = FSM_HOLD

        mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        """
        This function implements a PD controller for tracking
        the reference motion.
        """
        time = data.time

        # Check for state change
        if self.fsm_state == FSM_HOLD and time >= self.t_hold:
            self.fsm_state = FSM_SWING1
        elif self.fsm_state == FSM_SWING1 and time >= self.t_mid:
            self.fsm_state = FSM_SWING2
        elif self.fsm_state == FSM_SWING2 and time >= self.t_end:
            self.fsm_state = FSM_STOP

        # Get reference joint position & velocity
        if self.fsm_state == FSM_HOLD:
            q_ref = self.q_init
            dq_ref = np.zeros((2, 1))
        elif self.fsm_state == FSM_SWING1:
            q_ref = self.a_swing1[0] + self.a_swing1[1]*time + \
                self.a_swing1[2]*(time**2) + self.a_swing1[3]*(time**3)
            dq_ref = self.a_swing1[1] + 2 * self.a_swing1[2] * \
                time + 3 * self.a_swing1[3]*(time**2)
        elif self.fsm_state == FSM_SWING2:
            q_ref = self.a_swing2[0] + self.a_swing2[1]*time + \
                self.a_swing2[2]*(time**2) + self.a_swing2[3]*(time**3)
            dq_ref = self.a_swing2[1] + 2 * self.a_swing2[2] * \
                time + 3 * self.a_swing2[3]*(time**2)
        elif self.fsm_state == FSM_STOP:
            q_ref = self.q_end
            dq_ref = np.zeros((2, 1))

        # Define PD gains
        kp = 500
        kv = 50

        # Compute PD control
        data.ctrl = kp * (q_ref[:, 0] - data.qpos) + \
            kv * (dq_ref[:, 0] - data.qvel)

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

    def generate_trajectory(self, t0, tf, q0, qf):
        """
        Generates a trajectory
        q(t) = a0 + a1t + a2t^2 + a3t^3
        which satisfies the boundary condition
        q(t0) = q0, q(tf) = qf, dq(t0) = 0, dq(tf) = 0
        """
        tf_t0_3 = (tf - t0)**3
        a0 = qf*(t0**2)*(3*tf-t0) + q0*(tf**2)*(tf-3*t0)
        a0 = a0 / tf_t0_3

        a1 = 6 * t0 * tf * (q0 - qf)
        a1 = a1 / tf_t0_3

        a2 = 3 * (t0 + tf) * (qf - q0)
        a2 = a2 / tf_t0_3

        a3 = 2 * (q0 - qf)
        a3 = a3 / tf_t0_3

        return a0, a1, a2, a3


def main():
    xml_path = "./xml/doublependulum_leg.xml"
    sim = LegSwing(xml_path)
    sim.reset()
    sim.simulate()


if __name__ == "__main__":
    main()
