import matplotlib as mpl
import matplotlib.pyplot as plt
import mujoco as mj
import nlopt
import numpy as np
from mujoco.glfw import glfw

from mujoco_base import MuJoCoBase

mpl.rcParams['text.usetex'] = True
mpl.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'
plt.rcParams["font.size"] = 16


class ManipulatorIK(MuJoCoBase):
    def __init__(self, xml_path):
        super().__init__(xml_path)
        self.simend = 10.0
        self.data_sim = mj.MjData(self.model)

    def reset(self):
        # Set camera configuration
        self.cam.azimuth = 89.608063
        self.cam.elevation = -11.588379
        self.cam.distance = 5.0
        self.cam.lookat = np.array([0.0, 0.0, 1.5])

        # Get center of Lemniscate
        end_eff_pos = self.forward_kinematics([-0.5, 1.0])
        self.center_x = end_eff_pos[0] - 0.25
        self.center_z = end_eff_pos[1]
        self.omega = 0.4
        self.a = 0.25
        self.simend = 0.25 + 2 * np.pi / self.omega

        # Get initial joint angles
        q_guess = np.array([-0.5, 1.0])
        self.x_target = self.get_lemniscate_ref(0.0)
        self.q_pos = self.inverse_kinematics(q_guess)

        self.data.qpos[0] = self.q_pos[0]
        self.data.qpos[1] = self.q_pos[1]

        # Create list to store data
        self.end_eff_pos = []

        # mj.set_mjcb_control(self.controller)

    def controller(self, model, data):
        # Get reference end-effector position
        self.x_target = self.get_lemniscate_ref(data.time)

        # Use current joint angles as the initial guess
        qpos = np.array([data.qpos[0], data.qpos[1]])

        # Solve for the joint angles
        sol = self.inverse_kinematics(qpos)

        # Apply control
        self.data.ctrl[0] = sol[0]
        self.data.ctrl[2] = sol[1]

    def forward_kinematics(self, q):
        self.data_sim.qpos[0] = q[0]
        self.data_sim.qpos[1] = q[1]
        self.data_sim.ctrl[0] = self.data_sim.qpos[0]
        self.data_sim.ctrl[2] = self.data_sim.qpos[1]

        mj.mj_forward(self.model, self.data_sim)

        end_eff_pos = np.array([
            self.data_sim.sensordata[0],
            self.data_sim.sensordata[2]
        ])

        return end_eff_pos

    def cost_func(self, x, grad):
        cost = 0.0

        return cost

    def equality_constraints(self, result, x, grad):
        end_eff_pos = self.forward_kinematics(x)
        result[0] = end_eff_pos[0] - self.x_target[0]
        result[1] = end_eff_pos[1] - self.x_target[1]

    def inverse_kinematics(self, x):
        # Define optimization problem
        opt = nlopt.opt(nlopt.LN_COBYLA, 2)

        # Define lower and upper bounds
        opt.set_lower_bounds([-np.pi, -np.pi])
        opt.set_upper_bounds([np.pi, np.pi])

        # Set objective funtion
        opt.set_min_objective(self.cost_func)

        # Define equality constraints
        tol = [1e-4, 1e-4]
        opt.add_equality_mconstraint(self.equality_constraints, tol)

        # Set relative tolerance on optimization parameters
        opt.set_xtol_rel(1e-4)

        # Solve problem
        sol = opt.optimize(x)

        return sol

    def get_lemniscate_ref(self, t):
        wt = self.omega * t
        denominator = 1 + np.sin(wt) * np.sin(wt)

        x = self.center_x + (self.a * np.cos(wt)) / denominator
        z = self.center_z + (self.a * np.sin(wt) * np.cos(wt)) / denominator

        ref_pos = np.array([x, z])

        return ref_pos

    def simulate(self):
        while not glfw.window_should_close(self.window):
            simstart = self.data.time

            while (self.data.time - simstart < 1.0/60.0):
                # Step simulation environment
                self.controller(self.model, self.data)
                mj.mj_step(self.model, self.data)

            end_eff_pos = np.array([
                self.data.sensordata[0],
                self.data.sensordata[2]
            ])
            self.end_eff_pos.append(end_eff_pos[:, np.newaxis])

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

    def visualize(self):
        # Measured motion trajectory
        end_eff_pos_arr = np.concatenate(self.end_eff_pos, axis=1)

        # Get reference trajectory
        wt = self.omega * np.linspace(0.0, self.simend, 500)
        denominator = 1 + np.sin(wt) * np.sin(wt)
        leminiscate_x = self.center_x + (self.a * np.cos(wt)) / denominator
        leminiscate_z = self.center_z + \
            (self.a * np.sin(wt) * np.cos(wt)) / denominator

        # Visualization
        fig, ax = plt.subplots(1, 1, figsize=(8, 5))

        ax.plot(
            end_eff_pos_arr[0, :],
            end_eff_pos_arr[1, :],
            color="cornflowerblue",
            linewidth=4,
            zorder=-2,
            label=r"$\textbf{Inverse Kinematics}$"
        )
        ax.plot(
            leminiscate_x,
            leminiscate_z,
            color="darkorange",
            linewidth=1,
            zorder=-1,
            label=r"$\textbf{Reference}$"
        )

        ax.grid()
        ax.set_aspect("equal")
        ax.legend(frameon=False, ncol=2, loc="lower center",
                  bbox_to_anchor=(0.5, -0.4))

        plt.savefig("imgs/manipulator_ik.png", dpi=200,
                    transparent=False, bbox_inches="tight")


def main():
    xml_path = "./xml/manipulator.xml"
    sim = ManipulatorIK(xml_path)
    sim.reset()
    sim.simulate()
    sim.visualize()


if __name__ == "__main__":
    main()
