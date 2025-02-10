import os
from datetime import datetime
import numpy as np
import argparse
from pathlib import Path

from double_pendulum.model.symbolic_plant import SymbolicDoublePendulum
from double_pendulum.model.model_parameters import model_parameters
from double_pendulum.simulation.simulation import Simulator
from double_pendulum.controller.lqr.lqr_controller import LQRController
from double_pendulum.utils.plotting import plot_timeseries


if __name__ == "__main__":
    argparse = argparse.ArgumentParser()
    argparse.add_argument('-p', '--plant', help='Plant', required=True)
    argparse.add_argument('-t', '--time', help='Sim time', required=True)
    argparse.add_argument('-v', '--video', help='Save video', required=False, default=True )
    argparse.add_argument('-d', '--dir', help='Out dir (traj)', default="data/ML4KP", required=False)
    argparse.add_argument('--x0')
    args = argparse.parse_args()
    
    robot = args.plant

    if robot == "pendubot":
        torque_limit = [5.0, 0.0]
        active_act = 0
    elif robot == "acrobot":
        plant = SymbolicDoublePendulum(
            mass=[0.5234602302310271, 0.6255677234174437],
            length=[0.2, 0.3],
            com=[0.2, 0.25569305436052964],
            damping=[0.0, 0.0],
            gravity=9.81,
            coulomb_fric=[0.0, 0.0],
            inertia=[0.031887199591513114, 0.05086984812807257],
            motor_inertia=0.0,
            gear_ratio=6,
            torque_limit=[0, 6.0])

    # simulation parameters
    dt = 0.002
    t_final = float(args.time)
    integrator = "runge_kutta"
    x0 = [float(e) for e in args.x0.split(" ")]
    save_video = args.video == "True";
    # print(x0, save_video, args.video)
    # goal = [np.pi, 0., 0., 0.]

    # timestamp = datetime.today().strftime("%Y%m%d-%H%M%S")
    save_dir = args.dir
    # os.makedirs(save_dir,exist_ok=True, parents=True)
    Path(save_dir).mkdir(parents=True, exist_ok=True)

    sim = Simulator(plant=plant)


    T, X, U = sim.simulate_and_animate(t0=0.0, x0=x0,
                                   tf=t_final, dt=dt, controller=None,
                                   integrator=integrator,
                                   save_video=True,
                                   video_name=os.path.join(save_dir, "trajectory.mp4"))

    file = open(os.path.join(save_dir, "py_traj.txt"), "w");
    for state in X:
        line = f"%.5f %.5f %.5f %.5f\n" % (state[0], state[1], state[2], state[3]);
        file.write(line)
    file.close()
