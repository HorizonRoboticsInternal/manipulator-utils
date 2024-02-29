import dill
import time
import numpy as np
import mujoco
import mujoco.viewer


# Sim params
xml_path = "/home/asjchoi/Desktop/Hobot/hobot/environments/mj_models/widowx250s/widowx250s.xml"
m = mujoco.MjModel.from_xml_path(xml_path)
m.opt.timestep = 0.05
d = mujoco.MjData(m)


class MuJoCoStepper:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.dt = model.opt.timestep

    def apply_control(self, action):
        assert len(action) == 7, "Action size should be 7"
        self.data.ctrl = np.asarray(action)

    def step(self, step_start, record: list = None):
        mujoco.mj_step(self.model, self.data)
        viewer.sync()
        if record is not None:
            record.append(self.data.qpos.copy())
        self.wait_until_next_step(step_start)

    def wait_until_next_step(self, step_start):
        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = self.dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


stepper = MuJoCoStepper(m, d)

with open("data/joint_trajectories.pkl", "rb") as f:
    raw_joint_trajectories = dill.load(f)

# Need to add an extra column for the gripper
joint_trajectories = []
for i in range(len(raw_joint_trajectories)):
    tmp = raw_joint_trajectories[i]
    jt = np.zeros((tmp.shape[0], 7))
    jt[:, :6] = tmp
    joint_trajectories.append(jt)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Enable wireframe rendering of the entire scene.
    # viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
    # viewer.sync()

    # Go to neutral position
    for i in range(10):
        step_start = time.time()
        stepper.apply_control([0.0]*7)
        stepper.step(step_start)

    # Go to the starting position
    for i in range(50):
        step_start = time.time()
        stepper.apply_control([0.0, -0.00020950394840690834, 0.9273922091469897,
                               0.0, -0.9271827051985841, 0.0, 0.0])
        stepper.step(step_start)

    # Carry out the joint trajectories and record.
    recorded_trajectories = []
    for jt in joint_trajectories:
        rtj = []
        for cmd in jt:
            step_start = time.time()
            stepper.apply_control(cmd)
            stepper.step(step_start, rtj)
        recorded_trajectories.append(rtj)
        time.sleep(0.5)

# Record sim trajectory recordings
with open("data/sim_trajectories.pkl", "wb") as f:
    dill.dump(recorded_trajectories, f)

print("Finished sim recording.")
