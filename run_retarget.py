import time
import numpy as np
import argparse
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_, unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# Number of motors on the G1 robot
G1_NUM_MOTOR = 29

# PD gains for the 29 joints (using the same gains as in g1_low_level_example.py)
Kp = [
    60, 60, 60, 100, 40, 40,   # left leg
    60, 60, 60, 100, 40, 40,   # right leg
    60, 40, 40,               # waist
    40, 40, 40, 40, 40, 40, 40, # left arm
    40, 40, 40, 40, 40, 40, 40  # right arm
]
Kd = [
    1, 1, 1, 2, 1, 1,         # left leg
    1, 1, 1, 2, 1, 1,         # right leg
    1, 1, 1,                  # waist
    1, 1, 1, 1, 1, 1, 1,       # left arm
    1, 1, 1, 1, 1, 1, 1        # right arm
]

class MocapRetarget:
    def __init__(self, csv_file):
        # Load CSV data. Each row has 36 values:
        # - First 7: base pose (position + quaternion) [ignored for low-level joint control]
        # - Last 29: joint angles (in radians)
        self.data = np.loadtxt(csv_file, delimiter=',')
        self.num_frames = self.data.shape[0]
        self.fps = 30.0
        self.scale = .5
        self.mocap_skip = 7
        self.offset = 0 # set tp 15 for left arm & right arm only
        self.control_dt = 1.0 / self.fps 
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()  # default constructor as in original example
        self.low_state = None
        self.crc = CRC()

    def low_state_handler(self, msg: unitree_hg_msg_dds__LowState_):
        self.low_state = msg

    def init_communication(self):
        # Initialize the channel factory
        ChannelFactoryInitialize(0)
        
        # Create and initialize publisher and subscriber
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()
        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self.low_state_handler, 10)

        # Initialize the Motion Switcher Client to ensure no other control mode is active
        msc = MotionSwitcherClient()
        msc.SetTimeout(5.0)
        msc.Init()
        status, result = msc.CheckMode()
        if result is None:
            print("Warning: No response from CheckMode. Proceeding under assumption that no other control mode is active.")
        else:
            while result.get('name'):
                msc.ReleaseMode()
                status, result = msc.CheckMode()
                time.sleep(1)

        # Wait until the first low_state is received
        print("Waiting for robot state...")
        while self.low_state is None:
            time.sleep(0.001)
        self.mode_machine = self.low_state.mode_machine
        print("Robot state acquired; mode_machine =", self.mode_machine)

    def send_first_frame(self):
        # Extract joint angles from the first row (skip the first 7 base values)
        first_angles = self.data[0, self.mocap_skip:]
        for i in range(G1_NUM_MOTOR):
            self.low_cmd.motor_cmd[i].mode = 1          # Enable motor
            self.low_cmd.motor_cmd[i].q = float(first_angles[i])
            self.low_cmd.motor_cmd[i].dq = 0.0
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].kp = Kp[i]
            self.low_cmd.motor_cmd[i].kd = Kd[i]
        self.low_cmd.mode_pr = 0                         # Mode.PR (0) for pitch/roll mode
        self.low_cmd.mode_machine = self.mode_machine
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_pub.Write(self.low_cmd)

    def run(self):
        # Begin motion playback
        next_time = time.time()
        print(f"Starting motion playback at {self.fps} FPS...")
        for frame_idx in range(self.num_frames):
            next_time += self.control_dt
            joint_angles = self.data[frame_idx, self.mocap_skip:] * self.scale  # Extract joint angles from the row
            for i in range(self.offset, G1_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].q = float(joint_angles[i])
            self.low_cmd.mode_pr = 0
            self.low_cmd.mode_machine = self.mode_machine
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_pub.Write(self.low_cmd)
            
            sleep_duration = next_time - time.time()
            if sleep_duration > 0:
                time.sleep(sleep_duration)
        print("Motion playback finished.")

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Run mocap retargeting on G1 robot')
    parser.add_argument('--csv_file', type=str, required=True,
                      help='Path to the mocap CSV file')
    parser.add_argument('--fps', type=float, default=30.0,
                      help='Playback frames per second (default: 30.0)')
    parser.add_argument('--scale', type=float, default=0.5,
                      help='Scaling factor for joint angles (default: 0.5)')
    parser.add_argument('--mocap_skip', type=int, default=7,
                      help='Number of initial columns to skip in CSV (default: 7)')
    parser.add_argument('--offset', type=int, default=0,
                      help='Starting joint index offset (default: 0)')

    args = parser.parse_args()

    # Initialize and run the retargeting
    retarget = MocapRetarget(args.csv_file)
    retarget.fps = args.fps
    retarget.scale = args.scale
    retarget.mocap_skip = args.mocap_skip
    retarget.offset = args.offset
    
    retarget.init_communication()
    retarget.send_first_frame()
    retarget.run()
