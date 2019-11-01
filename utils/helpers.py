from dataclasses import dataclass
from typing import Tuple

import torch
import random
import numpy as np
import gym
from gym import spaces
import math
import matplotlib.pyplot as plt

from aido_schemas import EpisodeStart, protocol_agent_duckiebot1, PWMCommands, Duckiebot1Commands, LEDSCommands, RGB, \
    wrap_direct, Context, Duckiebot1Observations, JPGImage, Context
    
from simulation.src.gym_duckietown.simulator import Simulator
class AIDOSubmission:
    def __init__(self, exercise='test'):
        self.exercise = exercise

    def init(self, context: Context):
        context.info('init()')

    def on_received_seed(self, data: int):
        np.random.seed(data)

    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info(f'Starting episode "{data.episode_name}".')

    def on_received_observations(self, data: Duckiebot1Observations):
        camera: JPGImage = data.camera
        obs = jpg2rgb(camera.jpg_data)
        self.current_image = obs

    def compute_action(self, observation):
        action = np.random.random((2,))
        return action.astype(float)

    def on_received_get_commands(self, context: Context):
        pwm_left, pwm_right = self.compute_action(self.current_image)

        grey = RGB(0.0, 0.0, 0.0)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = Duckiebot1Commands(pwm_commands, led_commands)
        context.write('commands', commands)

    def finish(self, context: Context):
        context.info('finish()')

def submit_aido(submission):
    protocol = protocol_agent_duckiebot1
    wrap_direct(node=submission, protocol=protocol)

def jpg2rgb(image_data: bytes) -> np.ndarray:
    """ Reads JPG bytes as RGB"""
    from PIL import Image
    import io
    im = Image.open(io.BytesIO(image_data))
    im = im.convert('RGB')
    data = np.array(im) 
    assert data.ndim == 3
    assert data.dtype == np.uint8
    return data

def launch_env(simclass=None, map_name = "loop_empty"):
    from simulation.src.gym_duckietown.simulator import Simulator

    simclass = Simulator if simclass is None else simclass
    
    env = simclass(
        seed=123, # random seed
        map_name=map_name,#"loop_empty",
        max_steps=500001, # we don't want the gym to reset itself
        domain_rand=0,
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4, # start close to straight
        full_transparency=True,
        distortion=True,
    )
    return env

def change_exercise(exercise='master'):
    from git import Repo
    
    repo = Repo('../simulation')
    repo.git.checkout(exercise)
    print('Exercise successfully changed to', exercise)

def wrap_env(env, results_path='./gym_results', force=True):
    from gym import wrappers
    return wrappers.Monitor(env, results_path, force=force)

def view_results_ipython(env, results_path='./gym_results'):
    import os
    import io
    import base64
    from IPython.display import HTML
    
    video = io.open(os.path.join(results_path, 'openaigym.video.%s.video000000.mp4' % env.file_infix), 'r+b').read()
    encoded = base64.b64encode(video)
    return HTML(data='''
        <video width="360" height="auto" alt="test" controls><source src="data:video/mp4;base64,{0}" type="video/mp4" /></video>'''
    .format(encoded.decode('ascii')))

class SteeringToWheelVelWrapper():
    """
    Converts policy that was trained with [velocity|heading] actions to
    [wheelvel_left|wheelvel_right] to comply with AIDO evaluation format
    """

    def __init__(self, gain=1.0, trim=0.0, radius=0.0318, k=27.0, limit=1.0, wheel_dist=0.102):
        # Should be adjusted so that the effective speed of the robot is 0.2 m/s
        self.gain = gain

        # Directional trim adjustment
        self.trim = trim

        # Wheel radius
        self.radius = radius

        # Motor constant
        self.k = k

        # Wheel velocity limit
        self.limit = limit

        # Distance between wheels
        self.wheel_dist = wheel_dist

    def convert(self, action):
        vel, angle = action

        # Distance between the wheels
        baseline = self.wheel_dist

        # assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = (vel + 0.5 * angle * baseline) / self.radius
        omega_l = (vel - 0.5 * angle * baseline) / self.radius

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        vels = np.array([u_l_limited, u_r_limited])
        return vels

class ResizeWrapper(gym.ObservationWrapper):
    # TODO: Fix with internet!!
    def __init__(self, env=None, shape=(120, 160, 3)):
        super(ResizeWrapper, self).__init__(env)
        self.observation_space.shape = shape
        self.observation_space = spaces.Box(
            self.observation_space.low[0, 0, 0],
            self.observation_space.high[0, 0, 0],
            shape,
            dtype=self.observation_space.dtype)
        self.shape = shape

    def observation(self, observation):
        from PIL import Image
        return np.array(Image.fromarray(observation).resize(self.shape[0:2]))

class NormalizeWrapper(gym.ObservationWrapper):
    def __init__(self, env=None):
        super(NormalizeWrapper, self).__init__(env)
        self.obs_lo = self.observation_space.low[0, 0, 0]
        self.obs_hi = self.observation_space.high[0, 0, 0]
        obs_shape = self.observation_space.shape
        self.observation_space = spaces.Box(0.0, 1.0, obs_shape, dtype=np.float32)

    def observation(self, obs):
        if self.obs_lo == 0.0 and self.obs_hi == 1.0:
            return obs
        else:
            return (obs - self.obs_lo) / (self.obs_hi - self.obs_lo)


class ImgWrapper(gym.ObservationWrapper):
    def __init__(self, env=None):
        super(ImgWrapper, self).__init__(env)
        obs_shape = self.observation_space.shape
        self.observation_space = spaces.Box(
            self.observation_space.low[0, 0, 0],
            self.observation_space.high[0, 0, 0],
            [obs_shape[2], obs_shape[0], obs_shape[1]],
            dtype=self.observation_space.dtype)

    def observation(self, observation):
        return observation.transpose(2, 0, 1)

def seedall(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)
    random.seed(seed)

def force_done(env):
    env.done = True

def evaluate_policy(env, policy, eval_episodes=10, max_timesteps=500):
    avg_reward = 0.
    for _ in range(eval_episodes):
        obs = env.reset()
        done = False
        step = 0
        while not done and step < max_timesteps:
            action = policy.predict(np.array(obs))
            obs, reward, done, _ = env.step(action)
            avg_reward += reward
            step += 1

    avg_reward /= eval_episodes

    return avg_reward

def plot_poses(poses, goal = False, draw_line=False):
    coords = np.array([p[0] for p in poses])
    xmin = np.min(coords[:, 0])
    xmax = np.max(coords[:, 0])
    ymin = np.min(coords[:, 1])
    ymax = np.max(coords[:, 1])
    
    if goal:
        xmin = np.minimum(xmin, goal[0])
        xmax = np.maximum(xmax, goal[0])
        ymin = np.minimum(ymin, goal[1])
        ymax = np.maximum(ymax, goal[1])

    plt.axis([xmin - 0.1, xmax + 0.1, ymin - 0.1, ymax + 0.1])
    for i, p in enumerate(poses):
        x = p[0][0]
        y = p[0][1]
        arrow_angle = math.radians(p[1])
        
        if i == len(poses) -1:
            plt.arrow(x, y, 0.001 * math.cos(arrow_angle), 0.001 * math.sin(arrow_angle),
                 head_width=0.1, head_length=0.16,
                  fc='r', ec='r')
        else:
            plt.arrow(x, y, 0.001 * math.cos(arrow_angle), 0.001 * math.sin(arrow_angle),
                 head_width=0.1, head_length=0.16,
                  fc='k', ec='k')
            
    if goal:
        plt.arrow(goal[0], goal[1], 0.001 * math.cos(math.radians(goal[2])), 0.001 * math.sin(math.radians(goal[2])), head_width=0.1, head_length=0.16,
                  fc='g', ec='g')
        final_pose = poses[-1]
        if math.fabs(final_pose[0][0] - goal[0]) < 0.01 and math.fabs(final_pose[0][1] - goal[1]) < 0.01 and math.fabs(final_pose[1] - goal[2]) < 0.1:
            print("Goal achieved! Great job!")
            
    if draw_line:
        plt.hlines(0, 0, xmax, linestyles='dashed', colors='r')


def rotate_point(px, py, cx, cy, theta):
    """
    Rotate a 2D point around a center
    """

    dx = px - cx
    dy = py - cy

    new_dx = dx * math.cos(theta) - dy * math.sin(theta)
    new_dy = dy * math.cos(theta) + dx * math.sin(theta)

    return cx + new_dx, cy + new_dy
  
def get_dir_vec(angle):
    """
    Vector pointing in the direction the agent is looking (angle in degrees)
    """
    dir_angle = math.radians(angle)
    x = math.cos(dir_angle)
    y = math.sin(dir_angle)
    return np.array([x, y])

def get_right_vec(angle):
    """
    Vector pointing to the right of the agent (angle in degrees)
    """
    dir_angle = math.radians(angle)

    x = math.sin(dir_angle)
    y = -math.cos(dir_angle)
    return np.array([x, y])

def drive(cur_pos, cur_angle, left_rate, right_rate, wheel_dist, wheel_radius, dt):
    """
    Drive this bad boy
    """
    cur_pos = np.array(cur_pos)
    
    Vl = left_rate * wheel_radius * np.pi * 2
    Vr = right_rate * wheel_radius * np.pi * 2 # rate is in turns/sec
    l = wheel_dist

    # If the wheel velocities are the same, then there is no rotation
    if Vl == Vr:
      cur_pos = cur_pos + dt * Vl * get_dir_vec(cur_angle)
      return cur_pos, cur_angle

    # Compute the angular rotation velocity about the ICC (center of curvature)
    w = (Vr - Vl) / l
    # Compute the velocity
    v = (Vl + Vr) / 2
    

    # Compute the distance to the center of curvature
    r = v/w

    # Compute the rotation angle for this time step
    rotAngle = w * dt

    # Rotate the robot's position around the center of rotation
    r_vec = -get_right_vec(cur_angle)
    px, py = cur_pos
    cx = px + r * r_vec[0]
    cy = py + r * r_vec[1]
    npx, npy = rotate_point(px, py, cx, cy, rotAngle)
    next_pos = np.array([npx, npy])
   
    # Update the robot's direction angle
    next_angle = cur_angle + math.degrees(rotAngle)
    return next_pos, next_angle

def calibrate_drive(cur_pos, cur_angle, gain, trim, dt, seed):
    limit = 1
    l = 0.2                  # 0.2 meters of distance between the wheels
    wheel_radius = 0.03      # radius of the wheels is 0.03 meters (3 cm) as we know it
    
    desired_rate = 1                           # Turns per second
    desired_omega = 2*np.pi * desired_rate     # In radians
    
    desired_omega_r = desired_omega
    desired_omega_l = desired_omega
       
    
    k = 27                                     # Motor constant as we know it
    
    k_r_inv = (gain + trim) / k                # Inverse motor constant after trim
    k_l_inv = (gain - trim) / k                
    
    
    # conversion from motor rotation rate to duty cycle, according to what we know
    u_r = desired_omega_r * k_r_inv
    u_l = desired_omega_l * k_l_inv
    
    # limiting output to limit, which is 1.0 for the duckiebot
    u_r_limited = max(min(u_r, limit), -limit)
    u_l_limited = max(min(u_l, limit), -limit)
    
    # Modelling the real values of radius and motor constants
    np.random.seed(seed)
    radius_r = np.random.uniform(wheel_radius - 0.0015, wheel_radius + 0.0015)
    radius_l = np.random.uniform(wheel_radius - 0.0015, wheel_radius + 0.0015)
    k_l_noisy = np.random.uniform(k - 0.25, k + 0.25)
    k_r_noisy = np.random.uniform(k - 0.25, k + 0.25)

    # Real values of inverse motor constants
    k_r_inv_noisy = 1/k_r_noisy
    k_l_inv_noisy = 1/k_l_noisy

    # Real values of wheel angular velocity
    omega_r_limited = u_r_limited/k_r_inv_noisy
    omega_l_limited = u_l_limited/k_l_inv_noisy
    
    # Real wheel velocities
    Vr = omega_r_limited * radius_r
    Vl = omega_l_limited * radius_l
    

    # If the wheel velocities are the same, then there is no rotation
    if Vl == Vr:
        cur_pos = cur_pos + dt * Vr * get_dir_vec(cur_angle)
        return cur_pos, cur_angle

    
    # Compute the angular rotation velocity about the ICC (center of curvature)
    w = (Vr - Vl) / l
    # Compute the velocity
    v = (Vl + Vr) / 2

    # Compute the distance to the center of curvature
    r = v/w

    # Compute the rotation angle for this time step
    rotAngle = w * dt

    # Rotate the robot's position around the center of rotation
    r_vec = -get_right_vec(cur_angle)
    px, py = cur_pos
    cx = px + r * r_vec[0]
    cy = py + r * r_vec[1]
    npx, npy = rotate_point(px, py, cx, cy, rotAngle)
    next_pos = np.array([npx, npy])
   
    # Update the robot's direction angle
    next_angle = cur_angle + math.degrees(rotAngle)
    
    return next_pos, next_angle

class topViewSimulator(Simulator):
    
    def step(self, action: np.ndarray):
        action = np.clip(action, -1, 1)
        # Actions could be a Python list
        action = np.array(action)
        for _ in range(self.frame_skip):
            self.update_physics(action)

        # Generate the current camera image
        obs = self._render_img(640,
                480,
                self.multi_fbo,
                self.final_fbo,
                np.zeros(shape=self.observation_space.shape, dtype=np.uint8),
                top_down=True)        
        misc = self.get_agent_info()

        d = self._compute_done_reward()
        misc['Simulator']['msg'] = d.done_why

        return obs, d.reward, d.done, misc

def load_env_obstacles(local_env):
    ##### Loading Circular Obstacles
    list_obstacles = []
    for obstacle in local_env.objects:
        pose = obstacle.pos
        robot_radius = 0.1
        radius = obstacle.safety_radius + robot_radius
        list_obstacles.append([pose[0], pose[2], radius])
    
    ###### Non drivable tiles
    for i in range(local_env.grid_width):
        for j in range(local_env.grid_height):
            tile = local_env.grid[j * local_env.grid_width + i]
            if not tile['drivable']:
                coords = tile['coords']
                list_obstacles.append([(coords[0]+.5) * local_env.road_tile_size, (coords[1]+.5) * local_env.road_tile_size, 1.3*local_env.road_tile_size + 2*robot_radius])
    return list_obstacles

def proportional_next_point_controller(local_env, new_path):
    # Desired position is next point of the path
    next_pos = new_path[-2]
    # Current position and angle
    cur_pos_x = local_env.cur_pos[0]
    cur_pos_y = local_env.cur_pos[2]
    cur_angle = local_env.cur_angle

    ### Step 1: turn to reach the right angle
    # Angle that we need to reach
    x_diff = next_pos[0] - cur_pos_x
    y_diff = next_pos[1] - cur_pos_y
    angle_to_be = -1 * np.angle(x_diff + y_diff * 1j) # Negative because of frame mismatch

    # Turning with P control
    while math.fabs(local_env.cur_angle%(2*np.pi) - angle_to_be%(2*np.pi)) > 0.05:
        v = 0.
        omega = (angle_to_be%(2*np.pi) - local_env.cur_angle%(2*np.pi) )
        obs, _, d, _ = local_env.step([v, omega])
        if d:
            print("Crash")
            return d

    ### Step 2: move towards goal
    # Compute distance
    dist =  get_dist_to_goal(local_env.cur_pos, next_pos)
    last_dist = math.inf
    # Move forward until you reach the goal
    while dist > 0.02 and last_dist > dist:
        omega = 0
        v = dist
        obs, _, d, _ = local_env.step([v, omega])
        last_dist = dist
        dist = get_dist_to_goal(local_env.cur_pos, next_pos)
        if d:
            print("Crash")
            return d
    return d
            
def get_dist_to_goal(cur_pos, goal):
    return math.sqrt((cur_pos[0] - goal[0])**2 + (cur_pos[2] - goal[1])**2)