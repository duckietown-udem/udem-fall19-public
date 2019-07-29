from dataclasses import dataclass
from typing import Tuple

import numpy as np

from aido_schemas import EpisodeStart, protocol_agent_duckiebot1, PWMCommands, Duckiebot1Commands, LEDSCommands, RGB, \
    wrap_direct, Context, Duckiebot1Observations, JPGImage, Context

# from simulation.gym_duckietown.simulator import Simulator

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

def launch_env(simclass=None):
    from simulation.gym_duckietown.simulator import Simulator

    simclass = Simulator if simclass is None else simclass

    env = simclass(
        seed=123, # random seed
        map_name="loop_empty",
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
    
    repo = Repo('./simulation')
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