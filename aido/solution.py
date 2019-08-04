from utils.helpers import AIDOSubmission, submit_aido, jpg2rgb
from aido_schemas import JPGImage, Duckiebot1Observations

from custom.custom_helpers import encouragement
import numpy as np

class YourAIDOSubmission(AIDOSubmission):
    def on_received_observations(self, data: Duckiebot1Observations):
        camera: JPGImage = data.camera
        obs = jpg2rgb(camera.jpg_data)
        # Add random value to each pixel
        self.current_image = obs + np.random.random() 

    def compute_action(self, observation):
        encouragement()
        action = np.random.random((2,)) + 1
        return action.astype(float)

if __name__ == '__main__':
    submission = YourAIDOSubmission()
    submit_aido(submission)
