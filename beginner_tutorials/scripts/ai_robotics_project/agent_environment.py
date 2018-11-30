import numpy as np


class agent_environment:
    def __init__(self):
        # double array aanmaken
        self.static_map = np.array([[.0 for _ in range(4)] for _ in range(4)])
        # map vullen met xy "coordinaten/posities"
