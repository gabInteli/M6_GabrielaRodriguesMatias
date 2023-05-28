from Pose import Pose
from collections import deque

# Define a classe MissionControl, que representa a fila de pontos a serem seguidos
class MissionControl(deque):
    
    # Define a sequência de pontos para a formação da letra G 
    shape = [Pose(-1.0, 1.0), Pose(-2.0, 0.0), Pose(-1.0, -1.0), Pose(0.0, -0.3), Pose(-0.7, -0.45)]

    def __init__(self):
        super().__init__()
        for pose in MissionControl.shape:
            self.enqueue(pose)
        
    def enqueue(self, x):
        super().append(x)
    
    def dequeue(self):
        return super().popleft()