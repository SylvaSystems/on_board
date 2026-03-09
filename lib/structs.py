from dataclasses import dataclass
import numpy as np

@dataclass
class Pose:
    lat:float
    lon:float
    alt:float
    x:float
    y:float
    z:float
    roll:float
    pitch:float
    yaw:float
    timestamp:float
    id:int

@dataclass
class Img:
    img:np.ndarray
    path_name:str
    timestamp:float
    id:int