from PIL import Image, ImageOps
import numpy as np

from dataclasses import dataclass
from abc import ABC, abstractmethod

@dataclass 
class Env(ABC):
    """
    Abstract base class to represent the environment in a model road network.

    width: width of the track in meters
    height: height of the track in meters
    """
    width = 1
    height = 1
    track = None

    @abstractmethod
    def getObstacleXYArrays(self) -> tuple[list[int], list[int]]:
        raise NotImplementedError
    

@dataclass 
class RasterEnv(Env):
    """
    An environment which stores the track internally as a rasterised grid,
    i.e. a 2D array of binary values where obstacles are (True) 
    """

    def getTrackFromImage(self, path: str, invert=False) -> None:
        """
        Import the track data from an image file.

        path: Path to the image file
        invert: Obstacles are represented by black pixels by default, but you can override that
                with invert=True
        """
        img = Image.open(path)
        img = img.convert("1")  # convert to bilevel

        if invert:  # invert binary values
            img = ImageOps.invert(img)

        self.track = np.array(img)  # convert to numpy array

    def getObstacleXYArrays(self) -> tuple[list[int], list[int]]:
        """
        Get the obstacle arrays in the format specified in PythonRobotics.
        """
        if self.track is None:
            raise ValueError("Track array no initialised")
        
        ox, oy = [], []
        it = np.nditer(self.track, flags=['multi_index'])

        for x in it:
            if x:
                ox.append(it.multi_index[1])  # these are intentionally back to front
                oy.append(it.multi_index[0])

        return (ox, oy)


def main():
    env = RasterEnv()
    env.getTrackFromImage('mrn_bin.png')

    (ox, oy) = env.getObstacleXYArrays()

    print(len(ox), len(oy))

if __name__ == "__main__":
    main()

    
