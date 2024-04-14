"""
TODO: 

We would rather have the outline of an obstacle rather than a solid block of dots
when outputting ox,oy arrays [DONE]

Need some way of downsampling an imput image [DONE]

Need some way of tracking dimensions [DONE]
"""

from PIL import Image, ImageOps, ImageFilter
import numpy as np
import matplotlib.pyplot as plt

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
    cell_size = 0.01

    def __down_sample(self, img: Image.Image, pct: float) -> Image.Image:
        """
        Down-sample an image.

        pct: the length factor we want to down sample to i.e. 80% => (200, 100)px -> (160, 80)px
        """
        w, h = img.width, img.height
        target_width = int(w * (pct / 100))
        target_height = int(h * (pct / 100))

        return img.resize((target_width, target_height))  # do downsampling

    def getTrackFromImage(self, path: str, invert=False, 
                          target_width=None, target_height=None, ds_pct=None) -> None:
        """
        Import the track data from an image file.

        path: Path to the image file
        invert: Obstacles are represented by black pixels by default, but you can override that
                with invert=True
        target_width: real world dimension of the input image [m]
        target_height: real world dimension of the input image [m]
        ds_pct: down-sampling percentage if we wish to down-sample the image
        """
        if target_width is not None and target_height is not None:
            raise ValueError("Only one of 'target_width' and 'target_height can be specified.")
        
        img = Image.open(path)
        img = img.convert("1")  # convert to bilevel

        if invert:  # invert binary values
            img = ImageOps.invert(img)

        if ds_pct is not None:
            img = self.__down_sample(img, ds_pct)  # do downsampling

        # img = img.filter(ImageFilter.FIND_EDGES)  # only edges of obstacles are stored

        if target_width is not None:  # adjust scale properties of the environment
            self.width = target_width
            self.cell_size = target_width / img.width
            self.height = img.height * self.cell_size
        elif target_height is not None:
            self.height = target_height
            self.cell_size = target_height / img.height
            self.width = img.width * self.cell_size
        else:
            self.width = img.width * self.cell_size
            self.height = img.height * self.cell_size

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
    env.getTrackFromImage('mrn_bin.png', target_height=4)

    (ox, oy) = env.getObstacleXYArrays()

    plt.plot(ox, oy, ".k")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()

    
