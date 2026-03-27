from enum import Enum


class RobotAxis(Enum):
    X = 1
    Y = 2
    Z = 3
    RX = 4
    RY = 5
    RZ = 6

    @classmethod
    def get_by_string(cls, axis_str: str):
        """Convert string to RobotAxis enum instance"""
        axis_upper = axis_str.strip().upper()

        try:
            return cls[axis_upper]  # Returns RobotAxis.Z, not "Z"
        except KeyError:
            raise ValueError(f"Invalid axis: {axis_str}. Valid axes: {[a.name for a in cls]}")

class Direction(Enum):
    """
       Enum representing movement directions along an axis.
       """
    MINUS = -1
    PLUS = 1

    def __str__(self):
        return self.name

    @staticmethod
    def get_by_string(name:str):
        try:
            return Direction[name.upper()]
        except KeyError:
            raise ValueError(f"Invalid Direction name: {name}")