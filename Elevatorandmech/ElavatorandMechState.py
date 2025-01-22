from utils.calibration import Calibration
from utils.singleton import Singleton
from ElevatorControl import ElevatorControl
class ElevatorandMechState(metaclass=Singleton):

    def __init__(self):
        self.elevCtrl = ElevatorControl()

        # Fixed Position Cal's
        self.L1_Height = Calibration(name="height of L1", units="m", default=SINGER_ABS_ENC_OFF_DEG)
        self.L2_Height = Calibration(name="height of L2", units="m", default=-40.0 )
        self.L3_Height = Calibration(name="height of L3", units="m", default=-20.0 )
        self.L4_Height = Calibration(name="height of L4", units="m", default=57.0)

