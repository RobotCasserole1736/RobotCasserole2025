import wpilib
from utils.faults import Fault
from utils.singleton import Singleton
from phoenix6.orchestra import Orchestra
from phoenix6.hardware.parent_device import ParentDevice


class CTREMusicPlayback(metaclass=Singleton):
    def __init__(self):
        self.loadFault = Fault("Call Me Maybe Unavailable")
        self.orch = Orchestra()
        status = self.orch.load_music("/home/lvuser/py/deploy/callMeMaybe.chrp")
        print(status.description)
        self.loadFault.set(status.is_error())

    def registerDevice(self, device:ParentDevice):
        self.orch.add_instrument(device)

    def play(self):
        self.orch.play()

    def stop(self):
        self.orch.stop()

    def isPlaying(self) -> bool:
        return self.orch.is_playing() and wpilib.DriverStation.isTestEnabled()
