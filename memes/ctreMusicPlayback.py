from utils.singleton import Singleton
from phoenix6.orchestra import Orchestra
from phoenix6.hardware.parent_device import ParentDevice


class CTREMusicPlayback(metaclass=Singleton):
    def __init__(self):
        self.orch = Orchestra(filepath="callMeMaybe.chirp")

    def registerDevice(self, device:ParentDevice):
        self.orch.add_instrument(device)

    def play(self):
        self.orch.play()

    def stop(self):
        self.orch.stop()
