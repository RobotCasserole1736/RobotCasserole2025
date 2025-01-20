
from wrappers.wrapperedPulseWidthEncoder import WrapperedPulseWidthEncoder

""" 
Wrappers a CTRE SRX Magnetic absolute encoder
https://store.ctr-electronics.com/srx-mag-encoder/
Assumes the absolute-angle signal from the encoder has 
been connected to a DIO port on the RoboRIO.
"""


class WrapperedRevThroughBoreEncoder(WrapperedPulseWidthEncoder):
    def __init__(self, port, name, mountOffsetRad, dirInverted):
        WrapperedPulseWidthEncoder.__init__(self,
            port=port,
            name=name,
            mountOffsetRad=mountOffsetRad,
            dirInverted=dirInverted,
            minPulseSec=1e-6,
            maxPulseSec=1025e-6,
            minAcceptableFreqHz=0.9 / 1025e-6)
