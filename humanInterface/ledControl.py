from wpilib import PWMMotorController
from utils.constants import LED_STACK_LIGHT_CTRL_PWM
from utils.singleton import Singleton
from wpimath.filter import Debouncer
from wpilib import Timer
from wpilib import RobotController, DriverStation

BLINK = -1.0
GREEN = 0.35
RED = 0.03
ORANGE = 0.07
YELLOW = 0.15
BLUE = 0.75
OFF = 0.0

class LEDControl(metaclass=Singleton):
    """
    Simple control class for the small, custom LED boards that we've used for a few years now.
    Those LED boards are designed to take in a PWM signal, just like a motor controller.
    Zero is off, increasing magnitidue cycles through different hue's.
    Positive numbers are solid, negative numbers are blinky. 

    See https://github.com/gerth2/MiniLEDBar/tree/v2_casserole_light_board for source code for the light bar itself
    See https://oshwlab.com/chrisgerth010592/miniprogrammablelightbar_copy for hardware info
    """
    def __init__(self):

        self._isAutoDrive = False
        self._isStuck = False
        self._coralInterfers = False
        self.stuckDebounce = Debouncer(0.3, Debouncer.DebounceType.kFalling)
        self.ledPWMOutput = PWMMotorController("LEDCtrl", LED_STACK_LIGHT_CTRL_PWM)

    def update(self):
        """
        Pick a color to send to the LED based on the robot's status
        """
        stuckDebounced = self.stuckDebounce.calculate(self._isStuck)

        if(self._isAutoDrive):
            # Autos - blue for normal, blinky red for issues
            if(stuckDebounced):
                pwmVal = RED * BLINK
            else:
                pwmVal = BLUE
        else:
            # Manual
            # Indicate coral interference and endgame
            if(self._coralInterfers):
                pwmVal = YELLOW * BLINK
            else:
                if( self.isEndgame() ):
                    # Endgame - solid Orange-ish?
                    pwmVal = ORANGE
                else:
                    # Nominal enabled. Green is good!
                    pwmVal = GREEN

        self.ledPWMOutput.set(pwmVal)

    def isEndgame(self) -> bool:
        matchTime = Timer.getMatchTime()
        if(DriverStation.isTeleop() and matchTime < 20.0 and matchTime >= 0.0):
            return True
        else:
            return False

    def setAutoDrive(self, isAutoDrive:bool):
        """
        Set whether the LED should change color to indicate we are doing auto-drive now
        """
        self._isAutoDrive = isAutoDrive

    def setStuck(self, isStuck:bool):
        """
        Set whether the LED should change color to indicate we are stuck while auto-driving
        """
        self._isStuck = isStuck

    def setCoralInterferencePossible(self, isPossible:bool):
        """
        Set whether the LED should change color to indicate coral will prohibit elevator motion
        """
        self._coralInterfers = isPossible