
import wpilib
from utils.constants import SHOOTER_EJCT_PWM, SHOOTER_INT_PWM
from utils.signalLogging import addLog
from utils.singleton import Singleton


class BallShooterControl(metaclass=Singleton):
    def __init__(self):
        self.intakeCommandState = False
        self.ejectCommandState = False
        self.shooterRMotor = wpilib.Spark(1)
        self.shooterLMotor = wpilib.Spark(0)

        addLog("Algae Manipulator intake cmd",lambda:self.intakeCommandState,"Bool")
        addLog("Algae Manipulator  cmd",lambda:self.ejectCommandState,"Bool")

    def update(self):

        if self.intakeCommandState:
            self.updateIntake(True)
        elif self.ejectCommandState:
            self.updateEject(True)
        else:
            self.shooterLMotor.set(-.5)
            self.shooterRMotor.set(.5)

    def setInput(self, intakeBool, ejectBool):
        self.intakeCommandState = intakeBool
        self.ejectCommandState = ejectBool

    def updateIntake(self, run, ):
        voltage = -.8

        if run:
            self.shooterRMotor.set(voltage)
            self.shooterLMotor.set(-voltage)
        else: 
            self.shooterRMotor.set(0)
            self.shooterLMotor.set(0)
    
    def updateEject(self, run):
        voltage = .8

        if run:
            self.shooterRMotor.set(voltage)
            self.shooterLMotor.set(-voltage)
        else:
            self.shooterLMotor.set(0)
            self.shooterRMotor.set(0)