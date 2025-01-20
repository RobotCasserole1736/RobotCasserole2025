import wpilib
import wpimath 
from utils.singleton import Singleton
from enum import Enum
import utils.constants
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class AlgaeIntakeState(Enum):
   DISABLED = 0
   INTAKE = 1
   EJECT = 2
   HOLDING = 3

class AlgaeWristState(Enum):
    DISABLED = 4
    INTAKEOFFGROUND = 5
    STOW = 6
    REEF = 7

class AlgaeWristControl(metaclass=Singleton):

    def __init__(self):
        #one important assumption we're making right now is that we don't need limits on the algae manipulator based on elevator height
        
        self.WristCurState = AlgaeWristState.DISABLED
        self.wristMotor = WrapperedSparkMax(16, "CoralWristMotor", True, 10)
        self.curMotorVoltage = 0.0

        #set the degree details for your goals. For stow, intake off gorund, etc. 
        #Also, the absolute offset will be a constant that you can set here or just have in constants 

    def update(self):
        
        #this is where you will figure out where you're trying to go. See tunerAngleControl.py for a simple answer
        #this could work if it's light enough. But you might have to go to something more like singerAngleControl.py
        self.algaeCurState = AlgaeIntakeState.HOLDING
        
     
    def setGoal(self):
        #here is where you will want to set which angle you're trying to go to. See 2024 tunerAngleControl.py
        pass
         
         
class AlgeaIntakeControl(metaclass=Singleton):

    def __init__(self):
        #initialize all of your variables, spark maxes, etc. 
        self.algaeCurState = AlgaeIntakeState.DISABLED
        self.algaeMotor = WrapperedSparkMax(15, "CoralIntakeMotor", True, 10)

        #also, you want voltages set for each of your states. Ex. how many volts do you want algae eject to be at? 
        #for this, look at lines 63-65 of gamepieceHandling.py in our 2024 code. 

        #add the sensor. Is it a time of flight, what can ID, etc. 
        self.gamepieceSensor = None # game piece sensor empty for now

        #also, if we are using a sensor in the corner to detect algae, add thresholds for that. See gamepiecehandling.py 2024, lines 73-77
        #ask Abby and she'll explain better

        #and add a fault for if the TOF is disconnected. 

    def update(self):

        #update TOF (is it faulted? Is there a gamepiece being seen (see checkGamePiece function)?)
        self.hasGamePiece = self.checkForAlgae() #Check if we are holding a piece of coral. this way it will automaticly switch to holding mode
        #if there's a game piece being seen, set the hold voltage. 
        if self.hasGamePiece:
            #call the hold function
        # elif the cur state is AlgaeIntakeState.INTAKE
            #call the intake function
        #etc etc
            pass

    def setInput(self, intakeBool, ejectBool):
        #this will take in your inputs from the outside world and learn what state we are currently in
        #as an enum, which I'm not sure is the right way to do it. I'd rather have multiple booleans. it'd make calculations easier
        if intakeBool:
            self.algaeCurState = AlgaeIntakeState.INTAKE
        elif ejectBool:
            self.algaeCurState = AlgaeIntakeState.EJECT
        else:
            self.algaeCurState = AlgaeIntakeState.DISABLED
    
    def checkForAlgae(self):
        #this can be a function we use in this file to see if we think there is a game piece, based of TOF
        #if the time of flight is not faulted, check if the measurement (will be in update) is between our max and min calibrations for if there's an algae.
        #  if so, then return true (that we have an algae) If not, return false.
        if self.gamepieceSensor:
            return True
        else:
            return False 
    
    def updateIntake(self, run):
        #run is a boolean. It's either true or false
        #if run, then set algae motor voltage to the intake one
        #self.algaeMotor.setVoltage(___) if ___ is the voltage calibration you made earlier
        #otherwise, set it to 0
        pass

    def updateEject(self, run):
        #run is a boolean
        #if run, the set algae motor voltage to the eject one
        #see updateIntake comments
        pass
   
    def updateHold(self, run):
        #run is boolean
        #see updateIntake comments
        pass

    #we also might want a getHasAlgae function to use later in other areas. 