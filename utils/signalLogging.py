from typing import Callable
import wpilib
import ntcore as nt
import wpiutil.log as wpilog  # pylint: disable=import-error,no-name-in-module
from utils.extDriveManager import ExtDriveManager
from utils.singleton import Singleton


BASE_TABLE = "SmartDashboard"

class Logger():
    """
    Container for holding a data source (a callable in our code)
    and all the places the data might go - NetworkTables for live publishing,
    or a file for later review. If writing files is not possible (USB drive not in?)
    the file publisher should be None
    """
    valGetter:Callable[[], float]|None
    ntPublisher:nt.DoublePublisher
    filePublisher:wpilog.DoubleLogEntry|None

    __slots__ = 'valGetter', 'ntPublisher', 'filePublisher'

    def __init__(self,
                 valGetter:Callable[[], float]|None,
                 ntPublisher:nt.DoublePublisher,
                 filePublisher:wpilog.DoubleLogEntry|None):
        self.valGetter = valGetter
        self.ntPublisher = ntPublisher
        self.filePublisher = filePublisher

    def logNow(self, val):
        curTime = nt._now()  # pylint: disable=W0212
        SignalWrangler.updateALoggerValue(self, val, curTime)

# Wrangler for coordinating the set of all signals
class SignalWrangler(metaclass=Singleton):
    # Starts up logging to file, along with network tables infrastructure
    # Picks appropriate logging directory based on our current target
    def __init__(self):
        # Default to publishing things under Shuffleboard, which makes things more available
        self.table = nt.NetworkTableInstance.getDefault().getTable(BASE_TABLE)
        self.loggedValList:list[Logger] = []
        self.time = int(0)
        self.log = None
    
        if ExtDriveManager().isConnected():
            wpilib.DataLogManager.start(dir=ExtDriveManager().getLogStoragePath())
            wpilib.DataLogManager.logNetworkTables(
                False
            )  # We have a lot of things in NT that don't need to be logged
            self.log = wpilib.DataLogManager.getLog()


    @classmethod
    def updateALoggerValue(cls, lv:Logger, val, curTime):
        lv.ntPublisher.set(val, curTime)
        if (lv.filePublisher is not None):
            lv.filePublisher.append(val, curTime)

    def update(self):
        curTime = nt._now()  # pylint: disable=W0212
        for lv in self.loggedValList:
            if lv.valGetter is not None:
                val = lv.valGetter()
                self.updateALoggerValue(lv, val, curTime)


    def newLoggerSetup(self, name:str, valGetter:Callable[[],float], units:str|None)->Logger:

        # Set up NT publishing
        sigTopic = self.table.getDoubleTopic(name)
        sigPub = sigTopic.publish(
            nt.PubSubOptions(sendAll=True, keepDuplicates=True)
        )
        sigPub.setDefault(0)

        if(units is not None):
            sigTopic.setProperty("units", str(units))

        # Set up log file publishing if enabled
        if self.log is not None:
            sigLog = wpilog.DoubleLogEntry(
                log=self.log, name=sigNameToNT4TopicName(name)
            )
        else:
            sigLog = None

        logger = Logger(valGetter,sigPub, sigLog)
        return logger

    def newLogger(self, name:str, valGetter:Callable[[],float], units:str|None)->Logger:
        logger = self.newLoggerSetup(name, valGetter, units)

        self.loggedValList.append(
            logger
        )
        return logger




###########################################
# Public API
###########################################

def logUpdate():
    """
    Periodic call to sample and broadcast all logged values. Should happen once per 
    20ms loop.
    """
    SignalWrangler().update()

def addLog(alias: str, valueGetter: Callable[[], float], units:str|None=None) -> None:
    """
    Register a callable getter to be logged at alias

    Parameters:
    - alias: The name (str) used to identify the log.
    - valueGetter: A function that returns the current value of the log. Lambda is acceptable here.
    - units: The units (str) of the value_getter
    """
    SignalWrangler().newLogger(alias, valueGetter, units)

def log(alias: str, valueGetter, units:str|None=None) -> None:
    addLog(alias, valueGetter, units)

def getNowLogger(alias: str, units:str|None=None) -> Logger:
    logger = SignalWrangler().newLoggerSetup(alias, None, units)
    return logger

def sigNameToNT4TopicName(name):
    return f"/{BASE_TABLE}/{name}"
