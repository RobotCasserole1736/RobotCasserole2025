import sys
# from phoenix6 import SignalLogger
from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboard import Dashboard
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.autoSteer import AutoSteer
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from Elevatorandmech.algaeManipulatorControl import AlgeaIntakeControl, AlgaeWristControl
from Elevatorandmech.coralManipulatorControl import CoralManipulatorControl
from Elevatorandmech.ElevatorControl import ElevatorControl
from humanInterface.driverInterface import DriverInterface
from humanInterface.ledControl import LEDControl
from memes.ctreMusicPlayback import CTREMusicPlayback
from humanInterface.operatorInterface import OperatorInterface
from navigation.forceGenerators import PointObstacle
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.calibration import CalibrationWrangler
from utils.crashLogger import CrashLogger
from utils.faults import FaultWrangler
from utils.powerMonitor import PowerMonitor
from utils.rioMonitor import RIOMonitor
from utils.signalLogging import logUpdate
from utils.singleton import destroyAllSingletonInstances
from webserver.webserver import Webserver
import wpilib
from wpimath.geometry import Translation2d, Pose2d, Rotation2d

class MyRobot(wpilib.TimedRobot):

    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        remoteRIODebugSupport()

        self.crashLogger = CrashLogger()

        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.
        wpilib.LiveWindow.disableAllTelemetry()

        self.webserver = Webserver()

        self.algaeWrist = AlgaeWristControl()
        self.algaeIntake = AlgeaIntakeControl()
        self.coralMan = CoralManipulatorControl()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()
        self.autosteer = AutoSteer()

        self.stt = SegmentTimeTracker()

        self.dInt = DriverInterface()
        self.oInt = OperatorInterface()

        self.ledCtrl = LEDControl()

        self.autoSequencer = AutoSequencer()

        self.dashboard = Dashboard()

        self.rioMonitor = RIOMonitor()
        self.pwrMon = PowerMonitor()

        self.elev = ElevatorControl()

        self.algaeManip = AlgaeWristControl()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.addPeriodic(FaultWrangler().update, 0.2, 0.0)

        self.autoHasRun = False

    def robotPeriodic(self):
        self.stt.start()

        self.algaeIntake.update()
        self.stt.mark("algaeIntake")

        self.algaeWrist.update()
        self.stt.mark("algaeWrist")

        self.dInt.update()
        self.stt.mark("Driver Interface")

        self.driveTrain.update()
        self.stt.mark("Drivetrain")

        self.oInt.update()
        self.stt.mark("Operator Interface")

        self.coralMan.update()
        self.stt.mark("Coral Manipulator")

        self.elev.setSafeToLeaveL1(self.coralMan.getCoralSafeToMove())
        self.ledCtrl.setCoralInterferencePossible(not self.coralMan.getCoralSafeToMove())

        self.elev.update()
        self.stt.mark("Elevator")

        # TODO
        #self.coralMan.setAtL1()


        self.autodrive.updateTelemetry()
        self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(self.autodrive.getWaypoints())
        self.driveTrain.poseEst._telemetry.setCurObstacles(self.autodrive.rfp.getObstacleStrengths())
        self.stt.mark("Telemetry")


        self.ledCtrl.setAutoDrive(self.autodrive.isRunning())
        self.ledCtrl.setStuck(self.autodrive.rfp.isStuck())
        self.ledCtrl.update()
        self.stt.mark("LED Ctrl")

        logUpdate()
        self.stt.end()

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self):

        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous rouines starting pose to init the pose estimator
        startPose = self.autoSequencer.getStartingPose()
        if(startPose is not None):
            self.driveTrain.poseEst.setKnownPose(startPose)

        # Mark we at least started autonomous
        self.autoHasRun = True

    def autonomousPeriodic(self):

        self.autoSequencer.update()

        # Operators cannot control in autonomous
        #self.driveTrain.setManualCmd(DrivetrainCommand())

    def autonomousExit(self):
        self.autoSequencer.end()

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self):
        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(
                Pose2d(1.0, 1.0, Rotation2d(0.0))
            )


    def teleopPeriodic(self):

        # TODO - this is technically one loop delayed, which could induce lag
        driverCmd = self.dInt.getCmd()
        driverCmd.scaleBy(self.elev.getDtSpeedLimitFactor())
        self.driveTrain.setManualCmd(driverCmd)

        self.algaeIntake.setInput(self.oInt.getIntakeAlgae(),self.oInt.getEjectAlgae())

        self.algaeManip.setDesPos(self.oInt.getAlgaeManipCmd())

        if self.dInt.getGyroResetCmd():
            self.driveTrain.resetGyro()

        #if self.dInt.getCreateObstacle():
        #    # For test purposes, inject a series of obstacles around the current pose
        #    ct = self.driveTrain.poseEst.getCurEstPose().translation()
        #    tfs = [
        #        #Translation2d(1.7, -0.5),
        #        #Translation2d(0.75, -0.75),
        #        #Translation2d(1.7, 0.5),
        #        Translation2d(0.75, 0.75),
        #        Translation2d(2.0, 0.0),
        #        Translation2d(0.0, 1.0),
        #        Translation2d(0.0, -1.0),
        #    ]
        #    for tf in tfs:
        #        obs = PointObstacle(location=(ct+tf), strength=0.5)
        #        self.autodrive.rfp.addObstacleObservation(obs)

        self.autosteer.setReefAutoSteerCmd(self.dInt.getAutoSteer())
        self.autodrive.setRequest(self.dInt.getAutoDrive())

        self.coralMan.setCoralCmd(self.oInt.getCoralCmd())
        self.coralMan.setAtL1(self.elev.getHeightM() < (self.elev.L1_Height.get() + 0.1))

        self.elev.setSafeToLeaveL1(self.coralMan.getCoralSafeToMove())
        self.elev.setManualAdjCmd(self.oInt.getElevManAdjCmd())
        self.elev.setHeightGoal(self.oInt.getElevCmd())

        # No trajectory in Teleop
        Trajectory().setCmd(None)

    #########################################################
    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()

    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    #########################################################
    ## Test-Specific init and update
    def testInit(self):
        wpilib.LiveWindow.setEnabled(False)
        CTREMusicPlayback().play()

    def testPeriodic(self):
        pass

    def testExit(self) -> None:
        CTREMusicPlayback().stop()

    #########################################################
    ## Cleanup
    def endCompetition(self):

        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        rioMonitorExists = getattr(self, "rioMonitor", None)
        if rioMonitorExists is not None:
            self.rioMonitor.stopThreads()

        destroyAllSingletonInstances()
        super().endCompetition()

def remoteRIODebugSupport():
    if __debug__ and "run" in sys.argv:
        print("Starting Remote Debug Support....")
        try:
            import debugpy  # pylint: disable=import-outside-toplevel
        except ModuleNotFoundError:
            pass
        else:
            debugpy.listen(("0.0.0.0", 5678))
            debugpy.wait_for_client()
