import wpilib
from AutoSequencerV2.autoSequencer import AutoSequencer
from Elevatorandmech import coralManipulatorControl, algaeManipulatorControl
from Elevatorandmech.ElevatorControl import ElevatorControl
from dashboardWidgets.autoChooser import AutoChooser
from dashboardWidgets.circularGauge import CircularGauge
from dashboardWidgets.swerveState import SwerveState
from dashboardWidgets.reefIndicator import ReefIndicator
from dashboardWidgets.icon import Icon
from dashboardWidgets.text import Text
from utils.faults import FaultWrangler
from utils.signalLogging import addLog
from utils.units import m2ft
from webserver.webserver import Webserver
from drivetrain.controlStrategies.autoSteer import AutoSteer
from drivetrain.controlStrategies.autoDrive import AutoDrive


class Dashboard:
    def __init__(self):
        webServer = Webserver()

        #the reef/elevator dial on the lefthand side. 
        webServer.addDashboardWidget(ReefIndicator(15, 15, "/SmartDashboard/reefGoalPosIdx"))
        webServer.addDashboardWidget(
            CircularGauge(15, 55, "/SmartDashboard/ElevatorHeight", 0, 6, -1, 7))

        #all the indicators in the middle. Top row, then bottom row
        webServer.addDashboardWidget(Icon(35, 45, "/SmartDashboard/isautoSteerState", "#9632bf", "autoSteer"))
        webServer.addDashboardWidget(Icon(45, 45, "/SmartDashboard/isRedIconState", "#FF0000", "allianceRed"))
        webServer.addDashboardWidget(Icon(55, 45, "/SmartDashboard/isBlueIconState", "#0000FF", "allianceBlue"))
        webServer.addDashboardWidget(Icon(65, 45, "/SmartDashboard/PE Vision Targets Seen", "#00FF00", "vision"))
        webServer.addDashboardWidget(Icon(45, 55, "/SmartDashboard/hasCoral", "#FFFFFF", "coral"))
        webServer.addDashboardWidget(Icon(55, 55, "/SmartDashboard/hasAlgae", "#00FF00", "algae"))
        webServer.addDashboardWidget(Icon(65, 55, "/SmartDashboard/faultIcon", "#FF2200", "warning"))

        #the fault descriptions
        webServer.addDashboardWidget(Text(50, 75, "/SmartDashboard/faultDescription"))

        #swerve states icons
        webServer.addDashboardWidget(SwerveState(85, 15))

        #auto stuff
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                10,
                AutoSequencer().getDelayModeNTTableName(),
                AutoSequencer().getDelayModeList(),
            )
        )
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                20,
                AutoSequencer().getFlipModeNTTableName(),
                AutoSequencer().getFlipModeList(),
            )
        )
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                30,
                AutoSequencer().getMainModeNTTableName(),
                AutoSequencer().getMainModeList(),
            )
        )

        # Now, this is the stuff that updates the dashboard, through logs
        addLog("isautoSteerState",  
               lambda: (
            Icon.kON if AutoSteer().autoSteerIsRunning()
            else Icon.kOFF)
        )

        addLog("ElevatorHeight", lambda: (m2ft(ElevatorControl().getHeightM())))

        addLog("hasCoral",  
               lambda: (
            Icon.kON if coralManipulatorControl.CoralManipulatorControl().getCheckGamePiece()
            else Icon.kOFF)
        )

        addLog("isRedIconState",  
               lambda: (
            Icon.kON if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed 
            else Icon.kOFF)
        )

        addLog("isBlueIconState", 
            lambda: (
            Icon.kON if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue 
            else Icon.kOFF)
        )

        addLog("faultIcon",
                lambda: (Icon.kBLINK_FAST if FaultWrangler().hasActiveFaults() else Icon.kOFF)
        )


        # Test Only.
        # TODO: Real data
        addLog("reefGoalPosIdx",
                lambda: (AutoDrive().getDashTargetPositionIndex()) #Bottom is the side facing our driver station.
        )


