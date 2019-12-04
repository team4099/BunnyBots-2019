package org.usfirst.frc.team4099.robot

import com.team2363.logger.HelixEvents
import com.team2363.logger.HelixLogger
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.TimedRobot
import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.AutoModeExecutor
import org.usfirst.frc.team4099.lib.util.CrashTracker
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.FaultDetector
import org.usfirst.frc.team4099.robot.loops.Looper
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator
import org.usfirst.frc.team4099.robot.subsystems.Drive
import org.usfirst.frc.team4099.robot.subsystems.Vision

class Robot : TimedRobot() {
    private lateinit var autoModeExecutor: AutoModeExecutor

    private val drive = Drive.instance
    private val controlBoard = ControlBoard.instance
    private val disabledLooper = Looper("disabledLooper")
    private val enabledLooper = Looper("enabledLooper")

    init {
        HelixEvents.addEvent("ROBOT", "Robot Construction")
        HelixLogger.addSource("Battery Voltage", RobotController::getBatteryVoltage)

        HelixLogger.addSource("Enabled Looper dT") { enabledLooper.dt }
        HelixLogger.addSource("Disabled Looper dT") { disabledLooper.dt }
    }

    override fun robotInit() {
        try {
            HelixEvents.startLogging()
            CameraServer.getInstance().startAutomaticCapture()

            DashboardConfigurator.initDashboard()

            SubsystemManager.register(drive)
            SubsystemManager.register(Vision)

            enabledLooper.register(SubsystemManager.enabledLoop)
            enabledLooper.register(BrownoutDefender.instance)
            enabledLooper.register(FaultDetector)

            disabledLooper.register(SubsystemManager.disabledLoop)
            disabledLooper.register(VoltageEstimator.instance)
            enabledLooper.register(FaultDetector)

            HelixEvents.addEvent("ROBOT", "Robot Initialized")
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("robotInit", t)
            throw t
        }
    }

    override fun disabledInit() {
        try {
            enabledLooper.stop() // end EnabledLooper
            disabledLooper.start() // start DisabledLooper

            HelixEvents.addEvent("ROBOT", "Robot Disabled")
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("disabledInit", t)
            throw t
        }
    }

    override fun autonomousInit() {
        try {
            autoModeExecutor = AutoModeExecutor()
            HelixEvents.addEvent("ROBOT", "Autonomous Enabled")
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("autonomousInit", t)
            throw t
        }
    }

    override fun teleopInit() {
        try {
            enabledLooper.start()
            HelixEvents.addEvent("ROBOT", "Teleop Enabled")
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("teleopInit", t)
            throw t
        }
    }

    override fun disabledPeriodic() {
        try {
            // outputAllToSmartDashboard()
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("disabledPeriodic", t)
            throw t
        }
    }

    override fun autonomousPeriodic() {
        teleopPeriodic()
    }

    override fun teleopPeriodic() {
        try {
            drive.setCheesyishDrive(
                controlBoard.throttle,
                controlBoard.turn,
                Utils.around(controlBoard.throttle, 0.0, Constants.Joysticks.QUICK_TURN_THROTTLE_TOLERANCE)
            )
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("teleopPeriodic", t)
            throw t
        }
    }

    override fun testInit() {
        try {
            enabledLooper.start()
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("testInit", t)
            throw t
        }
    }

    override fun testPeriodic() {}
}
