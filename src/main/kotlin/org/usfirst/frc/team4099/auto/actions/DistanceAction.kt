package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.subsystems.Drive
import kotlin.math.abs
import kotlin.math.sign

class DistanceAction(inchesToMove: Double, slowMode: Boolean) : Action {
    private val drive = Drive.instance
    private val direction = sign(inchesToMove)
    private val inchesToMove: Double = abs(inchesToMove)
    private var startDist = 0.0
    private var otherStart = 0.0
    private var power = Constants.Autonomous.FORWARD_POWER
    private var startAngle = 0.0
    private var done = false
    private var startTime = 0.0

    init {
        if (slowMode) {
            this.power = Constants.Autonomous.SLOW_FORWARD_POWER
        }
    }

    override fun isFinished(): Boolean {
        return abs(drive.getLeftDistanceInches()) - startDist >= inchesToMove ||
            abs(drive.getRightDistanceInches()) - otherStart >= inchesToMove ||
            done ||
            Timer.getFPGATimestamp() - startTime > Constants.Autonomous.FORWARD_MAX_TIME_SECONDS
    }

    override fun update() {
        val correctionAngle = startAngle - drive.angle
        if (abs(correctionAngle) > Constants.Autonomous.FORWARD_GIVE_UP_ANGLE) {
            done = true
            return
        }
        drive.arcadeDrive(power * direction,
            correctionAngle * Constants.Autonomous.FORWARD_CORRECTION_KP * direction)
        println("correctionAngle: $correctionAngle")
        SmartDashboard.putNumber("distanceInAction", abs(drive.getRightDistanceInches()) - otherStart)
    }

    override fun done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL)
        println("------- END DISTANCE -------")
    }

    override fun start() {
        startTime = Timer.getFPGATimestamp()
        startAngle = drive.angle
        println("------- NEW START AUTONOMOUS RUN -------")
        println("Starting angle: $startAngle")
        startDist = drive.getLeftDistanceInches()
        otherStart = drive.getRightDistanceInches()
    }
}
