package org.usfirst.frc.team4099.auto.actions

import kotlin.math.abs
import kotlin.math.sign
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.subsystems.Drive

/**
 * Created by Oksana on 2/16/2017.
 */
class TurnAction(angleToTurn: Double, slowMode: Boolean) : Action {
    private val drive = Drive.instance
    private val direction = sign(angleToTurn)
    private val angleToTurn = abs(angleToTurn)
    private var power = Constants.Autonomous.TURN_POWER
    private var startAngle = 0.0
    private var done = false

    private val turnSignal = DriveSignal(direction * power, -direction * power)

    init {
        if (slowMode) {
            this.power = Constants.Autonomous.SLOW_TURN_POWER
        }
    }

    override fun isFinished(): Boolean {
        return abs(drive.angle - startAngle) >= angleToTurn || done
    }

    override fun update() {
        drive.setOpenLoop(turnSignal)
    }

    override fun done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL)
        println("------- END TURN -------")
    }

    override fun start() {
        startAngle = drive.angle
        println("------- NEW START AUTONOMOUS RUN -------")
        println("Starting angle: $startAngle")
    }
}
