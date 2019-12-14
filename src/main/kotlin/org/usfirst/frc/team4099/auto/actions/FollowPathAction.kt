package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.auto.paths.FieldPaths
import org.usfirst.frc.team4099.auto.paths.Path
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.subsystems.Drive

class FollowPathAction(path: FieldPaths) : Action {
    private val drive: Drive = Drive.instance
    private val pathF = Path(path)

    override fun start() {
        drive.enablePathFollow(pathF)
    }

    override fun update() {}

    override fun done() {
        drive.setOpenLoop(DriveSignal.NEUTRAL)
    }

    override fun isFinished(): Boolean {
        return drive.isPathFinished()
    }
}
