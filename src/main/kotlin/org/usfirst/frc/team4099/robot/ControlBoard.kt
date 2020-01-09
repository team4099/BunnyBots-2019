package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.joystick.Gamepad
import org.usfirst.frc.team4099.lib.joystick.XboxOneGamepad

@Suppress("MagicNumber")
class ControlBoard private constructor() {
    private val driver: Gamepad = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
    private val operator: Gamepad = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)

    val throttle: Double
        get() = driver.rightTriggerAxis - driver.leftTriggerAxis

    val turn: Double
        get() = -driver.leftXAxis

    val aim: Boolean
        get() = driver.aButton

    val shoot: Boolean
        get() = operator.xButton

    val overrideShoot: Boolean
        get() = operator.yButton

    companion object {
        val instance = ControlBoard()
    }
}
