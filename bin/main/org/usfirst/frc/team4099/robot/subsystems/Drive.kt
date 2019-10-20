package org.usfirst.frc.team4099.robot.subsystems

import java.util.*

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.auto.paths.FieldPaths
import org.usfirst.frc.team4099.auto.paths.Path
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.CANMotorControllerFactory
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop


class Drive private constructor() : Subsystem {

    private val leftMasterSRX: TalonSRX = CANMotorControllerFactory.createDefaultTalon(Constants.Drive.LEFT_MASTER_ID)
    private val leftSlave1SPX: VictorSPX = CANMotorControllerFactory.createPermanentSlaveVictor(Constants.Drive.LEFT_SLAVE_1_ID, leftMasterSRX)
    private val leftSlave2SRX: TalonSRX = CANMotorControllerFactory.createPermanentSlaveTalon(Constants.Drive.LEFT_SLAVE_2_ID, Constants.Drive.LEFT_MASTER_ID)
    private val rightMasterSRX: TalonSRX = CANMotorControllerFactory.createDefaultTalon(Constants.Drive.RIGHT_MASTER_ID)
    private val rightSlave1SPX: VictorSPX = CANMotorControllerFactory.createPermanentSlaveVictor(Constants.Drive.RIGHT_SLAVE_1_ID, rightMasterSRX)
    private val rightSlave2SRX: TalonSRX = CANMotorControllerFactory.createPermanentSlaveTalon(Constants.Drive.RIGHT_SLAVE_2_ID, Constants.Drive.RIGHT_MASTER_ID)

    private val pneumaticShifter: DoubleSolenoid = DoubleSolenoid(Constants.Drive.SHIFTER_FORWARD_ID, Constants.Drive.SHIFTER_REVERSE_ID)

//    private val test1 : DoubleSolenoid = DoubleSolenoid(2,5)
//    private val test2 : DoubleSolenoid = DoubleSolenoid(3,4)
   // private val test3 : DoubleSolenoid = DoubleSolenoid(1,6)

    private val ahrs: AHRS

    private var path: Path

    private var segment: Int
    private var trajLength: Int
    private var lastLeftError: Double
    private var lastRightError: Double


    var brakeMode: NeutralMode = NeutralMode.Coast //sets whether the break mode should be coast (no resistance) or by force
        set(type) {
            if (brakeMode != type) {
                rightMasterSRX.setNeutralMode(type)
                rightSlave1SPX.setNeutralMode(type)
                rightSlave2SRX.setNeutralMode(type)
                leftMasterSRX.setNeutralMode(type)
                leftSlave1SPX.setNeutralMode(type)
                leftSlave2SRX.setNeutralMode(type)
            }
        }

    var highGear: Boolean = true
        set(wantsHighGear) {
            pneumaticShifter.set(if (wantsHighGear) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
//            test1.set(if (wantsHighGear) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
//            test2.set(if (wantsHighGear) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
            //test3.set(if (wantsHighGear) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)

            field = wantsHighGear
        }

    enum class DriveControlState {
        OPEN_LOOP,
        VELOCITY_SETPOINT,
        PATH_FOLLOWING,
        TURN_TO_HEADING, //turn in place
        MOTION_MAGIC
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {
        leftMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0) //configs sensor to a quad encoder
        leftMasterSRX.setSensorPhase(true) //to align positive sensor velocity with positive motor output
        leftMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 0)//might change to 20 ms to align with looper

        leftMasterSRX.config_kP(0, Constants.Gains.LEFT_LOW_KP, 0) //sets PIDF values
        leftMasterSRX.config_kI(0, Constants.Gains.LEFT_LOW_KI, 0)
        leftMasterSRX.config_kD(0, Constants.Gains.LEFT_LOW_KD, 0)
        leftMasterSRX.config_kF(0, Constants.Gains.LEFT_LOW_KF, 0)

        leftMasterSRX.config_kP(1, Constants.Gains.LEFT_HIGH_KP, 0)
        leftMasterSRX.config_kI(1, Constants.Gains.LEFT_HIGH_KI, 0)
        leftMasterSRX.config_kD(1, Constants.Gains.LEFT_HIGH_KD, 0)
        leftMasterSRX.config_kF(1, Constants.Gains.LEFT_HIGH_KF, 0)

        rightMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0) //configs sensor to a quad encoder
        rightMasterSRX.setSensorPhase(true) //to align positive sensor velocity with positive motor output
        rightMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 0)
        rightMasterSRX.config_kP(0, Constants.Gains.RIGHT_LOW_KP, 0) //sets PIdF values
        rightMasterSRX.config_kI(0, Constants.Gains.RIGHT_LOW_KI, 0)
        rightMasterSRX.config_kD(0, Constants.Gains.RIGHT_LOW_KD, 0)
        rightMasterSRX.config_kF(0, Constants.Gains.RIGHT_LOW_KF, 0)

        rightMasterSRX.config_kP(1, Constants.Gains.RIGHT_HIGH_KP, 0)
        rightMasterSRX.config_kI(1, Constants.Gains.RIGHT_HIGH_KI, 0)
        rightMasterSRX.config_kD(1, Constants.Gains.RIGHT_HIGH_KD, 0)
        rightMasterSRX.config_kF(1, Constants.Gains.RIGHT_HIGH_KF, 0)

        leftMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0)
        leftMasterSRX.configVelocityMeasurementWindow(32, 0)
        rightMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0)
        rightMasterSRX.configVelocityMeasurementWindow(32, 0)

        leftMasterSRX.inverted = true
        leftSlave1SPX.inverted = true
        leftSlave2SRX.inverted = true
        rightMasterSRX.inverted = false
        rightSlave1SPX.inverted = false
        rightSlave2SRX.inverted = false

        highGear = false

        setOpenLoop(DriveSignal.NEUTRAL)

        ahrs = AHRS(SPI.Port.kMXP)

        this.zeroSensors()

        path = Path(FieldPaths.STANDSTILL)
        segment = 0
        trajLength = 0

        lastLeftError = 0.0
        lastRightError = 0.0
    }


    @Synchronized
    fun setOpenLoop(signal: DriveSignal) {
        if (currentState !== DriveControlState.OPEN_LOOP) {
            leftMasterSRX.set(ControlMode.PercentOutput, 0.0)
            rightMasterSRX.set(ControlMode.PercentOutput, 0.0)
            leftMasterSRX.configNominalOutputForward(0.0, 0)
            rightMasterSRX.configNominalOutputForward(0.0, 0)
            currentState = DriveControlState.OPEN_LOOP
            brakeMode = NeutralMode.Coast
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor)
    }

    /**
     * Powers the left and right talons during OPEN_LOOP
     * @param left
     * @param right
     */
    @Synchronized
    public fun setLeftRightPower(left: Double, right: Double) {
//                println("power: $left, $right")
        leftMasterSRX.set(ControlMode.PercentOutput, left)
        rightMasterSRX.set(ControlMode.PercentOutput, right)
        //        println("left out: $left, left speed: ${leftMasterSRX.getSelectedSensorVelocity(0)}")
        //        println("right out: $right, right speed: ${rightMasterSRX.getSelectedSensorVelocity(0)}")
        //        println("actual power: ${leftMasterSRX.motorOutputPercent}, ${rightMasterSRX.motorOutputPercent}")
    }

    override fun zeroSensors() {
        if (ahrs.isConnected) {
            ahrs.reset()
        }
        resetEncoders()
    }

    @Synchronized
    fun resetEncoders() {
        leftMasterSRX.setSelectedSensorPosition(0, 0, 0)
        leftMasterSRX.sensorCollection.setPulseWidthPosition(0, 0)
        leftSlave1SPX.setSelectedSensorPosition(0, 0, 0)
        leftSlave2SRX.setSelectedSensorPosition(0, 0, 0)
        rightMasterSRX.setSelectedSensorPosition(0, 0, 0)
        rightMasterSRX.sensorCollection.setPulseWidthPosition(0, 0)
        rightSlave1SPX.setSelectedSensorPosition(0, 0, 0)
        rightSlave2SRX.setSelectedSensorPosition(0, 0, 0)

    }

    fun getAHRS(): AHRS? {
        return if (ahrs.isConnected) ahrs else null
    }

    override fun outputToSmartDashboard() {
        if (this.getAHRS() != null) {
            SmartDashboard.putNumber("gyro", this.getAHRS()!!.yaw.toDouble())
        } else {
            SmartDashboard.putNumber("gyro", -31337.0)
        }
        SmartDashboard.putNumber("leftTalon", leftMasterSRX.motorOutputVoltage)
        SmartDashboard.putNumber("rightTalon", rightMasterSRX.motorOutputVoltage)
        SmartDashboard.putNumber("leftEncoderInches", getLeftDistanceInches())
        SmartDashboard.putNumber("rightEncoderInches", rightMasterSRX.sensorCollection.quadraturePosition.toDouble())
    }

    fun startLiveWindowMode() {
        LiveWindow.addSensor("Drive", "Gyro", ahrs);
    }

    fun stopLiveWindowMode() {
        //TODO
    }

    fun updateLiveWindowTables() {

    }

    @Synchronized
    fun arcadeDrive(outputMagnitude: Double, curve: Double) {
        val leftOutput: Double
        val rightOutput: Double

        when {
            curve < 0 -> {
                val value = Math.log(-curve)
                var ratio = (value - .5) / (value + .5)
                if (ratio == 0.0) {
                    ratio = .0000000001
                }
                leftOutput = outputMagnitude / ratio
                rightOutput = outputMagnitude
            }
            curve > 0 -> {
                val value = Math.log(curve)
                var ratio = (value - .5) / (value + .5)
                if (ratio == 0.0) {
                    ratio = .0000000001
                }
                leftOutput = outputMagnitude
                rightOutput = outputMagnitude / ratio
            }
            else -> {
                leftOutput = outputMagnitude
                rightOutput = outputMagnitude
            }
        }
        setLeftRightPower(leftOutput, rightOutput)
    }

    @Synchronized
    fun usesTalonVelocityControl(state: DriveControlState): Boolean {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true
        }
        return false
    }

    @Synchronized
    fun usesTalonPositionControl(state: DriveControlState): Boolean {
        if (state == DriveControlState.TURN_TO_HEADING || state == DriveControlState.MOTION_MAGIC) {
            return true
        }
        return false
    }

    @Synchronized
    fun setVelocitySetpoint(leftInchesPerSec: Double, rightInchesPerSec: Double) {
        if (usesTalonVelocityControl(currentState)) {
            leftMasterSRX.set(ControlMode.Velocity, leftInchesPerSec)
            rightMasterSRX.set(ControlMode.Velocity, rightInchesPerSec)
           // println("left err: ${leftMasterSRX.getClosedLoopError(0)} trg: $leftInchesPerSec actual: ${leftMasterSRX.getSelectedSensorVelocity(0)}")
            //println("right err: ${rightMasterSRX.getClosedLoopError(0)} trg: $rightInchesPerSec actual: ${rightMasterSRX.getSelectedSensorVelocity(0)}")
        }
        else {
            configureTalonsForVelocityControl()
            currentState = DriveControlState.VELOCITY_SETPOINT
            setVelocitySetpoint(leftInchesPerSec, rightInchesPerSec)

        }
    }
    @Synchronized
    fun setPositionSetpoint(leftInches: Double, rightInches: Double) {
        if (usesTalonPositionControl(currentState)) {
            leftMasterSRX.set(ControlMode.MotionMagic, leftInches)
            rightMasterSRX.set(ControlMode.MotionMagic, rightInches)
            // println("left err: ${leftMasterSRX.getClosedLoopError(0)} trg: $leftInchesPerSec actual: ${leftMasterSRX.getSelectedSensorVelocity(0)}")
            //println("right err: ${rightMasterSRX.getClosedLoopError(0)} trg: $rightInchesPerSec actual: ${rightMasterSRX.getSelectedSensorVelocity(0)}")
        }
        else {
            configureTalonsforPositionControl()
            currentState = DriveControlState.MOTION_MAGIC
            setPositionSetpoint(leftInches, rightInches)

        }
    }

    @Synchronized
    private fun updatePositionSetpoint(leftPositionInches: Double, rightPositionInches: Double) {
        if (usesTalonPositionControl(currentState)) {
            leftMasterSRX.set(ControlMode.MotionMagic, leftPositionInches)
            rightMasterSRX.set(ControlMode.MotionMagic, rightPositionInches)
        } else {
            println("Bad position control state")
            leftMasterSRX.set(ControlMode.MotionMagic, 0.0)
            rightMasterSRX.set(ControlMode.MotionMagic, 0.0)
        }
    }

    @Synchronized
    private fun configureTalonsForVelocityControl() { //should further review cause im bad
        if (!usesTalonVelocityControl(currentState)) {
            // We entered a velocity control state.
            leftMasterSRX.set(ControlMode.Velocity, 0.0) //velocity  output value is in position change / 100ms
            leftMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.selectProfileSlot(Constants.Velocity.LOW_GEAR_VELOCITY_CONTROL_SLOT, 0)
            leftMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT, 0)
            leftMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT, 0)

            rightMasterSRX.set(ControlMode.Velocity, 0.0) //velocity  output value is in position change / 100ms
            rightMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.selectProfileSlot(Constants.Velocity.LOW_GEAR_VELOCITY_CONTROL_SLOT, 0)
            rightMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT, 0)
            rightMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT, 0)
            brakeMode = NeutralMode.Coast
        }
    }

    @Synchronized
    private fun configureTalonsforPositionControl() {
        if (!usesTalonPositionControl(currentState)) {
            // We entered a position control state.
            leftMasterSRX.set(ControlMode.MotionMagic, 0.0)
            leftMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.selectProfileSlot(Constants.Velocity.LOW_GEAR_VELOCITY_CONTROL_SLOT, 0)
            leftMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT, 0)
            leftMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT, 0)
            rightMasterSRX.set(ControlMode.MotionMagic, 0.0)
            rightMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.selectProfileSlot(Constants.Velocity.LOW_GEAR_VELOCITY_CONTROL_SLOT, 0)
            rightMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT, 0)
            rightMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT, 0)
            brakeMode = NeutralMode.Coast
        }
    }
    fun enablePathFollow(pathInput: Path){
        path = pathInput
        configureTalonsForVelocityControl()
        zeroSensors()
        segment = 0
        trajLength = path.getTrajLength()
        currentState = DriveControlState.PATH_FOLLOWING
        brakeMode = NeutralMode.Brake


    }
    fun updatePathFollowing(){
        //note *12 is to convert ft to inches
        if (segment < trajLength) {
            var leftTurn: Double = path.getLeftVelocityIndex(segment) * 12
            var rightTurn: Double = path.getRightVelocityIndex(segment) * 12
            val gyroHeading: Float = ahrs.yaw
            val desiredHeading: Double = radiansToDegrees(path.getHeadingIndex(segment))
            val angleDifference: Double = boundHalfDegrees(desiredHeading - gyroHeading)
            val turn: Double = 0.8 * 12 * (-1.0 / 80.0) * angleDifference

            val leftDistance: Double = getLeftDistanceInches()
            val rightDistance: Double = getRightDistanceInches()

            val leftErrorDistance: Double = path.getLeftDistanceIndex(segment)*12 - leftDistance
            val rightErrorDistance: Double = path.getRightDistanceIndex(segment)*12 - rightDistance

            val leftVelocityAdjustment = Constants.Gains.LEFT_LOW_KP * leftErrorDistance + Constants.Gains.LEFT_LOW_KD * ((leftErrorDistance - lastLeftError)/path.getDeltaTime())
            val rightVelocityAdjustment = Constants.Gains.RIGHT_LOW_KP * rightErrorDistance + Constants.Gains.RIGHT_LOW_KD * ((rightErrorDistance - lastRightError)/path.getDeltaTime())

         //   leftTurn = leftTurn + leftVelocityAdjustment
           // rightTurn = rightTurn + rightVelocityAdjustment

            lastLeftError = leftErrorDistance


            //leftTurn = leftTurn + turn
//            rightTurn = rightTurn - turn

            setVelocitySetpoint(leftTurn, rightTurn)
            println(" " +segment  + " " +leftTurn+" " + rightTurn)

            segment++
        }
        else {
            setVelocitySetpoint(0.0, 0.0)
        }
    }
    fun isPathFinished(): Boolean {
        return segment >= trajLength
    }


    fun onStart(timestamp: Double) {
        synchronized(this) {
            setOpenLoop(DriveSignal.NEUTRAL)
            brakeMode = NeutralMode.Coast
            setVelocitySetpoint(0.0, 0.0) //could update in future
        }
    }

    override fun stop() {
        synchronized(this) {
            setOpenLoop(DriveSignal.NEUTRAL)
        }
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setOpenLoop(DriveSignal.NEUTRAL)
        }

        override fun onLoop() {
            synchronized(this@Drive) {
                println("Left: " + leftMasterSRX.sensorCollection.quadraturePosition.toDouble())
                println("Right: " + rightMasterSRX.sensorCollection.quadraturePosition.toDouble())
                when (currentState) {
                    DriveControlState.OPEN_LOOP -> {
                        return
                    }
                    DriveControlState.VELOCITY_SETPOINT -> {
                        return
                    }
                    DriveControlState.PATH_FOLLOWING ->{
                        updatePathFollowing()
                    }
                    DriveControlState.TURN_TO_HEADING -> {
                        //updateTurnToHeading(timestamp);
                        return
                    }
                    else -> {
                        System.out.println("Unexpected drive control state: " + currentState)
                    }
                }
            }
        }

        override fun onStop() {
            setOpenLoop(DriveSignal.NEUTRAL)
        }
    }

    private fun pulsesToInches(pulses: Double): Double {
        return pulses*12/2336
    }

    private fun rpmToInchesPerSecond(rpm: Double): Double {
        return pulsesToInches(rpm) / 60
    }

    private fun inchesToRotations(inches: Double): Double {
        return inches / (Constants.Wheels.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI)
    }

    private fun inchesPerSecondToRpm(inches_per_second: Double): Double {
        return inchesToRotations(inches_per_second) * 60
    }

    fun getLeftDistanceInches(): Double {
        return pulsesToInches(leftMasterSRX.sensorCollection.quadraturePosition.toDouble())
    }

    fun getRightDistanceInches(): Double {
        return pulsesToInches(rightMasterSRX.sensorCollection.quadraturePosition.toDouble())
    }

    fun getLeftVelocityInchesPerSec(): Double {
        return rpmToInchesPerSecond(leftMasterSRX.getSelectedSensorVelocity(0).toDouble())
    }

    fun getRightVelocityInchesPerSec(): Double {
        return rpmToInchesPerSecond(rightMasterSRX.getSelectedSensorVelocity(0).toDouble())
    }
    fun radiansToDegrees(rad: Double): Double{
        return rad * 180 / (2 * Math.PI)
    }
    fun boundHalfDegrees(angle_degrees: Double): Double {
        var angle_degrees = angle_degrees
        while (angle_degrees >= 180.0) angle_degrees -= 360.0
        while (angle_degrees < -180.0) angle_degrees += 360.0
        return angle_degrees
    }


    companion object {
        val instance = Drive()
    }

}
