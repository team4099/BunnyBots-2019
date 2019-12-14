package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.DemandType
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.team2363.logger.HelixEvents
import com.team2363.logger.HelixLogger
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.auto.paths.FieldPaths
import org.usfirst.frc.team4099.auto.paths.Path
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import kotlin.math.abs
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.sin

class Drive private constructor() : Subsystem() {
    private val rightMasterSpark = CANSparkMax(Constants.Drive.RIGHT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless)// CANMotorControllerFactory.createDefaultTalon(Constants.Drive.RIGHT_MASTER_ID)
//    private val rightSlaveSpark = CANSparkMax(Constants.Drive.RIGHT_SLAVE_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless)// CANMotorControllerFactory.createPermanentSlaveTalon(
        //Constants.Drive.RIGHT_SLAVE_1_ID,
        //Constants.Drive.RIGHT_MASTER_ID
    //)

    private val leftMasterSpark = CANSparkMax(Constants.Drive.LEFT_MASTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless)//CANMotorControllerFactory.createDefaultTalon(Constants.Drive.LEFT_MASTER_ID)
//    private val leftSlaveSpark = CANSparkMax(Constants.Drive.LEFT_SLAVE_1_ID, CANSparkMaxLowLevel.MotorType.kBrushless)//CANMotorControllerFactory.createPermanentSlaveTalon(
//        Constants.Drive.LEFT_SLAVE_1_ID,
//        Constants.Drive.LEFT_MASTER_ID
//    )

    private val leftEncoder = leftMasterSpark.encoder
    private val rightEncoder = rightMasterSpark.encoder

    private val ahrs: AHRS
    private var path: Path
    private var segment: Int
    private var trajLength: Int
    private var lastLeftError: Double
    private var lastRightError: Double

    private var leftTargetVel = 0.0
    private var rightTargetVel = 0.0

    var yaw: Double = 0.0
        get() {
            if (ahrs.isConnected) field = ahrs.yaw.toDouble()
            else HelixEvents.addEvent("DRIVETRAIN", "Gyroscope queried but not connected")
            return field
        }
    var angle: Double = 0.0
        get() {
            if (ahrs.isConnected) field = ahrs.angle.toDouble()
            else HelixEvents.addEvent("DRIVETRAIN", "Gyroscope queried but not connected")
            return field
        }

    var brakeMode: CANSparkMax.IdleMode =
        CANSparkMax.IdleMode.kCoast // sets whether the brake mode should be coast (no resistance) or by force
        set(type) {
            if (brakeMode != type) {
//                rightMasterSpark.setNeutralMode(type)
//                rightSlaveSpark.setNeutralMode(type)

                rightMasterSpark.idleMode = type
//                rightSlaveSpark.idleMode = type
//                leftMasterSpark.setNeutralMode(type)
//                leftSlaveSpark.setNeutralMode(type)

                leftMasterSpark.idleMode = type
//                rightSlaveSpark.idleMode = type
            }
        }

    enum class DriveControlState {
        OPEN_LOOP,
        VELOCITY_SETPOINT,
        PATH_FOLLOWING,
        TURN_TO_HEADING, // turn in place
        MOTION_MAGIC
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {
//        rightMasterSpark.configFactoryDefault()
//        rightSlaveSpark.configFactoryDefault()

        rightMasterSpark.restoreFactoryDefaults(true)
//        rightSlaveSpark.restoreFactoryDefaults(true)

//        leftMasterSpark.configFactoryDefault()
//        leftSlaveSpark.configFactoryDefault()

        leftMasterSpark.restoreFactoryDefaults(true)
//        leftSlaveSpark.restoreFactoryDefaults(true)

//        rightMasterSpark.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_2_Feedback0,
//            Constants.Drive.STATUS_FRAME_PERIOD_MS, Constants.Universal.TIMEOUT
//        )
//        leftMasterSpark.setStatusFramePeriod(
//            StatusFrameEnhanced.Status_2_Feedback0,
//            Constants.Drive.STATUS_FRAME_PERIOD_MS, Constants.Universal.TIMEOUT
//        )

        rightMasterSpark.inverted = true
//        rightSlaveSpark.inverted = true
        leftMasterSpark.inverted = false
//        leftSlaveSpark.inverted = false

//        leftSlaveSpark.follow(leftMasterSpark)
//        rightSlaveSpark.follow(rightMasterSpark)
//        rightMasterSpark.setSensorPhase(true)
//        rightSlaveSpark.setSensorPhase(true)
//        leftMasterSpark.setSensorPhase(true)
//        leftSlaveSpark.setSensorPhase(true)

//        rightMasterSpark.configSelectedFeedbackSensor(
//            FeedbackDevice.CTRE_MagEncoder_Relative,
//            0,
//            Constants.Universal.TIMEOUT
//        )
//        rightMasterSpark.configSelectedFeedbackSensor(
//            FeedbackDevice.CTRE_MagEncoder_Relative,
//            1,
//            Constants.Universal.TIMEOUT
//        )
//
//        leftMasterSpark.configSelectedFeedbackSensor(
//            FeedbackDevice.CTRE_MagEncoder_Relative,
//            0,
//            Constants.Universal.TIMEOUT
//        )
//        leftMasterSpark.configSelectedFeedbackSensor(
//            FeedbackDevice.CTRE_MagEncoder_Relative,
//            1,
//            Constants.Universal.TIMEOUT
//        )
//
//        rightMasterSpark.enableVoltageCompensation(true)
//        rightMasterSpark.configVoltageCompSaturation(Constants.Drive.VOLTAGE_COMP_LEVEL, Constants.Universal.TIMEOUT)
//        leftMasterSpark.enableVoltageCompensation(true)
//        leftMasterSpark.configVoltageCompSaturation(Constants.Drive.VOLTAGE_COMP_LEVEL, Constants.Universal.TIMEOUT)
//
//        rightMasterSpark.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.Universal.TIMEOUT)
//        leftMasterSpark.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.Universal.TIMEOUT)
//
//        rightMasterSpark.configClosedloopRamp(Constants.Drive.CLOSED_LOOP_RAMP, Constants.Universal.TIMEOUT)
//        leftMasterSpark.configClosedloopRamp(Constants.Drive.CLOSED_LOOP_RAMP, Constants.Universal.TIMEOUT)
//        rightMasterSpark.configNeutralDeadband(
//            Constants.Drive.PERCENT_DEADBAND,
//            Constants.Universal.TIMEOUT
//        ) // 254 used 0 for timeout
//        leftMasterSpark.configNeutralDeadband(Constants.Drive.PERCENT_DEADBAND, Constants.Universal.TIMEOUT)

        leftMasterSpark.openLoopRampRate = 0.1
        rightMasterSpark.openLoopRampRate = 0.1

        // TODO: SET CONVERSION FACTORS

//        leftMasterSpark.config_kP(0, Constants.Gains.LEFT_LOW_KP, Constants.Universal.TIMEOUT)
//        leftMasterSpark.config_kI(0, Constants.Gains.LEFT_LOW_KI, Constants.Universal.TIMEOUT)
//        leftMasterSpark.config_kD(0, Constants.Gains.LEFT_LOW_KD, Constants.Universal.TIMEOUT)
//        leftMasterSpark.config_kF(0, Constants.Gains.LEFT_LOW_KF, Constants.Universal.TIMEOUT)
//
//        leftMasterSpark.config_kP(1, Constants.Gains.LEFT_HIGH_KP, Constants.Universal.TIMEOUT)
//        leftMasterSpark.config_kI(1, Constants.Gains.LEFT_HIGH_KI, Constants.Universal.TIMEOUT)
//        leftMasterSpark.config_kD(1, Constants.Gains.LEFT_HIGH_KD, Constants.Universal.TIMEOUT)
//        leftMasterSpark.config_kF(1, Constants.Gains.LEFT_HIGH_KF, Constants.Universal.TIMEOUT)
//
//        rightMasterSpark.config_kP(0, Constants.Gains.RIGHT_LOW_KP, Constants.Universal.TIMEOUT)
//        rightMasterSpark.config_kI(0, Constants.Gains.RIGHT_LOW_KI, Constants.Universal.TIMEOUT)
//        rightMasterSpark.config_kD(0, Constants.Gains.RIGHT_LOW_KD, Constants.Universal.TIMEOUT)
//        rightMasterSpark.config_kF(0, Constants.Gains.RIGHT_LOW_KF, Constants.Universal.TIMEOUT)
//
//        rightMasterSpark.config_kP(1, Constants.Gains.RIGHT_HIGH_KP, Constants.Universal.TIMEOUT)
//        rightMasterSpark.config_kI(1, Constants.Gains.RIGHT_HIGH_KI, Constants.Universal.TIMEOUT)
//        rightMasterSpark.config_kD(1, Constants.Gains.RIGHT_HIGH_KD, Constants.Universal.TIMEOUT)
//        rightMasterSpark.config_kF(1, Constants.Gains.RIGHT_HIGH_KF, Constants.Universal.TIMEOUT)
//
//        rightMasterSpark.configContinuousCurrentLimit(
//            Constants.Drive.CONTINUOUS_CURRENT_LIMIT,
//            Constants.Universal.TIMEOUT
//        )
//        leftMasterSpark.configContinuousCurrentLimit(
//            Constants.Drive.CONTINUOUS_CURRENT_LIMIT,
//            Constants.Universal.TIMEOUT
//        )

        setOpenLoop(DriveSignal.NEUTRAL)

        ahrs = AHRS(SPI.Port.kMXP)

        this.zeroSensors()

        path = Path(FieldPaths.STANDSTILL)
        segment = 0
        trajLength = 0

        lastLeftError = 0.0
        lastRightError = 0.0

        registerLogging()
    }

    override fun stop() {
        synchronized(this) {
            setOpenLoop(DriveSignal.NEUTRAL)
        }
    }

    override val loop = object : Loop {
        override fun onStart(timestamp: Double) {
            setOpenLoop(DriveSignal.NEUTRAL)
        }

        override fun onLoop(timestamp: Double) {
            synchronized(this@Drive) {
                when (currentState) {
                    DriveControlState.OPEN_LOOP -> {
                        leftTargetVel = 0.0
                        rightTargetVel = 0.0
                    }
                    DriveControlState.VELOCITY_SETPOINT -> {
                    }
                    DriveControlState.PATH_FOLLOWING -> {
                        updatePathFollowing()
                    }
                    DriveControlState.TURN_TO_HEADING -> {
                        leftTargetVel = 0.0
                        rightTargetVel = 0.0
                        // updateTurnToHeading(timestamp)
                    }
                    else -> {
                        HelixEvents.addEvent("DRIVETRAIN", "Unexpected drive control state: $currentState")
                    }
                }
            }
        }

        override fun onStop(timestamp: Double) = stop()
    }

    private fun registerLogging() {
        HelixLogger.addSource("DT Left Output %") { leftMasterSpark.appliedOutput }
        HelixLogger.addSource("DT Right Output %") { rightMasterSpark.appliedOutput }

        HelixLogger.addSource("DT Left Master Input Current") { leftMasterSpark.outputCurrent }
//        HelixLogger.addSource("DT Left Slave Input Current") { leftSlaveSpark.outputCurrent }
        HelixLogger.addSource("DT Right Master Input Current") { rightMasterSpark.outputCurrent }
//        HelixLogger.addSource("DT Right Slave Input Current") { rightSlaveSpark.outputCurrent }

        HelixLogger.addSource("DT Left Velocity (in/s)") { getLeftVelocityInchesPerSec() }
        HelixLogger.addSource("DT Right Velocity (in/s)") { getRightVelocityInchesPerSec() }
        HelixLogger.addSource("DT Left Target Velocity (in/s)") { leftTargetVel }
        HelixLogger.addSource("DT Left Target Velocity (in/s)") { rightTargetVel }

        HelixLogger.addSource("DT Left Position (in)") { getLeftDistanceInches() }
        HelixLogger.addSource("DT Right Position (in)") { getRightDistanceInches() }

        HelixLogger.addSource("DT Gyro Angle") { angle }

        HelixLogger.addSource("DT Pathfollow Segment") { segment }
    }

    override fun outputToSmartDashboard() {
        if (ahrs.isConnected) {
            SmartDashboard.putNumber("gyro", yaw)
        } else {
            SmartDashboard.putNumber("gyro", Constants.Drive.GYRO_BAD_VALUE)
        }
    }

    override fun zeroSensors() {
        if (ahrs.isConnected) {
            while (!Utils.around(yaw, 0.0, 1.0)) {
                ahrs.reset()
            }
        } else {
            HelixEvents.addEvent("DRIVETRAIN", "Gyroscope queried but not connected")
        }
        resetEncoders()
    }

    @Synchronized
    fun setOpenLoop(signal: DriveSignal) {
        if (currentState !== DriveControlState.OPEN_LOOP) {
//            leftMasterSpark.configNominalOutputForward(0.0, Constants.Universal.TIMEOUT)
//            rightMasterSpark.configNominalOutputForward(0.0, Constants.Universal.TIMEOUT)
            currentState = DriveControlState.OPEN_LOOP
            brakeMode = CANSparkMax.IdleMode.kCoast
            HelixEvents.addEvent("DRIVETRAIN", "Entered open loop control")
        }
        setLeftRightPower(
            signal.leftMotor * Constants.Drive.MAX_LEFT_OPEN_LOOP_POWER,
            signal.rightMotor * Constants.Drive.MAX_RIGHT_OPEN_LOOP_POWER
        )
    }

    /**
     * Powers the left and right talons during OPEN_LOOP
     * @param left
     * @param right
     */
    @Synchronized
    fun setLeftRightPower(left: Double, right: Double) {
//        leftMasterSpark.set(ControlMode.PercentOutput, left)
//        rightMasterSpark.set(ControlMode.PercentOutput, right)
        leftMasterSpark.set(left)
        rightMasterSpark.set(right)
    }

    @Synchronized
    fun resetEncoders() {
//        rightMasterSpark.sensorCollection.setQuadraturePosition(0, Constants.Universal.TIMEOUT)
//        leftMasterSpark.sensorCollection.setQuadraturePosition(0, Constants.Universal.TIMEOUT)
        leftEncoder.position = 0.0
        rightEncoder.position = 0.0
    }

    // Copied from WPIlib arcade drive with no functional modification
    @Suppress("MagicNumber")
    @Synchronized
    fun arcadeDrive(outputMagnitude: Double, curve: Double) {
        val leftOutput: Double
        val rightOutput: Double

        when {
            curve < 0 -> {
                val value = ln(-curve)
                var ratio = (value - .5) / (value + .5)
                if (ratio == 0.0) {
                    ratio = .0000000001
                }
                leftOutput = outputMagnitude / ratio
                rightOutput = outputMagnitude
            }
            curve > 0 -> {
                val value = ln(curve)
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

    // thank you team 254 but i like 148 better...
    @Synchronized
    fun setCheesyishDrive(throttle: Double, wheel: Double, quickTurn: Boolean) {
        var mThrottle = throttle
        var mWheel = wheel
        if (Utils.around(mThrottle, 0.0, Constants.Joysticks.THROTTLE_DEADBAND)) {
            mThrottle = 0.0
        }

        if (Utils.around(mWheel, 0.0, Constants.Joysticks.TURN_DEADBAND)) {
            mWheel = 0.0
        }

        val denominator = sin(Math.PI / 2.0 * Constants.Drive.WHEEL_NON_LINEARITY)
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            mWheel = sin(Math.PI / 2.0 * Constants.Drive.WHEEL_NON_LINEARITY * mWheel)
            mWheel = sin(Math.PI / 2.0 * Constants.Drive.WHEEL_NON_LINEARITY * mWheel)
            mWheel = mWheel / (denominator * denominator) * abs(mThrottle)
        }

        mWheel *= Constants.Drive.WHEEL_GAIN
        val driveSignal = if (abs(mWheel) < Constants.Universal.EPSILON) {
            DriveSignal(mThrottle, mThrottle)
        } else {
            val deltaV = Constants.Drive.WHEEL_TRACK_WIDTH_INCHES * mWheel / (2 * Constants.Drive.TRACK_SCRUB_FACTOR)
            DriveSignal(mThrottle - deltaV, mThrottle + deltaV)
        }

        val scalingFactor = max(1.0, max(abs(driveSignal.leftMotor), abs(driveSignal.rightMotor)))
        setOpenLoop(DriveSignal(driveSignal.leftMotor / scalingFactor, driveSignal.rightMotor / scalingFactor))
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
    fun setVelocitySetpoint(
        leftFeetPerSec: Double,
        rightFeetPerSec: Double,
        leftFeetPerSecSq: Double,
        rightFeetPerSecSq: Double
    ) {
        if (usesTalonVelocityControl(currentState)) {
            // TODO: change constants
            leftTargetVel = leftFeetPerSec * Constants.Drive.FEET_PER_SEC_TO_NATIVE
            rightTargetVel = rightFeetPerSec * Constants.Drive.FEET_PER_SEC_TO_NATIVE

            val leftFeedForward: Double = if (leftFeetPerSec > 0) {
                Constants.Drive.LEFT_KV_FORWARD_HIGH * leftFeetPerSec +
                    Constants.Drive.LEFT_KA_FORWARD_HIGH * leftFeetPerSecSq +
                    Constants.Drive.LEFT_V_INTERCEPT_FORWARD_HIGH
            } else {
                Constants.Drive.LEFT_KV_REVERSE_HIGH * leftFeetPerSec +
                    Constants.Drive.LEFT_KA_REVERSE_HIGH * leftFeetPerSecSq +
                    Constants.Drive.LEFT_V_INTERCEPT_REVERSE_HIGH
            }
            val rightFeedForward: Double = if (rightFeetPerSec > 0) {
                Constants.Drive.RIGHT_KV_FORWARD_HIGH * rightFeetPerSec +
                    Constants.Drive.RIGHT_KA_FORWARD_HIGH * rightFeetPerSecSq +
                    Constants.Drive.RIGHT_V_INTERCEPT_FORWARD_HIGH
            } else {
                Constants.Drive.RIGHT_KV_REVERSE_HIGH * rightFeetPerSec +
                    Constants.Drive.RIGHT_KA_REVERSE_HIGH * rightFeetPerSecSq +
                    Constants.Drive.RIGHT_V_INTERCEPT_REVERSE_HIGH
            }


//            leftMasterSpark.set(ControlMode.Velocity, leftTargetVel, DemandType.ArbitraryFeedForward, leftFeedForward)
//            rightMasterSpark.set(
//                ControlMode.Velocity,
//                rightTargetVel,
//                DemandType.ArbitraryFeedForward,
//                rightFeedForward
//            )
        } else {
            configureTalonsForVelocityControl()
            currentState = DriveControlState.VELOCITY_SETPOINT
            setVelocitySetpoint(leftFeetPerSec, rightFeetPerSec, leftFeetPerSecSq, rightFeetPerSecSq)
        }
    }

    @Synchronized
    fun setPositionSetpoint(leftInches: Double, rightInches: Double) {
        if (usesTalonPositionControl(currentState)) {
//            leftMasterSpark.set(ControlMode.MotionMagic, leftInches * Constants.Drive.FEET_PER_SEC_TO_NATIVE)
//            rightMasterSpark.set(ControlMode.MotionMagic, rightInches * Constants.Drive.FEET_PER_SEC_TO_NATIVE)
        } else {
            configureTalonsforPositionControl()
            currentState = DriveControlState.MOTION_MAGIC
            setPositionSetpoint(leftInches, rightInches)
        }
    }

    @Synchronized
    private fun configureTalonsForVelocityControl() { // should further review cause im bad
        if (!usesTalonVelocityControl(currentState)) {
            // We entered a velocity control state.

//            leftMasterSpark.set(ControlMode.Velocity, 0.0) // velocity  output value is in position change / 100ms
//            leftMasterSpark.configNominalOutputForward(Constants.Drive.AUTO_NOMINAL_OUTPUT, Constants.Universal.TIMEOUT)
//            leftMasterSpark.configNominalOutputReverse(Constants.Drive.AUTO_NOMINAL_OUTPUT, Constants.Universal.TIMEOUT)
//            leftMasterSpark.selectProfileSlot(0, Constants.Universal.TIMEOUT)
//            leftMasterSpark.configPeakOutputForward(Constants.Drive.AUTO_PEAK_OUTPUT, Constants.Universal.TIMEOUT)
//            leftMasterSpark.configPeakOutputReverse(
//                Constants.Drive.AUTO_PEAK_OUTPUT * -1.0,
//                Constants.Universal.TIMEOUT
//            )
//
//            rightMasterSpark.set(ControlMode.Velocity, 0.0) // velocity  output value is in position change / 100ms
//            rightMasterSpark.configNominalOutputForward(
//                Constants.Drive.AUTO_NOMINAL_OUTPUT,
//                Constants.Universal.TIMEOUT
//            )
//            rightMasterSpark.configNominalOutputReverse(
//                Constants.Drive.AUTO_NOMINAL_OUTPUT,
//                Constants.Universal.TIMEOUT
//            )
//            rightMasterSpark.selectProfileSlot(0, 0)
//            rightMasterSpark.configPeakOutputForward(Constants.Drive.AUTO_PEAK_OUTPUT, Constants.Universal.TIMEOUT)
//            rightMasterSpark.configPeakOutputReverse(
//                Constants.Drive.AUTO_PEAK_OUTPUT * -1.0,
//                Constants.Universal.TIMEOUT
//            )
//            brakeMode = NeutralMode.Brake
        }
        HelixEvents.addEvent("DRIVETRAIN", "Configured Talons for velocity control")
    }

    @Synchronized
    private fun configureTalonsforPositionControl() {
        if (!usesTalonPositionControl(currentState)) {
            // We entered a position control state.
//            leftMasterSpark.configNominalOutputForward(Constants.Drive.AUTO_NOMINAL_OUTPUT, Constants.Universal.TIMEOUT)
//            leftMasterSpark.configNominalOutputReverse(Constants.Drive.AUTO_NOMINAL_OUTPUT, Constants.Universal.TIMEOUT)
//            leftMasterSpark.selectProfileSlot(0, Constants.Universal.TIMEOUT)
//            leftMasterSpark.configPeakOutputForward(Constants.Drive.AUTO_PEAK_OUTPUT, Constants.Universal.TIMEOUT)
//            leftMasterSpark.configPeakOutputReverse(
//                Constants.Drive.AUTO_PEAK_OUTPUT * -1.0,
//                Constants.Universal.TIMEOUT
//            )
//
//            rightMasterSpark.configNominalOutputForward(
//                Constants.Drive.AUTO_NOMINAL_OUTPUT,
//                Constants.Universal.TIMEOUT
//            )
//            rightMasterSpark.configNominalOutputReverse(
//                Constants.Drive.AUTO_NOMINAL_OUTPUT,
//                Constants.Universal.TIMEOUT
//            )
//            rightMasterSpark.selectProfileSlot(0, 0)
//            rightMasterSpark.configPeakOutputForward(Constants.Drive.AUTO_PEAK_OUTPUT, Constants.Universal.TIMEOUT)
//            rightMasterSpark.configPeakOutputReverse(
//                Constants.Drive.AUTO_PEAK_OUTPUT * -1.0,
//                Constants.Universal.TIMEOUT
//            )
//
//            brakeMode = NeutralMode.Brake
        }
        HelixEvents.addEvent("DRIVETRAIN", "Configured Talons for position control")
    }

    fun enablePathFollow(pathInput: Path) {
        path = pathInput
        configureTalonsForVelocityControl()
        zeroSensors()
        segment = 0
        trajLength = path.getTrajLength()
        currentState = DriveControlState.PATH_FOLLOWING
        HelixEvents.addEvent("DRIVETRAIN", "Path following")
    }

    fun updatePathFollowing() {
        var leftTurn = path.getLeftVelocityIndex(segment)
        var rightTurn = path.getRightVelocityIndex(segment)

        val desiredHeading = Math.toDegrees(path.getHeadingIndex(segment))
        val angleDifference = boundHalfDegrees(desiredHeading - yaw)
        val turn: Double = Constants.Autonomous.PATH_FOLLOW_TURN_KP * angleDifference

        val leftDistance: Double = getLeftDistanceInches()
        val rightDistance: Double = getRightDistanceInches()

        val leftErrorDistance: Double = path.getLeftDistanceIndex(segment) - leftDistance
        val rightErrorDistance: Double = path.getRightDistanceIndex(segment) - rightDistance

        val leftVelocityAdjustment =
            Constants.Gains.LEFT_LOW_KP * leftErrorDistance +
                Constants.Gains.LEFT_LOW_KD * ((leftErrorDistance - lastLeftError) / path.getDeltaTime())
        val rightVelocityAdjustment =
            Constants.Gains.RIGHT_LOW_KP * rightErrorDistance +
                Constants.Gains.RIGHT_LOW_KD * ((rightErrorDistance - lastRightError) / path.getDeltaTime())

        leftTurn += leftVelocityAdjustment
        rightTurn += rightVelocityAdjustment

        lastLeftError = leftErrorDistance
        lastRightError = rightErrorDistance

        leftTurn += turn
        rightTurn -= turn

        setVelocitySetpoint(
            leftTurn,
            rightTurn,
            path.getLeftAccelerationIndex(segment),
            path.getRightAccelerationIndex(segment)
        )
        segment++
    }

    fun isPathFinished(): Boolean {
        return segment >= trajLength
    }

    private fun nativeToInches(nativeUnits: Int): Double {
        return nativeUnits * Constants.Drive.NATIVE_TO_REVS * Constants.Drive.WHEEL_DIAMETER_INCHES * Math.PI
    }

    private fun rpmToInchesPerSecond(rpm: Double): Double {
        return (rpm) / 60 * Math.PI * Constants.Drive.WHEEL_DIAMETER_INCHES
    }

    private fun nativeToInchesPerSecond(nativeUnits: Int): Double {
        return nativeToInches(nativeUnits) * 10
    }

    private fun inchesToRotations(inches: Double): Double {
        return inches / (Constants.Drive.WHEEL_DIAMETER_INCHES * Math.PI)
    }

    private fun inchesPerSecondToRpm(inchesPerSecond: Double): Double {
        return inchesToRotations(inchesPerSecond) * 60
    }

    fun getLeftDistanceInches(): Double {
        return nativeToInches(leftEncoder.position.toInt())
    }

    fun getRightDistanceInches(): Double {
        return nativeToInches(rightEncoder.position.toInt())
    }

    fun getLeftVelocityInchesPerSec(): Double {
        return nativeToInchesPerSecond(leftEncoder.velocity.toInt())
    }

    fun getRightVelocityInchesPerSec(): Double {
        return nativeToInchesPerSecond(rightEncoder.velocity.toInt())
    }

    fun boundHalfDegrees(angleDegrees: Double): Double {
        return ((angleDegrees + 180.0) % 360.0) - 180.0
    }

    companion object {
        val instance = Drive()
    }
}
