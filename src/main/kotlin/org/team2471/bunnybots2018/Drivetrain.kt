package org.team2471.bunnybots2018

import com.analog.adis16448.frc.ADIS16448_IMU
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.launch
import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.MeanlibScope
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem

object Drivetrain : DaemonSubsystem("Drivetrain") {
    //    private const val EDGES_PER_100_MS = 216 * 6.0 * Math.PI / 10.0
    private const val TICKS_PER_FOOT = 522.0
    private const val HIGH_SHIFTPOINT = 4.0
    private const val LOW_SHIFTPOINT = 3.5

    private const val PEAK_CURRENT_LIMIT = 0
    private const val CONTINUOUS_CURRENT_LIMIT = 25
    private const val PEAK_CURRENT_DURATION = 0

    private const val DISTANCE_P = 2.0 * 0.40 // 0.75
    private const val DISTANCE_D = 0.5

    const val GYRO_CORRECTION_P = 0.03
    const val GYRO_CORRECTION_I = 0.001
    const val GYRO_CORRECTION_I_DECAY = 1.0 - 0.0

    const val MAX_SPEED = 14.0
    const val TURNING_FEED_FORWARD = 0.03237277804646985

    const val LEFT_FEED_FORWARD_COEFFICIENT = 0.070541988198899
    const val LEFT_FEED_FORWARD_OFFSET = 0.021428882425651

    const val RIGHT_FEED_FORWARD_COEFFICIENT = 0.070541988198899
    const val RIGHT_FEED_FORWARD_OFFSET = 0.021428882425651


    private val leftMotors = TalonSRX(Talons.DRIVE_LEFT_1, Talons.DRIVE_LEFT_2, Talons.DRIVE_LEFT_3).config {
        feedbackCoefficient = 1.0 / TICKS_PER_FOOT
        brakeMode()
        currentLimit(CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, PEAK_CURRENT_DURATION)
        closedLoopRamp(0.1)
        sensorPhase(true)
        pid(0) {
            p(DISTANCE_P)
            d(DISTANCE_D)
        }
    }
    private val rightMotors = TalonSRX(Talons.DRIVE_RIGHT_1, Talons.DRIVE_RIGHT_2, Talons.DRIVE_RIGHT_3).config {
        feedbackCoefficient = 1.0 / TICKS_PER_FOOT
        brakeMode()
        inverted(true)
        currentLimit(CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, PEAK_CURRENT_DURATION)
        closedLoopRamp(0.1)
        sensorPhase(true)
        pid(0) {
            p(DISTANCE_P)
            d(DISTANCE_D)
        }
    }

    private val shifter = Solenoid(Solenoids.SHIFTER)

    private val gyro = ADIS16448_IMU()

    val table = NetworkTableInstance.getDefault().getTable("Drivetrain")

    val speed: Double get() = (leftMotors.velocity + rightMotors.velocity) / 2.0

    val leftSpeed: Double get() = leftMotors.velocity

    val rightSpeed: Double get() = rightMotors.velocity

    val position: Double get() = (leftMotors.position + rightMotors.position) / 2.0

    val leftDistance get() = leftMotors.position

    val rightDistance get() = rightMotors.position

    val distance get() = (leftDistance + rightDistance) / 2.0

    val leftError get() = leftMotors.closedLoopError

    val rightError get() = rightMotors.closedLoopError

    val gyroAngle: Double
        get() = gyro.angleZ

    init {
        SmartDashboard.putBoolean("Use Gyro", true)
        MeanlibScope.launch {
            val leftDistanceEntry = table.getEntry("Left Distance")
            val rightDistanceEntry = table.getEntry("Right Distance")
            val leftValuesEntry = table.getEntry("Left Motor Values")
            val rightValuesEntry = table.getEntry("Right Motor Values")
            val gyroAngleEntry = table.getEntry("Gyro Angle")

            periodic(0.1) {
                leftDistanceEntry.setDouble(leftDistance)
                rightDistanceEntry.setDouble(rightDistance)
                leftValuesEntry.setDouble(leftMotors.output)
                rightValuesEntry.setDouble(rightMotors.output)
                gyroAngleEntry.setDouble(gyroAngle)
            }
        }
    }

    fun drive(throttle: Double, softTurn: Double, hardTurn: Double) {
        val totalTurn = (softTurn * Math.abs(throttle)) + hardTurn

        var leftPower = throttle + totalTurn
        var rightPower = throttle - totalTurn

        val maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower))
        if (maxPower > 1) {
            leftPower /= maxPower
            rightPower /= maxPower
        }

        val speed = Math.abs(speed)
        if (speed > HIGH_SHIFTPOINT) {
            highGear()
        } else if (speed < LOW_SHIFTPOINT) {
            lowGear()
        }

        leftMotors.setPercentOutput(leftPower)
        rightMotors.setPercentOutput(rightPower)
    }

    fun highGear() = shifter.set(true)

    fun lowGear() = shifter.set(false)

    fun zeroDistances() {
        leftMotors.position = 0.0
        rightMotors.position = 0.0
    }

    fun zeroGyro() = gyro.reset()

    fun driveToSetPoints(leftDistance: Double, rightDistance: Double, leftFeedForward: Double, rightFeedForward: Double) {
        leftMotors.setPositionSetpoint(leftDistance, leftFeedForward)
        rightMotors.setPositionSetpoint(rightDistance, rightFeedForward)
    }

    fun coast() {
        leftMotors.config (0){
            coastMode()
        }
        rightMotors.config (0) {
            coastMode()
        }
    }

    fun brake() {
        leftMotors.config (0){
            brakeMode()
        }
        rightMotors.config (0) {
            brakeMode()
        }
    }

    fun stop() {
        leftMotors.stop()
        rightMotors.stop()
    }

    override suspend fun default() {
        leftMotors.position = 0.0
        rightMotors.position = 0.0
        periodic {
            val throttle = OI.driveThrottle
            val softTurn = OI.softTurn
            val hardTurn = OI.hardTurn
            drive(throttle, softTurn, hardTurn)
        }
    }
}