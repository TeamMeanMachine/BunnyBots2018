package org.team2471.bunnybots2018

import com.analog.adis16448.frc.ADIS16448_IMU
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.windRelativeAngles
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.vector.Vector2

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

    const val GYRO_CORRECTION_P = 0.025 * 0.75
    const val GYRO_CORRECTION_I = 0.002 * 0.5
    const val GYRO_CORRECTION_I_DECAY = 1.0 - 0.0

    const val MAX_SPEED = 14.0

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

    val leftError get() = leftMotors.closedLoopError

    val rightError get() = rightMotors.closedLoopError

    val gyroAngle: Double
        get() = gyro.angleZ

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
            shifter.set(true)
        } else if (speed < LOW_SHIFTPOINT) {
            shifter.set(false)
        }

        leftMotors.setPercentOutput(leftPower)
        rightMotors.setPercentOutput(rightPower)
    }

    fun zeroDistances() {
        leftMotors.position = 0.0
        rightMotors.position = 0.0
    }

    fun driveToSetPoints(leftDistance: Double, rightDistance: Double) {
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

suspend fun Drivetrain.driveAlongPath(path: Path2D) = use(Drivetrain) {
        println("Driving along path ${path.name}, duration: ${path.durationWithSpeed}, travel direction: ${path.robotDirection}, mirrored: ${path.isMirrored}")
        path.resetDistances()

        zeroDistances()
        var prevLeftDistance = 0.0
        var prevRightDistance = 0.0
        var prevLeftVelocity = 0.0
        var prevRightVelocity = 0.0
        var prevTime = 0.0

        val pathAngleEntry = table.getEntry("Path Angle")
        val angleErrorEntry = table.getEntry("Angle Error")
        val gyroCorrectionEntry = table.getEntry("Gyro Correction")

        val leftPositionErrorEntry = table.getEntry("Left Position Error")
        val rightPositionErrorEntry = table.getEntry("Right Position Error")

        val leftVelocityErrorEntry = table.getEntry("Left Velocity Error")
        val rightVelocityErrorEntry = table.getEntry("Right Velocity Error")

        val timer = Timer().apply { start() }

        var angleErrorAccum = 0.0
        try {
            periodic {
                val t = timer.get()
                val dt = t - prevTime

                // apply gyro corrections to the distances
                val gyroAngle = gyroAngle
                val pathAngle = Math.toDegrees(Vector2.angle(path.getTangent(t)))
                val angleError = pathAngle - windRelativeAngles(pathAngle, gyroAngle)

                angleErrorAccum = angleErrorAccum * GYRO_CORRECTION_I_DECAY + angleError

                val gyroCorrection = if (SmartDashboard.getBoolean("Use Gyro", true)) {
                    angleError * GYRO_CORRECTION_P + angleErrorAccum * GYRO_CORRECTION_I
                } else {
                    0.0
                }

                val leftDistance = path.getLeftDistance(t) + gyroCorrection
                val rightDistance = path.getRightDistance(t) - gyroCorrection

                val leftVelocity = (leftDistance - prevLeftDistance) / dt
                val rightVelocity = (rightDistance - prevRightDistance) / dt

                val leftVelocityError = leftSpeed - leftVelocity
                val rightVelocityError = rightSpeed - rightVelocity

                val velocityDeltaTimesCoefficient = (leftVelocity - rightVelocity) * TURNING_FEED_FORWARD

                pathAngleEntry.setDouble(pathAngle)
                angleErrorEntry.setDouble(angleError)
                leftPositionErrorEntry.setDouble(leftError)
                rightPositionErrorEntry.setDouble(rightError)
                leftVelocityErrorEntry.setDouble(leftVelocityError)
                rightVelocityErrorEntry.setDouble(rightVelocityError)

                gyroCorrectionEntry.setDouble(gyroCorrection)

                val leftFeedForward = leftVelocity * DrivetrainConstants.LEFT_FEED_FORWARD_COEFFICIENT +
                        (DrivetrainConstants.LEFT_FEED_FORWARD_OFFSET * signum(leftVelocity)) +
                        velocityDeltaTimesCoefficient

                val rightFeedForward = rightVelocity * DrivetrainConstants.RIGHT_FEED_FORWARD_COEFFICIENT +
                        (DrivetrainConstants.RIGHT_FEED_FORWARD_OFFSET * signum(rightVelocity)) -
                        velocityDeltaTimesCoefficient

                leftMaster.set(ControlMode.Position, feetToTicks(leftDistance),
                    DemandType.ArbitraryFeedForward, leftFeedForward)
                rightMaster.set(ControlMode.Position, feetToTicks(rightDistance),
                    DemandType.ArbitraryFeedForward, rightFeedForward)

                if (leftMaster.motorOutputPercent > 0.95) {
                    DriverStation.reportWarning("Left motor is saturated", false)
                }
                if (rightMaster.motorOutputPercent > 0.95) {
                    DriverStation.reportWarning("Right motor is saturated", false)
                }

                finished = t >= path.durationWithSpeed + extraTime

                prevTime = t
                prevLeftDistance = leftDistance
                prevRightDistance = rightDistance
                prevLeftVelocity = leftVelocity
                prevRightVelocity = rightVelocity
//            }
//        } finally {
            leftMaster.neutralOutput()
            rightMaster.neutralOutput()
//        }
//}
