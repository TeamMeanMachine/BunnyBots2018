package org.team2471.bunnybots2018

import edu.wpi.first.wpilibj.Solenoid
import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem
import org.team2471.frc.lib.framework.use

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

    val speed: Double get() = (leftMotors.velocity + rightMotors.velocity) / 2.0

    val position: Double get() = (leftMotors.position + rightMotors.position) / 2.0

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
    }

    fun driveToSetPoints(leftDistance: Double, rightDistance: Double) {
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

suspend fun Drivetrain.driveAlongPath() = use(Drivetrain) {
}
