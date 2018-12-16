package org.team2471.bunnybots2018

import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem
import org.team2471.frc.lib.framework.use

object Drivetrain : DaemonSubsystem("Drivetrain") {
    private const val EDGES_PER_100_MS = 216 * 4.0 / 10.0
    private const val HIGH_SHIFTPOINT = 5.0
    private const val LOW_SHIFTPOINT = 4.0

    private const val PEAK_CURRENT_LIMIT = 0
    private const val CONTINUOUS_CURRENT_LIMIT = 25
    private const val PEAK_CURRENT_DURATION = 100

    private const val DISTANCE_P = 2.0 * 0.40 // 0.75
    private const val DISTANCE_D = 0.5

    private val leftMotors = TalonSRX(Talons.DRIVE_LEFT_1, Talons.DRIVE_LEFT_2, Talons.DRIVE_LEFT_3)
        .config {
        brakeMode()
        currentLimit(CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, PEAK_CURRENT_DURATION)
        closedLoopRamp(0.1)
        pid(0) {
            p(DISTANCE_P)
            d(DISTANCE_D)
        }
    }
    private val rightMotors = TalonSRX(Talons.DRIVE_RIGHT_1, Talons.DRIVE_RIGHT_2, Talons.DRIVE_RIGHT_3)
        .config {
        brakeMode()
        inverted(true)
        currentLimit(CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, PEAK_CURRENT_DURATION)
        closedLoopRamp(0.1)
        pid(0) {
            p(DISTANCE_P)
            d(DISTANCE_D)
        }
    }

//    private val shifter = Solenoid(0)

    val speed: Double get() = Math.abs(-leftMotors.velocity / EDGES_PER_100_MS +
            rightMotors.velocity / EDGES_PER_100_MS) / 2.0


    fun drive(throttle: Double, softTurn: Double, hardTurn: Double) {
        val totalTurn = (softTurn * Math.abs(throttle)) + hardTurn

        var leftPower = throttle + totalTurn
        var rightPower = throttle - totalTurn

        val maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower))
        if (maxPower > 1) {
            leftPower /= maxPower
            rightPower /= maxPower
        }

//        if(speed > HIGH_SHIFTPOINT) {
//            shifter.set(true)
//        } else if (speed < LOW_SHIFTPOINT) {
//            shifter.set(false)
//        }

        leftMotors.setPercent(leftPower)
        rightMotors.setPercent(rightPower)
    }

    fun zeroDistances() {
    }

    fun driveToSetPoints(leftDistance: Double, rightDistance: Double) {
    }

    override suspend fun default() {
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
