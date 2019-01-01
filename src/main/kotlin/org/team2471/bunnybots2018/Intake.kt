package org.team2471.bunnybots2018

import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem
import org.team2471.frc.lib.framework.use

object Intake : DaemonSubsystem("Intake") {
    private val motors = TalonSRX(Talons.INTAKE_LEFT, Talons.INTAKE_RIGHT).config {
        openLoopRamp(0.1)
        currentLimit(25, 0, 0)
        ctreFollowers[0].inverted = true
    }

    var autoIntake = true

    fun intake(speed: Double) = motors.setPercentOutput(speed)

    fun flip() = motors.setPercentOutput(-0.6)

    fun stop() = motors.setPercentOutput(0.0)

    override suspend fun default() {
        try {
            periodic {
                val speed = Math.abs(Drivetrain.speed)
                if (autoIntake && Math.abs(Drivetrain.speed) > 0.5) {
                    val percentMaxSpeed = speed / Drivetrain.MAX_SPEED
                    intake(0.6 + percentMaxSpeed * 0.4)
                } else {
                    stop()
                }
            }
        } finally {
            stop()
        }
    }
}

suspend fun Intake.flipCubes() = use(Intake) {
    periodic {
        flip()
    }
}