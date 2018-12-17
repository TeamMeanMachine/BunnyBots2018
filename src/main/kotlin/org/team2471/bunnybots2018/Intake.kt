package org.team2471.bunnybots2018

import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem

object Intake : DaemonSubsystem("Intake") {
    private val motors = TalonSRX(Talons.INTAKE_LEFT, Talons.INTAKE_RIGHT).config {
        openLoopRamp(0.1)
        ctreFollowers[0].inverted = true
    }

    fun intake(speed: Double) = motors.setPercentOutput(speed)

    fun flip() = motors.setPercentOutput(-1.0)

    fun stop() = motors.setPercentOutput(0.0)

    override suspend fun default() {
        try {
            periodic {
//                println("Speed: ${Drivetrain.speed}")
                val speed = Math.abs(Drivetrain.speed)
                if (Math.abs(Drivetrain.speed) > 0.5) {
                    val percentMaxSpeed = speed / Drivetrain.MAX_SPEED
                    intake(0.4 + percentMaxSpeed * 0.6)
                } else {
                    stop()
                }
            }
        } finally {
            stop()
        }
    }
}