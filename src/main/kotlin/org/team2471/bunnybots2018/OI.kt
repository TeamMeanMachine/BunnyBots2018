package org.team2471.bunnybots2018

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import org.team2471.frc.lib.framework.createMappings
import org.team2471.frc.lib.framework.leftBumperHold
import org.team2471.frc.lib.math.deadband

object OI {
    private val driverController = XboxController(0)
    private val operatorController = XboxController(1)

    val driveThrottle: Double
        get() = -driverController.getY(GenericHID.Hand.kLeft)
            .deadband(0.1)

    val softTurn: Double
        get() = driverController.getX(GenericHID.Hand.kRight)
            .deadband(0.1)

    val hardTurn: Double
        get() = driverController.getTriggerAxis(GenericHID.Hand.kRight) - driverController.getTriggerAxis(GenericHID.Hand.kLeft)

    val uptakeDirection: Ballsorter.UptakeDirection?
        get() {
            val left = operatorController.getTriggerAxis(GenericHID.Hand.kLeft)
                    .deadband(0.2)
            val right = operatorController.getTriggerAxis(GenericHID.Hand.kRight)
                    .deadband(0.2)

            return when {
                left > right -> Ballsorter.UptakeDirection.LEFT
                right > left -> Ballsorter.UptakeDirection.RIGHT
                else -> null
            }
        }

    init {
        driverController.createMappings {
            leftBumperHold { Intake.flipCubes() }
        }
    }
}