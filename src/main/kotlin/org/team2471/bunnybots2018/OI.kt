package org.team2471.bunnybots2018

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.bHold
import org.team2471.frc.lib.framework.createMappings
import org.team2471.frc.lib.framework.leftBumperHold
import org.team2471.frc.lib.framework.use
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

    val direction: Uptake.Direction?
        get() {
            val left = operatorController.getBumper(GenericHID.Hand.kLeft)
            val right = operatorController.getBumper(GenericHID.Hand.kRight)

            return when {
                left == right -> null
                left -> Uptake.Direction.LEFT
                right -> Uptake.Direction.RIGHT
                else -> null
            }
        }
    val leftSpit: Double
        get() = operatorController.getTriggerAxis(GenericHID.Hand.kLeft) * 0.6
    val rightSpit: Double
        get() = operatorController.getTriggerAxis(GenericHID.Hand.kRight) * 0.6

    init {
        driverController.createMappings {
            leftBumperHold { Intake.flipCubes() }
        }
        operatorController.createMappings {
            bHold {
                use(Uptake) {
                    periodic {
                        Uptake.antiJam()
                    }
                }
            }
        }
    }
}