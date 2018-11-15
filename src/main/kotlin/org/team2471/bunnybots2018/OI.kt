package org.team2471.bunnybots2018

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController

object OI {
    private val driverController = XboxController(0)
    private val operatorController = XboxController(1)

    val driveThrottle: Double
        get() = -driverController.getY(GenericHID.Hand.kLeft)

    val driveTurn: Double
        get() = driverController.getX(GenericHID.Hand.kRight)
}