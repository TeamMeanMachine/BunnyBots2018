package org.team2471.bunnybots2018

import org.team2471.frc.lib.framework.RobotProgram
import org.team2471.frc.lib.framework.initializeWpilib
import org.team2471.frc.lib.framework.runRobotProgram

object Robot : RobotProgram {
    override suspend fun autonomous() {
        println("Autonomous")
    }

    override suspend fun teleop() {
        println("Teleop")
    }

    override suspend fun disable() {
        println("Disable")
    }

    override suspend fun test() {
        println("Test")
    }
}

fun main(args: Array<String>) {
    initializeWpilib()

    Drivetrain

    runRobotProgram(Robot)
}