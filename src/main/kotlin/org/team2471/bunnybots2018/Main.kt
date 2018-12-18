package org.team2471.bunnybots2018

import org.team2471.frc.lib.framework.*

object Robot : RobotProgram {
    override suspend fun autonomous() {
        Drivetrain.enable()
        Intake.enable()
    }

    override suspend fun teleop() {
        Drivetrain.enable()
        Intake.enable()
    }

    override suspend fun disable() {
        Drivetrain.disable()
        Intake.disable()
    }

    override suspend fun test() {
        println("Test")
    }
}

fun main(args: Array<String>) {
    initializeWpilib()

    Drivetrain
    Intake
    OI

    runRobotProgram(Robot)
}