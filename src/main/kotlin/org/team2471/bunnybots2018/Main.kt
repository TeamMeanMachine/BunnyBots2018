package org.team2471.bunnybots2018

import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.*

object Robot : RobotProgram {
    override suspend fun autonomous() {
        Drivetrain.enable()
        Intake.enable()

        Drivetrain.brake()
    }

    override suspend fun teleop() {
        Drivetrain.enable()
        Intake.enable()

        Drivetrain.brake()
    }

    override suspend fun disable() {
        Drivetrain.disable()
        Intake.disable()

        suspendUntil { Math.abs(Drivetrain.speed) < 1.0 }
        Drivetrain.coast()
    }

    override suspend fun test() {
        println("Test")
    }
}

fun main(args: Array<String>) {
    initializeWpilib()

    Drivetrain
    Intake
    Uptake
    OI

    runRobotProgram(Robot)
}