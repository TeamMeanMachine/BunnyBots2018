package org.team2471.bunnybots2018

import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.*

object Robot : RobotProgram {
    override suspend fun autonomous() {
        Drivetrain.brake()
        Drivetrain.zeroGyro()
        AutoChooser.runAuto()
    }

    override suspend fun teleop() {

        Drivetrain.brake()
    }

    override suspend fun disable() {
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
    AutoChooser

    runRobotProgram(Robot)
}