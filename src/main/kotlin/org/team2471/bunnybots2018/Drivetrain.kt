package org.team2471.workshop

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem

object Drivetrain : DaemonSubsystem("Drivetrain") {
    private val leftMaster = TalonSRX(0)
    private val rightMaster = TalonSRX(15)

    init {
        val leftSlave1 = TalonSRX(1)
        val leftSlave2 = TalonSRX(2)
        val rightSlave1 = TalonSRX(14)
        val rightSlave2 = TalonSRX(13)

        leftSlave1.follow(leftMaster)
        leftSlave2.follow(leftMaster)
        rightSlave1.follow(rightMaster)
        rightSlave2.follow(rightMaster)

        leftMaster.inverted = true
        leftSlave1.inverted = true
        leftSlave2.inverted = true
    }

    fun drive(throttle: Double, turn: Double){
        leftMaster.set(ControlMode.PercentOutput, throttle + turn)
        rightMaster.set(ControlMode.PercentOutput, throttle - turn)
    }

    override suspend fun default() {
        println("Made it to default")
        periodic {
            val throttle = OI.driveThrottle
            val turn = OI.driveTurn
            drive(throttle, turn)
            false
        }
    }
}