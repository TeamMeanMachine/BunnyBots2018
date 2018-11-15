package org.team2471.bunnybots2018

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Solenoid
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem

object Drivetrain : DaemonSubsystem("Drivetrain") {
    private const val EDGES_PER_100_MS = 216 * 4.0 / 10.0 //uh this is probably wrong idk what the actual number is idsfl;kafjiod;sajfoipw;das
    private const val HIGH_SHIFTPOINT = 5.0
    private const val LOW_SHIFTPOINT = 4.0

    private val leftMaster = TalonSRX(0)
    private val rightMaster = TalonSRX(15)
    private val shifter = Solenoid(0)

    val speed: Double get() = Math.abs(-leftMaster.getSelectedSensorVelocity(1)/ EDGES_PER_100_MS +
            rightMaster.getSelectedSensorVelocity(1) / EDGES_PER_100_MS) / 2.0


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

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 10)
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 10)
    }

    fun drive(throttle: Double, softTurn: Double, hardTurn: Double){
        val totalTurn = (softTurn * Math.abs(throttle)) + hardTurn

        var leftPower = throttle + totalTurn
        var rightPower = throttle - totalTurn

        val maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower))
        if (maxPower > 1) {
            leftPower /= maxPower
            rightPower /= maxPower
        }

        if(speed > HIGH_SHIFTPOINT) {
            shifter.set(true)
        } else if (speed < LOW_SHIFTPOINT) {
            shifter.set(false)
        }

        leftMaster.set(ControlMode.PercentOutput, leftPower)
        rightMaster.set(ControlMode.PercentOutput, rightPower)
    }

    override suspend fun default() {
        println("Made it to default")
        periodic {
            val throttle = OI.driveThrottle
            val hardTurn = OI.hardTurn
            val softTurn = OI.softTurn
            drive(throttle, hardTurn, softTurn)
            false
        }
    }
}