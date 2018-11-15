package org.team2471.bunnybots2018

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Solenoid

object Ballsorter {
    private val lowerHorizontalBelt = TalonSRX(TODO())
    private val verticalBelt = TalonSRX(TODO())
    private val ballPusher = Solenoid(TODO())
//    private val topHorizontalBelt = TalonSRX(TODO())

    fun uptake (speed: Double) {
        lowerHorizontalBelt.set(ControlMode.PercentOutput, speed)
        verticalBelt.set(ControlMode.PercentOutput, speed)
    }

    fun pushBall (eject: Boolean){
        ballPusher.set(eject)
    }
}