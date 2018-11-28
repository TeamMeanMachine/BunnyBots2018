package org.team2471.bunnybots2018

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Solenoid
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem

object Ballsorter : DaemonSubsystem("Ballsorter") {
    private val lowerHorizontalBelt = TalonSRX(TODO())
    private val verticalBelt = TalonSRX(TODO())
    private val ballPusher = Solenoid(TODO())
    private val colorSensor = DigitalInput(TODO())
    private val topHorizontalBelt = TalonSRX(TODO())

    val lastBallColor: Color
        get() = if (colorSensor.get()) Color.RED else Color.BLUE

    fun uptake (uptakeDirection: UptakeDirection) {
        lowerHorizontalBelt.set(ControlMode.PercentOutput, 1.0)
        verticalBelt.set(ControlMode.PercentOutput, 1.0)

        topHorizontalBelt.set(ControlMode.PercentOutput, if (uptakeDirection == UptakeDirection.LEFT) -1.0 else 1.0)

        ballPusher.set(lastBallColor == Color.BLUE)
    }

    fun stop(){
        lowerHorizontalBelt.set(ControlMode.PercentOutput, 0.0)
        verticalBelt.set(ControlMode.PercentOutput, 0.0)
        topHorizontalBelt.set(ControlMode.PercentOutput, 0.0)
        ballPusher.set(false)
    }

    override suspend fun default() {
        periodic {
            val direction = OI.uptakeDirection

            if (direction != null) uptake(direction) else stop()

            false
        }
    }

    enum class Color {
        RED,
        BLUE,
    }

    enum class UptakeDirection {
        LEFT,
        RIGHT,
    }
}