package org.team2471.bunnybots2018

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem
import org.team2471.frc.lib.framework.use

object Drivetrain : DaemonSubsystem("Drivetrain") {
    private const val EDGES_PER_100_MS = 216 * 4.0 / 10.0
    private const val HIGH_SHIFTPOINT = 5.0
    private const val LOW_SHIFTPOINT = 4.0

    private const val PEAK_CURRENT_LIMIT = 0
    private const val CONTINUOUS_CURRENT_LIMIT = 40
    private const val PEAK_CURRENT_DURATION = 100

    private const val DISTANCE_P = 2.0 * 0.40 // 0.75
    private const val DISTANCE_D = 0.5

    private val leftMaster = TalonSRX(Talons.DRIVE_LEFT_1)
    private val rightMaster = TalonSRX(Talons.DRIVE_RIGHT_1)
//    private val shifter = Solenoid(0)

    val speed: Double get() = Math.abs(-leftMaster.getSelectedSensorVelocity(1)/ EDGES_PER_100_MS +
            rightMaster.getSelectedSensorVelocity(1) / EDGES_PER_100_MS) / 2.0


    init {
        val leftSlave1 = TalonSRX(Talons.DRIVE_LEFT_2)
        val leftSlave2 = TalonSRX(Talons.DRIVE_LEFT_3)
        val rightSlave1 = TalonSRX(Talons.DRIVE_RIGHT_2)
        val rightSlave2 = TalonSRX(Talons.DRIVE_RIGHT_3)

        leftSlave1.follow(leftMaster)
        leftSlave2.follow(leftMaster)
        rightSlave1.follow(rightMaster)
        rightSlave2.follow(rightMaster)

        leftMaster.inverted = true
        leftSlave1.inverted = true
        leftSlave2.inverted = true

        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 10)
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 1, 10)

        leftMaster.setNeutralMode(NeutralMode.Brake)
        leftSlave1.setNeutralMode(NeutralMode.Coast)
        leftSlave2.setNeutralMode(NeutralMode.Coast)

        rightMaster.setNeutralMode(NeutralMode.Brake)
        rightSlave1.setNeutralMode(NeutralMode.Coast)
        rightSlave2.setNeutralMode(NeutralMode.Coast)

        leftMaster.configPeakCurrentLimit(PEAK_CURRENT_LIMIT, 10)
        leftMaster.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, 10)
        leftMaster.configPeakCurrentDuration(PEAK_CURRENT_DURATION, 10)
        leftMaster.enableCurrentLimit(true)
        leftSlave1.configPeakCurrentLimit(PEAK_CURRENT_LIMIT, 10)
        leftSlave1.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, 10)
        leftSlave1.configPeakCurrentDuration(PEAK_CURRENT_DURATION, 10)
        leftSlave1.enableCurrentLimit(true)
        leftSlave2.configPeakCurrentLimit(PEAK_CURRENT_LIMIT, 10)
        leftSlave2.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, 10)
        leftSlave2.configPeakCurrentDuration(PEAK_CURRENT_DURATION, 10)
        leftSlave2.enableCurrentLimit(true)

        rightMaster.configPeakCurrentLimit(PEAK_CURRENT_LIMIT, 10)
        rightMaster.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, 10)
        rightMaster.configPeakCurrentDuration(PEAK_CURRENT_DURATION, 10)
        rightMaster.enableCurrentLimit(true)
        rightSlave1.configPeakCurrentLimit(PEAK_CURRENT_LIMIT, 10)
        rightSlave1.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, 10)
        rightSlave1.configPeakCurrentDuration(PEAK_CURRENT_DURATION, 10)
        rightSlave1.enableCurrentLimit(true)
        rightSlave2.configPeakCurrentLimit(PEAK_CURRENT_LIMIT, 10)
        rightSlave2.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT, 10)
        rightSlave2.configPeakCurrentDuration(PEAK_CURRENT_DURATION, 10)
        rightSlave2.enableCurrentLimit(true)

        leftMaster.configClosedloopRamp(0.1, 10)
        leftMaster.configOpenloopRamp(0.0, 10)
        leftMaster.config_kP(0, DISTANCE_P, 10)
        leftMaster.config_kD(0, DISTANCE_D, 10)
        leftMaster.config_kF(0, 0.0, 10)
        leftMaster.selectProfileSlot(0, 0)

        rightMaster.configClosedloopRamp(0.1, 10)
        rightMaster.configOpenloopRamp(0.0, 10)
        rightMaster.config_kP(0, DISTANCE_P, 10)
        rightMaster.config_kD(0, DISTANCE_D, 10)
        rightMaster.config_kF(0, 0.0, 10)
        rightMaster.selectProfileSlot(0, 0)
    }

    fun drive(throttle: Double, softTurn: Double, hardTurn: Double) {
        val totalTurn = (softTurn * Math.abs(throttle)) + hardTurn

        var leftPower = throttle + totalTurn
        var rightPower = throttle - totalTurn

        val maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower))
        if (maxPower > 1) {
            leftPower /= maxPower
            rightPower /= maxPower
        }

//        if(speed > HIGH_SHIFTPOINT) {
//            shifter.set(true)
//        } else if (speed < LOW_SHIFTPOINT) {
//            shifter.set(false)
//        }

        leftMaster.set(ControlMode.PercentOutput, leftPower)
        rightMaster.set(ControlMode.PercentOutput, rightPower)
    }

    fun zeroDistances() {
    }

    fun driveToSetPoints(leftDistance: Double, rightDistance: Double) {
    }

    override suspend fun default() {
        periodic {
            val throttle = OI.driveThrottle
            val softTurn = OI.softTurn
            val hardTurn = OI.hardTurn
            drive(throttle, softTurn, hardTurn)
        }
    }
}

suspend fun Drivetrain.driveAlongPath() = use(Drivetrain) {
}
