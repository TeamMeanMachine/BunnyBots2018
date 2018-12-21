package org.team2471.bunnybots2018

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.Timer
import org.team2471.frc.lib.actuators.TalonSRX
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.DaemonSubsystem

object Uptake : DaemonSubsystem("Uptake") {
    private const val UPTAKE_SPEED = 0.7
    private const val FEEDER_SPEED = 0.5
    private const val SEPERATOR_SPEED = 0.6

    private val lowerHorizontalBelt = TalonSRX(Talons.LOWER_HORIZ_BELT).config {
        inverted(true)
    }
    private val verticalBelt = TalonSRX(Talons.VERTICAL_BELT)
    private val ballSorter = Solenoid(Solenoids.SORTER)
    private val colorSensor = DigitalInput(0)
    private val topHorizontalBelt = TalonSRX(Talons.UPPER_HORIZ_BELT)
    private val leftSpitter = TalonSRX(Talons.LEFT_SPITTER).config {
        inverted(true)
    }

    private val rightSpitter = TalonSRX(Talons.RIGHT_SPITTER)

    private val timer = Timer().apply { start() }

    val ballColor: Color
        get() = if (colorSensor.get()) Color.BLUE else Color.RED

    private var lastBallColor = ballColor

    fun uptake(direction: Direction, filterBalls: Boolean) {
        lowerHorizontalBelt.setPercentOutput(FEEDER_SPEED)
        verticalBelt.setPercentOutput(UPTAKE_SPEED)

        topHorizontalBelt.setPercentOutput(if (direction == Direction.LEFT) SEPERATOR_SPEED else -SEPERATOR_SPEED)

//        ballSorter.set(ballColor == Color.BLUE)
//        val ballColor = ballColor
//        if (lastBallColor != ballColor && ballColor == Color.BLUE) timer.reset()
//        lastBallColor = ballColor
//
//        ballSorter.set(filterBalls && timer.get() < 0.1)
    }

    fun spit(direction: Direction, power: Double, filterBalls: Boolean) {
        uptake(direction, filterBalls)
        if (direction == Direction.LEFT) {
            leftSpitter.setPercentOutput(power)
            rightSpitter.setPercentOutput(0.0)
        } else {
            rightSpitter.setPercentOutput(power)
            leftSpitter.setPercentOutput(0.0)
        }
    }

    fun rawSpit(leftPower:Double, rightPower: Double) {
        leftSpitter.setPercentOutput(leftPower)
        rightSpitter.setPercentOutput(rightPower)
    }

    fun stop() {
        lowerHorizontalBelt.setPercentOutput(0.0)
        verticalBelt.setPercentOutput(0.0)
        topHorizontalBelt.setPercentOutput(0.0)
        rightSpitter.setPercentOutput(0.0)
        leftSpitter.setPercentOutput(0.0)
        ballSorter.set(false)
    }

    override suspend fun default() {
        periodic {
            val leftSpit = OI.leftSpit
            val rightSpit = OI.rightSpit

            if (leftSpit == rightSpit) {
                val direction = OI.direction
                if (direction != null) uptake(direction, true) else stop()
            } else if (leftSpit > rightSpit) {
                spit(Direction.LEFT, leftSpit, true)
            } else {
                spit(Direction.RIGHT, rightSpit, true)
            }
        }
    }

    enum class Color {
        RED,
        BLUE,
    }

    enum class Direction {
        LEFT,
        RIGHT,
    }
}