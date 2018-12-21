package org.team2471.bunnybots2018

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.team2471.frc.lib.coroutines.periodic
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.math.windRelativeAngles
import org.team2471.frc.lib.motion_profiling.Path2D
import org.team2471.frc.lib.vector.Vector2

suspend fun Drivetrain.driveAlongPath(path: Path2D) = use(Drivetrain) {
    println("Driving along path ${path.name}, duration: ${path.durationWithSpeed}, travel direction: ${path.robotDirection}, mirrored: ${path.isMirrored}")
    Drivetrain.highGear()
    path.resetDistances()
    zeroDistances()
    var prevLeftDistance = 0.0
    var prevRightDistance = 0.0
    var prevLeftVelocity = 0.0
    var prevRightVelocity = 0.0
    var prevTime = 0.0

    val pathAngleEntry = table.getEntry("Path Angle")
    val angleErrorEntry = table.getEntry("Angle Error")
    val gyroCorrectionEntry = table.getEntry("Gyro Correction")

    val leftPositionErrorEntry = table.getEntry("Left Position Error")
    val rightPositionErrorEntry = table.getEntry("Right Position Error")

    val leftVelocityErrorEntry = table.getEntry("Left Velocity Error")
    val rightVelocityErrorEntry = table.getEntry("Right Velocity Error")

    val timer = Timer().apply { start() }

    var angleErrorAccum = 0.0
    try {
        periodic {
            val t = timer.get()
            val dt = t - prevTime

            // apply gyro corrections to the distances
            val gyroAngle = gyroAngle
            val pathAngle = Math.toDegrees(Vector2.angle(path.getTangent(t)))
            val angleError = pathAngle - windRelativeAngles(pathAngle, gyroAngle)

            angleErrorAccum = angleErrorAccum * GYRO_CORRECTION_I_DECAY + angleError

            val gyroCorrection = if (SmartDashboard.getBoolean("Use Gyro", true)) {
                angleError * GYRO_CORRECTION_P + angleErrorAccum * GYRO_CORRECTION_I
            } else {
                0.0
            }

            val leftDistance = path.getLeftDistance(t) + gyroCorrection
            val rightDistance = path.getRightDistance(t) - gyroCorrection

            val leftVelocity = (leftDistance - prevLeftDistance) / dt
            val rightVelocity = (rightDistance - prevRightDistance) / dt

            val leftVelocityError = leftSpeed - leftVelocity
            val rightVelocityError = rightSpeed - rightVelocity

            val velocityDeltaTimesCoefficient = (leftVelocity - rightVelocity) * TURNING_FEED_FORWARD

            pathAngleEntry.setDouble(pathAngle)
            angleErrorEntry.setDouble(angleError)
            leftPositionErrorEntry.setDouble(leftError)
            rightPositionErrorEntry.setDouble(rightError)
            leftVelocityErrorEntry.setDouble(leftVelocityError)
            rightVelocityErrorEntry.setDouble(rightVelocityError)

            gyroCorrectionEntry.setDouble(gyroCorrection)

            val leftFeedForward = leftVelocity * LEFT_FEED_FORWARD_COEFFICIENT +
                    (LEFT_FEED_FORWARD_OFFSET * Math.signum(leftVelocity)) +
                    velocityDeltaTimesCoefficient

            val rightFeedForward = rightVelocity * RIGHT_FEED_FORWARD_COEFFICIENT +
                    (RIGHT_FEED_FORWARD_OFFSET * Math.signum(rightVelocity)) -
                    velocityDeltaTimesCoefficient

            driveToSetPoints(leftDistance, rightDistance, leftFeedForward, rightFeedForward)

            if (t >= path.durationWithSpeed) {
                exitPeriodic()
            }

            prevTime = t
            prevLeftDistance = leftDistance
            prevRightDistance = rightDistance
            prevLeftVelocity = leftVelocity
            prevRightVelocity = rightVelocity
        }
    } finally {
        stop()
    }
}
