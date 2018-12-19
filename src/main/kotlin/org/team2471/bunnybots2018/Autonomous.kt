package org.team2471.bunnybots2018

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.team2471.frc.lib.coroutines.delay
import org.team2471.frc.lib.coroutines.parallel
import org.team2471.frc.lib.coroutines.suspendUntil
import org.team2471.frc.lib.framework.use
import org.team2471.frc.lib.motion_profiling.Autonomi
import org.team2471.frc.lib.motion_profiling.Autonomous
import org.team2471.frc.lib.util.measureTimeFPGA
import java.io.File

object AutoChooser {
    private val cacheFile = File("/home/lvuser/autonomi.json")

    private var autonomi: Autonomi

    private val testAutoChooser = SendableChooser<String?>().apply {
        addDefault("None", null)
        addObject("8 Foot Circle", "8 Foot Circle")
        addObject("4 Foot Circle", "4 Foot Circle")
        addObject("2 Foot Circle", "2 Foot Circle")
        addObject("Drive Straight", "8 Foot Straight")
    }

    private val autoChooser = SendableChooser<String>().apply {
        addDefault("Simple Auto", "Simple Auto")
    }

    init {
        SmartDashboard.putData("Test Path Chooser", testAutoChooser)
        SmartDashboard.putData("Auto Chooser", autoChooser)

        // load cached autonomi
        try {
            autonomi = Autonomi.fromJsonString(cacheFile.readText())
            println("Autonomi cache loaded.")
        } catch (_: Exception) {
            DriverStation.reportError("Autonomi cache could not be found", false)
            autonomi = Autonomi()
        }

        NetworkTableInstance.getDefault()
            .getTable("PathVisualizer")
            .getEntry("Autonomi").addListener({ event ->
                val json = event.value.string
                if (!json.isEmpty()) {
                    val t = measureTimeFPGA {
                        autonomi = Autonomi.fromJsonString(json)
                    }
                    println("Loaded autonomi in $t seconds")

                    cacheFile.writeText(json)
                    println("New autonomi written to cache")
                } else {
                    autonomi = Autonomi()
                    DriverStation.reportWarning("Empty autonomi received from network tables", false)
                }
            }, EntryListenerFlags.kImmediate or EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
    }

    suspend fun runAuto() = use(Drivetrain, Intake, Uptake) {
        val testPath = testAutoChooser.selected

        if (testPath != null) {
            Drivetrain.driveAlongPath(autonomi.getPath("Tests", testPath))
            return@use
        }

        val auto = autoChooser.selected
        when (auto) {
            "Simple Auto" -> simpleAuto(autonomi[auto])
            else -> throw IllegalStateException("Unknown auto: $auto")
        }
    }
}

private suspend fun simpleAuto(autonomous: Autonomous) {
    Intake.intake(1.0)
    parallel({
        Drivetrain.driveAlongPath(autonomous["Forward"])
        println("Path 1 Done")
    }, {
        Uptake.rawSpit(0.3, 0.0)
        delay(0.5)
        Uptake.uptake(Uptake.Direction.RIGHT, false)
    })

    parallel({
        Drivetrain.driveAlongPath(autonomous["Backward"])
        println("Path 2 Done")
    }, {
        suspendUntil { Drivetrain.distance < -6.0 }
        Uptake.spit(Uptake.Direction.RIGHT, 0.3, false)
    })
    Uptake.stop()
}