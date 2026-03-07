import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel

/*
    == D I S C L A I M E R ==
    Using Kotlin DSL for Gradle builds is not officially supported by FRC/WPILib.
    As such, by using the Kotlin DSL, you acknowledge that you will not receive any support from the WPI Lib team,
    and support from the community will likely be very limited. There are known issues with using Kotlin DSL Gradle
    files in VS Code. As such, if you have team members using VS Code, use of the Kotlin DSL is not recommended.
    The use of Kotlin DSL is only recommend for developers experienced with the Gradle Kotlin DSL.
    (This disclaimer is included at the request of the WPI Lib development team.)
 */
        
plugins {
    java
    id("edu.wpi.first.GradleRIO") version "2026.2.1"
    idea
}


val javaVersion: JavaVersion by extra { JavaVersion.VERSION_17 }
val javaLanguageVersion: JavaLanguageVersion by extra { JavaLanguageVersion.of(javaVersion.toString()) }
val jvmVendor: JvmVendorSpec by extra { JvmVendorSpec.ADOPTIUM }

@Suppress("PropertyName")
val ROBOT_MAIN_CLASS = "frc.robot.sultans3965.Main"

// Define my targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        val roborio by register<RoboRIO>("roborio") {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            // You can use project.frc.getTeamOrDefault(####) instead of project.frc.teamNumber
            // if you want to store a team number in this file.
            team = frc.teamNumber
            debug = frc.getDebugOrDefault(false)
        }

        roborio.artifacts {
            register<FRCJavaArtifact>("frcJava") {
                setJarTask(tasks.jar)
            }

            register<FileTreeArtifact>("frcStaticFileDeploy") {
                files = project.fileTree("src/main/deploy")
                directory = "/home/lvuser/deploy"
                // Change to true to delete files on roboRIO that no longer exist in deploy directory of this project
                deleteOldFiles = false
            }
        }
    }
}

wpi {
    with(java) {
        // Set to true to use debug for JNI.
        debugJni = false
        configureExecutableTasks(tasks.jar.get())
        configureTestTasks(tasks.test.get())
    }

    // Simulation configuration (e.g. environment variables).
    with(sim) {
        addGui().apply {
            defaultEnabled = true
        }
        addDriverstation()
    }
}

// Set this to true to enable desktop support.
val includeDesktopSupport = false

dependencies {
    annotationProcessor(wpi.java.deps.wpilibAnnotations())
    implementation(wpi.java.deps.wpilib())
    implementation(wpi.java.vendor.java())

    roborioDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio))
    roborioDebug(wpi.java.vendor.jniDebug(NativePlatforms.roborio))

    roborioRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio))
    roborioRelease(wpi.java.vendor.jniRelease(NativePlatforms.roborio))

    nativeDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop))
    nativeDebug(wpi.java.vendor.jniDebug(NativePlatforms.desktop))
    simulationDebug(wpi.sim.enableDebug())

    nativeRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop))
    nativeRelease(wpi.java.vendor.jniRelease(NativePlatforms.desktop))
    simulationRelease(wpi.sim.enableRelease())
}

java {
    toolchain {
        languageVersion = javaLanguageVersion
        vendor = jvmVendor
    }
}

tasks {
    compileJava {
        options.encoding = Charsets.UTF_8.name()
        // Configure string concat to always inline compile
        options.compilerArgs.add("-XDstringConcat=inline")
    }

    // Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
    // in order to make them all available at runtime. Also adding the manifest so WPILib
    // knows where to look for our Robot Class.
    jar {
        group = "build"
        manifest(GradleRIOPlugin.javaManifest(ROBOT_MAIN_CLASS))
        duplicatesStrategy = DuplicatesStrategy.INCLUDE

        // Adding this closure makes this expression lazy, allowing GradleRIO to add
        // its dependencies before the jar task is fully configured.
        from({ configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) } })
        
        from({ sourceSets.main.get().allSource })
    }
}

idea {
    project {
        // The project.sourceCompatibility setting is not always picked up, so we set explicitly
        languageLevel = IdeaLanguageLevel(javaVersion)
    }
    module {
        // Improve development & (especially) debugging experience (and IDEA's capabilities) by having libraries' source & javadoc attached
        isDownloadJavadoc = true
        isDownloadSources = true
        // Exclude the .vscode directory from indexing and search
        excludeDirs.add(file(".run" ))
        excludeDirs.add(file(".vscode" ))
    }
}

// Helper Functions to keep syntax cleaner
// @formatter:off
fun DependencyHandler.addDependencies(configurationName: String, dependencies: List<Provider<String>>) = dependencies.forEach { add(configurationName, it) }
fun DependencyHandler.roborioDebug(dependencies: List<Provider<String>>) = addDependencies("roborioDebug", dependencies)
fun DependencyHandler.roborioRelease(dependencies: List<Provider<String>>) = addDependencies("roborioRelease", dependencies)
fun DependencyHandler.nativeDebug(dependencies: List<Provider<String>>) = addDependencies("nativeDebug", dependencies)
fun DependencyHandler.simulationDebug(dependencies: List<Provider<String>>) = addDependencies("simulationDebug", dependencies)
fun DependencyHandler.nativeRelease(dependencies: List<Provider<String>>) = addDependencies("nativeRelease", dependencies)
fun DependencyHandler.simulationRelease(dependencies: List<Provider<String>>) = addDependencies("simulationRelease", dependencies)
fun DependencyHandler.implementation(dependencies: List<Provider<String>>) = dependencies.forEach{ implementation(it) }
fun DependencyHandler.annotationProcessor(dependencies: List<Provider<String>>) = dependencies.forEach{ annotationProcessor(it) }
 // @formatter:on