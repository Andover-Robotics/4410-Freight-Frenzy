package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt

class AutoPaths(val opMode: LinearOpMode) {

    //TODO: reverse this for alliances
    enum class AutoType{
        PARK,
        CAROUSEL,
        TESTING
    }
    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory): AutoPathElement(name)
        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit): AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    fun p2d(x: Double, y: Double, h: Double): Pose2d{
        return Pose2d(x, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -y, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h)
    }
    fun p2d(v: Vector2d, h: Double): Pose2d{
        return Pose2d(v.x, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) v.y else -v.y, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else -h)
    }
    fun v2d(x: Double, y: Double): Vector2d{
        return Vector2d(x, if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else -y)
    }
    fun bypass(startPose: Pose2d, endPose: Pose2d): Pose2d{
        val dif = endPose.minus(startPose).vec()
        return Pose2d(startPose.vec().plus(dif.div(dif.norm() * 10)), startPose.heading)
    }
    fun als(d: Double): Double{
        return if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) d else -d;
    }
    fun bypassVec(startVec: Vector2d, endVec: Vector2d): Vector2d{
        val dif = endVec.minus(startVec)
        return startVec.plus(dif.div(dif.norm() * 10))
    }
    fun bypassStraight(startPose: Pose2d, angle: Double): Pose2d{
        return Pose2d(startPose.vec().plus(startPose.vec().rotated(angle - 90)), startPose.heading)
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))
    val Double.toRadAS get() = (if(GlobalConfig.alliance == GlobalConfig.Alliance.RED) toRadians(this) else -toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    val Int.toRadAS get() = (this.toDouble().toRadAS)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path{
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action{
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit): MarkerCallback {
        override fun onMarkerReached() = func()
    }

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                "to ${Math.toDegrees(to).roundToInt()}deg") {
            bot.roadRunner.turn(to - from)
        }
    }


    //TODO: insert action vals here

    //                                                                  =======================================================

    //example
    //private val shootRings = AutoPathElement.Action("Shoot 3 rings") {
    //        bot.shooter.shootRings(opMode, 3, 0.8)
    //        bot.shooter.turnOff()
    //        Thread.sleep(1000)
    //    }


    //Copy from here on into AutoPathVisualizer ==============================================================================

    //TODO: Insert pose/vector vals here

    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )


    val startPose = p2d(-24.0, -72.0+9.0, -90.0.toRadians)

    //TODO: Make Trajectories in trajectorySets

    val dropFreight = mapOf(
        TemplateDetector.PipelineResult.LEFT to listOf(
            makeAction("test"){

            }
        ),
        TemplateDetector.PipelineResult.RIGHT to listOf(
            makeAction("test"){

            }
        ),
        TemplateDetector.PipelineResult.MIDDLE to listOf(
            makeAction("test"){

            }
        )
    )

    val dropFreightPose = mapOf(
        TemplateDetector.PipelineResult.LEFT to getLastPose(dropFreight[TemplateDetector.PipelineResult.LEFT]!!),
        TemplateDetector.PipelineResult.RIGHT to getLastPose(dropFreight[TemplateDetector.PipelineResult.RIGHT]!!),
        TemplateDetector.PipelineResult.MIDDLE to getLastPose(dropFreight[TemplateDetector.PipelineResult.MIDDLE]!!)
    )

    fun getLastPose(paths: List<AutoPathElement>): Pose2d {
        for(i in paths.reversed()){
            if(i is AutoPathElement.Path){
                return i.trajectory.end()
            }
        }
        return startPose
    }

    fun park(result: TemplateDetector.PipelineResult): List<AutoPathElement>{
        return run{
            dropFreight[result]!! + listOf(
                makePath("drive into warehouse",
                    drive.trajectoryBuilder(startPose)
                        .forward(72.0)
                        .build())
            )
        }
    }

    fun carousel(result: TemplateDetector.PipelineResult): List<AutoPathElement> {
        return run {
            dropFreight[result]!! + listOf(
                makePath("drive to carousel",
                    drive.trajectoryBuilder(dropFreightPose[result]!!)
                        .splineToSplineHeading(p2d(-59.0, -59.0, -135.toRadians), -135.toRadAS)
                        .addSpatialMarker(v2d(-55.0, -55.0)){
                            bot.carousel.run()
                        }
                        .build()
                ),
                makeAction("do carousel"){
                    Thread.sleep(2000)
                    bot.carousel.stop()
                },
                makePath("drive to park",
                    drive.trajectoryBuilder(lastPosition, 45.toRadAS)
                        .splineToSplineHeading(p2d(0.0, -72.0+5.3, 0.0), -45.toRadAS)
                        .build()
                ),
                makePath("move into warehouse",
                    drive.trajectoryBuilder(lastPosition)
                        .lineToConstantHeading(v2d(48.0, -72.0+5.3))
                        .build()
                )
            )
        }
    }

    //                                                                          ====================================================
    private val trajectorySets: Map<AutoType, Map<TemplateDetector.PipelineResult, List<AutoPathElement>>> = mapOf(
        AutoType.PARK to mapOf(
            TemplateDetector.PipelineResult.LEFT to park(TemplateDetector.PipelineResult.LEFT),
            TemplateDetector.PipelineResult.RIGHT to park(TemplateDetector.PipelineResult.RIGHT),
            TemplateDetector.PipelineResult.MIDDLE to park(TemplateDetector.PipelineResult.MIDDLE)
        ),
        AutoType.CAROUSEL to mapOf(
            TemplateDetector.PipelineResult.LEFT to carousel(TemplateDetector.PipelineResult.LEFT),
            TemplateDetector.PipelineResult.RIGHT to carousel(TemplateDetector.PipelineResult.RIGHT),
            TemplateDetector.PipelineResult.MIDDLE to carousel(TemplateDetector.PipelineResult.MIDDLE)
        ),
        AutoType.TESTING to mapOf(
            TemplateDetector.PipelineResult.MIDDLE to run{
                listOf(
                    makePath("do stuff",
                        drive.trajectoryBuilder(startPose)
                            .forward(20.0)
                            .build()
                    )
                )
            },
            TemplateDetector.PipelineResult.LEFT to run{
                listOf(
                    makePath("forward 8",
                        drive.trajectoryBuilder(startPose)
                            .lineToConstantHeading(v2d(0.0, 24.0))
                            .splineToConstantHeading(v2d(0.001, 24.0), 0.0)
                            .lineToConstantHeading(v2d(24.0, 24.0))
                            .build())
                )
            },
            TemplateDetector.PipelineResult.RIGHT to run{
                val c1 = Pose2d()
                val c2 = p2d(0.0, 24.0, 90.0.toRadians)
                val c3 = p2d(24.0, 24.0, 180.0.toRadians)
                val c4 = p2d(24.0, 0.0, -90.0.toRadians)
                listOf(
                    makeAction("set pose"){
                        drive.poseEstimate = Pose2d()
                    },
                    makePath("testing",
                        drive.trajectoryBuilder(c1)
                            .lineToConstantHeading(c2.vec())
                            .splineToConstantHeading(bypassVec(c2.vec(), c3.vec()), 0.0)
                            .lineToConstantHeading(c3.vec())
                            .splineToConstantHeading(bypassVec(c3.vec(), c4.vec()), -90.0.toRadAS)
                            .lineToConstantHeading(c4.vec())
                            .splineToConstantHeading(bypassVec(c4.vec(), c1.vec()), 180.0.toRadAS)
                            .lineToConstantHeading(c1.vec())
                            .splineToConstantHeading(bypassVec(c1.vec(), c3.vec()), 45.0.toRadAS)
                            .lineToSplineHeading(c3)
                            .splineToConstantHeading(bypassVec(c3.vec(), c4.vec()), -90.0.toRadAS)
                            .lineToSplineHeading(c4)
                            .splineToConstantHeading(bypassVec(c4.vec(), c2.vec()), 135.0.toRadAS)
                            .lineToSplineHeading(c2)
                            .splineToConstantHeading(bypassVec(c2.vec(), c1.vec()), -90.0.toRadAS)
                            .lineToSplineHeading(c1)
                            .splineToSplineHeading(c4, 0.0)
                            .splineToSplineHeading(c3, 90.0.toRadAS)
                            .splineToSplineHeading(c2, 180.0.toRadAS)
                            .splineToSplineHeading(c1, -90.0.toRadAS)
                            .build()
                    )
                )
            }
        )
    )


    fun getTrajectories(b: AutoType, a: TemplateDetector.PipelineResult): List<AutoPathElement>{
        return trajectorySets[b]!![a]!!
    }


}