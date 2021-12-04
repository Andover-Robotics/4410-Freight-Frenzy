package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines.BarcodePipeline
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt

class AutoPaths(val opMode: LinearOpMode) {

    //TODO: reverse this for alliances
    enum class AutoType{
        PARK,
        PARKWAREHOUSE,
        CAROUSEL,
        TESTING
    }

    enum class InitSide{
        CAROUSEL,
        WAREHOUSE
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


    //TODO: Insert pose/vector vals here, Don't use toRadAS in p2d/v2d, Red(-y) is default

    //                                                                  ===================================================
    val initSide = when(GlobalConfig.autoType){
        in listOf(AutoType.CAROUSEL) -> InitSide.CAROUSEL
        in listOf(AutoType.PARKWAREHOUSE, AutoType.PARK) -> InitSide.WAREHOUSE
        else -> InitSide.CAROUSEL
    }

    val startPose = if(initSide == InitSide.CAROUSEL) p2d(-36.0, -72.0+9.0, -90.0.toRadians) else p2d(12.0, -72.0 + 9.0, -90.0.toRadians)

    val dropFreight = if(initSide == InitSide.CAROUSEL) mapOf(//CAROUSEL SIDE
        BarcodePipeline.BarcodeResult.LEFT to listOf(
            makeAction(""){}
        ),
        BarcodePipeline.BarcodeResult.RIGHT to listOf(
            makeAction(""){}
        ),
        BarcodePipeline.BarcodeResult.CENTER to listOf(
            makeAction(""){}
        )
    ) else mapOf(//WAREHOUSE SIDE
        BarcodePipeline.BarcodeResult.LEFT to listOf(
            makeAction(""){}
        ),
        BarcodePipeline.BarcodeResult.RIGHT to listOf(
            makeAction(""){}
        ),
        BarcodePipeline.BarcodeResult.CENTER to listOf(
            makeAction(""){}
        )
    )

    val dropFreightPose = mapOf(
        BarcodePipeline.BarcodeResult.LEFT to getLastPose(dropFreight[BarcodePipeline.BarcodeResult.LEFT]!!),
        BarcodePipeline.BarcodeResult.RIGHT to getLastPose(dropFreight[BarcodePipeline.BarcodeResult.RIGHT]!!),
        BarcodePipeline.BarcodeResult.CENTER to getLastPose(dropFreight[BarcodePipeline.BarcodeResult.CENTER]!!)
    )

    fun getLastPose(paths: List<AutoPathElement>): Pose2d {
        for(i in paths.reversed()){
            if(i is AutoPathElement.Path){
                return i.trajectory.end()
            }
        }
        return startPose
    }

    fun park(result: BarcodePipeline.BarcodeResult): List<AutoPathElement>{
        return run{
            dropFreight[result]!! + listOf(
                    makePath("drive into warehouse",
                            drive.trajectoryBuilder(p2d(12.0, -72.0 + 6.0, 0.0))
                                    .strafeTo(v2d(40.0, -72.0+5.5))
                                    .build())
            )
        }
    }

    fun parkWarehouse(result: BarcodePipeline.BarcodeResult): List<AutoPathElement>{
        return run{
            dropFreight[result]!! + listOf(
                    makePath("drive into warehouse",
                            drive.trajectoryBuilder(p2d(12.0, -72.0 + 6.0, 0.0))
                                    .strafeTo(v2d(40.0, -72.0+5.5))
                                    .build()),
                    makePath("drive in warehouse",
                            drive.trajectoryBuilder(lastPosition)
                                    .strafeTo(v2d(40.0, -40.0))
                                    .build()),
                    makePath("drive inside warehouse",
                            drive.trajectoryBuilder(lastPosition)
                                    .splineToLinearHeading(p2d(72.0-5.5, -40.0, 90.0.toRadians), 0.0)
                                    .build()),
                    makePath("drive out of warehouse",
                            drive.trajectoryBuilder(lastPosition)
                                    .strafeTo(v2d(72.0-5.5, 40.0))
                                    .build())
            )
        }
    }

    fun carousel(result: BarcodePipeline.BarcodeResult): List<AutoPathElement> {
        return run {
            dropFreight[result]!! + listOf(
                makePath("drive to carousel",
                    drive.trajectoryBuilder(dropFreightPose[result]!!)
                        .splineToSplineHeading(p2d(-59.0, -59.0, -135.toRadians), -135.toRadAS)
                        .addSpatialMarker(v2d(-55.0, -55.0)){
                            bot.outtake.resetArm()
                            bot.carousel.run()
                        }
                        .build()
                ),
                makeAction("do carousel"){
                    Thread.sleep(2000)
                        bot.carousel.stop()
                },
                makePath("park",
                    drive.trajectoryBuilder(lastPosition, 45.toRadAS)
                        .splineToSplineHeading(p2d(-72.0 + 9.0, -36.0,  0.0), 180.toRadAS)
                        .build()
                )
            )
        }
    }

    private val trajectorySets: Map<AutoType, Map<BarcodePipeline.BarcodeResult, List<AutoPathElement>>> = mapOf(
        AutoType.PARK to makeMap(::park),
        AutoType.PARKWAREHOUSE to makeMap(::parkWarehouse),
        AutoType.CAROUSEL to makeMap(::carousel)
    )

    fun makeMap(func: (BarcodePipeline.BarcodeResult) -> List<AutoPathElement>): Map<BarcodePipeline.BarcodeResult, List<AutoPathElement>>{
        return mapOf(
            BarcodePipeline.BarcodeResult.LEFT to func(//sus
                BarcodePipeline.BarcodeResult.LEFT),
            BarcodePipeline.BarcodeResult.RIGHT to func(
                BarcodePipeline.BarcodeResult.RIGHT),
            BarcodePipeline.BarcodeResult.CENTER to func(
                BarcodePipeline.BarcodeResult.CENTER));
    }


    fun getTrajectories(b: AutoType, a: BarcodePipeline.BarcodeResult): List<AutoPathElement>{
        return trajectorySets[b]!![a]!!
    }


}