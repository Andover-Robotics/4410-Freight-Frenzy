package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.Camera
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.old.TemplateDetector
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.old.TemplateDetector.PipelineResult
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines.BarcodePipeline
import org.firstinspires.ftc.teamcode.b_hardware.Bot

@Autonomous(name = "Main Autonomous KT", group = "Competition")
class MainAutonomousKT : LinearOpMode() {
    lateinit var bot: Bot
    lateinit var detected: PipelineResult
    lateinit var pipeline: TemplateDetector
    var performActions = true
    lateinit var gamepad: GamepadEx
    lateinit var scheduler: CommandScheduler
    var currentPath = 0

    class FollowTrajectory(val bot: Bot, val trajectory: Trajectory) : CommandBase() {
        override fun initialize() = bot.roadRunner.followTrajectoryAsync(trajectory)
        override fun execute() = bot.roadRunner.update()
        override fun isFinished() = !bot.roadRunner.isBusy
    }


    override fun runOpMode() {
        Bot.instance = null
        bot = Bot.getInstance(this)
        scheduler = CommandScheduler.getInstance()



//        gamepad = GamepadEx(gamepad1)
        val camera = Camera(this, "Webcam 1", BarcodePipeline(telemetry))
        val pipeline = BarcodePipeline()
        camera.setPipeline(pipeline)
        val paths = AutoPaths(this)


        //TODO: add initialization here

        //  ie set servo position                             ========================================================================
        while (!isStarted) {
            if (isStopRequested) {
                return
            }
            if (gamepad1.x) {
                performActions = false
            }
            telemetry.addLine(GlobalConfig.alliance.toString() + " is selected alliance")
            telemetry.addLine(GlobalConfig.autoType.toString() + " is autoType")
            telemetry.update()
        }

        //Pipeline stuff

//    while (!isStarted()) {
//      if (isStopRequested())
//        return;
//      // keep getting results from the pipeline
//      pipeline.currentlyDetected()
//          .ifPresent((pair) -> {
//            telemetry.addData("detected", pair.first);
//            telemetry.addData("confidence", pair.second);
//            telemetry.update();
//            detected = pair.first;
//            confidence = pair.second;
//          });
//      if (gamepad1.x) {
//        performActions = false;
//      }
//      if (gamepad.wasJustPressed(Button.Y)) {
//        pipeline.saveImage();
//      }
//    }
//
//    pipeline.currentlyDetected().ifPresent(pair -> {
//      detected = pair.first;
//      confidence = pair.second;
//    });
//
//    if (detected == null)
//      detected = PipelineResult.LEFT;

        if(isStopRequested)
            return
        ///TODO add buildable trajectories
        val trajectories =
            paths.getTrajectories(GlobalConfig.autoType, BarcodePipeline.BarcodeResult.CENTER)
        //    pipeline.close();

        scheduler.onCommandFinish{
            currentPath++;
            if(currentPath >= trajectories.size){
                requestOpModeStop()
            }else{
                scheduleElement(trajectories, currentPath)
            }
        }

        //Roadrunner stuff
        bot.roadRunner.poseEstimate = paths.startPose
        scheduleElement(trajectories, 0);
        while(!isStopRequested){
            scheduler.run()
            telemetry.addData("running ", trajectories[currentPath].name)
            telemetry.update()
        }
    }

    private fun scheduleElement(trajectories: List<AutoPathElement>, currentPath: Int){
        val command: CommandBase
        if(trajectories[currentPath] is AutoPathElement.Path){
            command = FollowTrajectory(bot, (trajectories[currentPath] as AutoPathElement.Path).trajectory)
        }else if(trajectories[currentPath] is AutoPathElement.Action){
            command = (trajectories[currentPath] as AutoPathElement.Action).command
        }else{
            command = ParallelCommandGroup(
                FollowTrajectory(bot, (trajectories[currentPath] as AutoPathElement.ActionPath).trajectory),
                (trajectories[currentPath] as AutoPathElement.ActionPath).command
            )
        }
        scheduler.schedule(command)
    }
}