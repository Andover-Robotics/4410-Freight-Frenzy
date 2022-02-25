package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.Camera
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.old.TemplateDetector.PipelineResult
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines.BarcodePipeline
import org.firstinspires.ftc.teamcode.b_hardware.Bot

@Autonomous(name = "Main Autonomous KT", group = "Competition")
class MainAutonomousKT : LinearOpMode() {
    lateinit var bot: Bot
    lateinit var detected: PipelineResult
    private var performActions = true
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
        scheduler.schedule(bot.outtake.runInwards(600, 0));
        bot.init();
        while (!isStarted) {
            if (isStopRequested) {
                return
            }
            if (gamepad1.x) {
                performActions = false
            }

            scheduler.run()

      // keep getting results from the pipeline
            telemetry.addLine("result " + pipeline.result.toString())
            telemetry.addLine("\n performActions " + performActions)
            telemetry.addLine(GlobalConfig.alliance.toString() + " is selected alliance")
            telemetry.addLine(GlobalConfig.autoType.toString() + " is autoType")
            telemetry.update()
        }

        //Pipeline stuff

//    while (!isStarted()) {
//
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
            paths.getTrajectories(GlobalConfig.autoType, BarcodePipeline.BarcodeResult.LEFT)
        //    pipeline.close();



        val commands: SequentialCommandGroup = SequentialCommandGroup()
        for(i in trajectories.indices){
            commands.addCommands(getCommand(trajectories, i))
        }
        scheduler.schedule(commands)

        //Roadrunner stuff
        bot.roadRunner.poseEstimate = paths.startPose

//
//        scheduler.onCommandFinish{
//        Log.d("autonomous", "finished command")
//            currentPath++;
//            if(currentPath >= trajectories.size){
//                requestOpModeStop()
//            }else{
//                scheduleElement(trajectories, currentPath)
//            }
//        }
//        scheduleElement(trajectories, 0);
//
        while(!isStopRequested){
            scheduler.run()
//            telemetry.addData("running ", trajectories[currentPath].name)
//            telemetry.update()
        }

    }

    private fun scheduleElement(trajectories: List<AutoPathElement>, currentPath: Int){
        scheduler.schedule(getCommand(trajectories, currentPath))
    }

    private fun getCommand(trajectories: List<AutoPathElement>, currentPath: Int): CommandBase{
        val command: CommandBase
        if(trajectories[currentPath] is AutoPathElement.Path){
            command = FollowTrajectory(bot, (trajectories[currentPath] as AutoPathElement.Path).trajectory)
        }else if(trajectories[currentPath] is AutoPathElement.Action){
            command = if(performActions) {
                (trajectories[currentPath] as AutoPathElement.Action).command
            }else{
                WaitCommand(1000)
            }
        }else{
            command = if(performActions) {
                ParallelCommandGroup(
                        FollowTrajectory(
                                bot,
                                (trajectories[currentPath] as AutoPathElement.ActionPath).trajectory
                        ),
                        (trajectories[currentPath] as AutoPathElement.ActionPath).command
                )
            }else{
                FollowTrajectory(
                        bot,
                        (trajectories[currentPath] as AutoPathElement.ActionPath).trajectory
                )
            }
        }
        return command
    }
}