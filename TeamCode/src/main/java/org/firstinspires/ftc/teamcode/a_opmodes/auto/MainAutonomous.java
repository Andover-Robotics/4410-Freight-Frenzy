package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement.Action;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths.AutoPathElement.Path;
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.Camera;
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.old.TemplateDetector;
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.old.TemplateDetector.PipelineResult;
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines.BarcodePipeline;
import org.firstinspires.ftc.teamcode.b_hardware.Bot;

import java.util.List;

@Autonomous(name = "Main Autonomous", group = "Competition")
public class MainAutonomous extends LinearOpMode {

    private Bot bot;

    PipelineResult detected;
    double confidence;
    TemplateDetector pipeline;
    boolean performActions = true;
    GamepadEx gamepad;
    CommandScheduler scheduler;


    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;
        bot = Bot.getInstance(this);
        scheduler = CommandScheduler.getInstance();

        gamepad = new GamepadEx(gamepad1);
        Camera camera = new Camera(this, "Webcam 1");
        BarcodePipeline pipeline = new BarcodePipeline();
        camera.setPipeline(pipeline);

        AutoPaths paths = new AutoPaths(this);

        //TODO: add initialization here

        //  ie set servo position                             ========================================================================


        while (!isStarted()) {
            if (isStopRequested()) {
                return;
            }
            if (gamepad1.x) {
                performActions = false;
            }
            if (gamepad.wasJustPressed(GamepadKeys.Button.Y)) {

            }

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
        telemetry.addLine(GlobalConfig.alliance + " is selected alliance");
        telemetry.addLine(GlobalConfig.autoType + " is autoType");
        telemetry.update();
        waitForStart();

        List<AutoPathElement> trajectories = paths.getTrajectories(GlobalConfig.autoType, BarcodePipeline.BarcodeResult.CENTER);
//    pipeline.close();


        //Roadrunner stuff

        bot.roadRunner.setPoseEstimate(paths.getStartPose());

        if (isStopRequested())
            return;

        for (AutoPathElement item : trajectories) {

            telemetry.addData("executing path element", item.getName());
            telemetry.update();

            if (item instanceof AutoPathElement.Path) {
                bot.roadRunner.followTrajectory(((Path) item).getTrajectory());
            }

            if (isStopRequested())
                return;
        }
    }
}