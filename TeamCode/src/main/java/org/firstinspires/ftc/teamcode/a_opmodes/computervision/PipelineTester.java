package org.firstinspires.ftc.teamcode.a_opmodes.computervision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines.BarcodePipeline;

@Autonomous(name = "Pipeline Tester", group = "Competition")
public class PipelineTester extends LinearOpMode {
    Recognizer<BarcodePipeline> recognizer;

    @Override
    public void runOpMode() throws InterruptedException {
        recognizer = new Recognizer<>(this, "Webcam 1", new BarcodePipeline(telemetry));
        while(!isStopRequested()){

        }
    }
}