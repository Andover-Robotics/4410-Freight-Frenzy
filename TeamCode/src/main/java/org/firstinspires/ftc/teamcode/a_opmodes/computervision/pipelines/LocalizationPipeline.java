package org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class LocalizationPipeline extends OpenCvPipeline {//TODO: add alliance hub + shipping hub + freight pipeline

    Telemetry telemetry;

    public LocalizationPipeline(){}

    public LocalizationPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        telemetry.addLine("hello");
        telemetry.update();
        return null;
    }
}
