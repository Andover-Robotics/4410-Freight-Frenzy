package org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DuckPipeline extends OpenCvPipeline {

    public enum DuckResult {
        LEFT,
        CENTER,
        RIGHT;

        @Override
        public String toString() { return super.toString(); }

    }

    @Override
    public Mat processFrame(Mat input) {
        Mat copy = new Mat();
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HLS);

        int low_H = 6,
            low_S = 150,
            low_V = 0;
        
        int high_H = 45,
            high_S = 255,
            high_V = 255;
        
        Core.inRange(input, new Scalar(low_H, low_S, low_V), new Scalar(high_H, high_S, high_V), copy);
        return copy;
    }


}
