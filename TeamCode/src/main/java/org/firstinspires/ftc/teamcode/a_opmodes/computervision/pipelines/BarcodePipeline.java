package org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines;

//import androidx.annotation.NonNull;

import static org.opencv.core.CvType.CV_32S;
import static org.opencv.core.CvType.CV_8S;
import static org.opencv.core.CvType.CV_8SC1;
import static org.opencv.core.CvType.CV_8UC1;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BarcodePipeline extends OpenCvPipeline {

    public enum BarcodeResult{
        LEFT,
        CENTER,
        RIGHT;

//        @NonNull
        @Override
        public String toString() {
            return super.toString();
        }
    }

    Telemetry telemetry;

    public BarcodePipeline(){}

    public BarcodePipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    Mat blur = new Mat(),
    a = new Mat(),
    b = new Mat(),
    c = new Mat(),
    contourFill = new Mat(),
    duck = new Mat(),
    barcode = new Mat(),
    output = new Mat();



    Size kernel = new Size(9,9);
    Scalar lowerBoundBarcode = new Scalar(141, 133, 78),//HSL -> HLS reversed
    upperBoundBarcode = new Scalar(180, 255, 255),
    lowerBoundDuck = new Scalar(10, 69, 140),
    upperBoundDuck = new Scalar(71, 198, 255);
    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HLS);
        input.convertTo(input, CV_8UC1);
        Imgproc.GaussianBlur(input, blur, kernel, 0, 0);

        Core.inRange(input, lowerBoundDuck, upperBoundDuck, duck);
        Core.inRange(input, lowerBoundBarcode, upperBoundBarcode, barcode);

        Core.add(barcode, duck, contourFill);
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(contourFill, contours, a, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        List<MatOfPoint> barcodeContours = new ArrayList<>();

        for(MatOfPoint contour : contours){//separate barcode contours
            if(contour.toList().stream().mapToDouble((p) -> p.x).sum()/contour.toList().size() > input.size().width * 3 / 16 &&//TODO: tune this
                    contour.toList().stream().mapToDouble((p) -> p.y).sum()/contour.toList().size() > input.size().height * 5 / 16 &&
                    Imgproc.contourArea(contour) < 1500 && Imgproc.contourArea(contour) > 100){
                barcodeContours.add(contour);
            }
        }

        output = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));
        Imgproc.fillPoly(output, barcodeContours, new Scalar(255, 255, 255));
        Imgproc.cvtColor(input, input, Imgproc.COLOR_HLS2RGB);
        Core.copyTo(input, output, output);



//        Imgproc.drawContours(output, contours, -1, new Scalar(50, 100, 100, 1));


        telemetry.addLine("b");
        telemetry.update();
//        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
//        input.convertTo(input, CV_8UC1);
//        Imgproc.GaussianBlur(input, input, kernel, 0, 0);
//        Core.inRange(input, new Scalar(15, 46, 161), new Scalar(180, 255, 255), input);
//        List<MatOfPoint> contour = new ArrayList<>();
//        Imgproc.findContours(input, contour, c, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
//        Imgproc.drawContours(input, contour, -1, new Scalar(0, 100, 100, 0));
        return output;
    }
}
