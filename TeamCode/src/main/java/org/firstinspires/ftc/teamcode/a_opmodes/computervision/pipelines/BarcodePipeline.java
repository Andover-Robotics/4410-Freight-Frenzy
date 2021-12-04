package org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines;

import static org.opencv.core.CvType.CV_8U;
import static org.opencv.core.CvType.CV_8UC1;



import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BarcodePipeline extends OpenCvPipeline {

    public enum BarcodeResult {
        LEFT,
        CENTER,
        RIGHT;

        //        @NonNull
        @Override
        public String toString() {
            return super.toString();
        }
    }

    BarcodeResult result = BarcodeResult.LEFT;

    Telemetry telemetry;

    public BarcodePipeline() {
    }

    public BarcodePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    Mat blur = new Mat(),
            a = new Mat(),
            b = new Mat(),
            c = new Mat(),
            contourFill = new Mat(),
            inputHLS = new Mat(),
            duck = new Mat(),
            barcode = new Mat(),
            output = new Mat();

    String[] position = new String[]{"left", "center", "right"};


    Size kernel = new Size(19, 19);
    Scalar lowerBoundBarcode = new Scalar(141, 133, 78),//HSL -> HLS reversed
            upperBoundBarcode = new Scalar(180, 255, 255),
            lowerBoundDuck = new Scalar(10, 69, 140),
            upperBoundDuck = new Scalar(71, 198, 255);

    @Override
    public Mat processFrame(Mat input) {

        input.convertTo(input, CV_8UC1);
        Imgproc.cvtColor(input, inputHLS, Imgproc.COLOR_RGB2HLS);
        Imgproc.GaussianBlur(inputHLS, blur, kernel, 0, 0);

        Core.inRange(inputHLS, lowerBoundDuck, upperBoundDuck, duck);
        Core.inRange(inputHLS, lowerBoundBarcode, upperBoundBarcode, barcode);

        Core.add(barcode, duck, contourFill);
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(contourFill, contours, a, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        List<MatOfPoint> barcodeContours = new ArrayList<>();

        for (MatOfPoint contour : contours) {//separate barcode contours
            if (contour.toList().stream().mapToDouble((p) -> p.x).sum() / contour.toList().size() > inputHLS.size().width * 3 / 16 &&//TODO: tune this
                    contour.toList().stream().mapToDouble((p) -> p.y).sum() / contour.toList().size() > inputHLS.size().height * 5 / 16 &&
                    Imgproc.contourArea(contour) < 1500 && Imgproc.contourArea(contour) > 50) {
                barcodeContours.add(contour);
            }
        }

        barcodeContours.sort((a, b) -> a.toList().stream().mapToDouble((p) -> p.x).sum() / a.toList().size() >
                b.toList().stream().mapToDouble((p) -> p.x).sum() / b.toList().size() ? 1 : -1);



        ArrayList<Pair<Integer, Scalar>> colorsHLS = new ArrayList<>();//0 left, 1 mid, 2 right
        Scalar[] colorsRGB = new Scalar[3];
//        output = input;
        output = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));

        for (int i = 0; i < barcodeContours.size(); i++) {
            MatOfPoint contour = barcodeContours.get(i);
            Mat temp = new Mat(input.size(), CV_8U, new Scalar(0, 0, 0));
            List<MatOfPoint> tempContours = new ArrayList<>();
            tempContours.add(contour);
            Imgproc.fillPoly(temp, tempContours, new Scalar(255, 255, 255));
            colorsHLS.add(new Pair<>(i, Core.mean(inputHLS, temp)));
            colorsRGB[i] = Core.mean(input, temp);
            Imgproc.fillPoly(output, tempContours, colorsRGB[i]);
        }

        colorsHLS.sort((a, b) -> colorScore(a.second) > colorScore(b.second) ? 1 : -1);

        switch(colorsHLS.get(0).first){
            case 0:
                result = BarcodeResult.LEFT;
                break;
            case 1:
                result = BarcodeResult.CENTER;
                break;
            default:
                 result = BarcodeResult.RIGHT;
                 break;
        }

        telemetry.addLine("total contours " + barcodeContours.size());
        telemetry.addLine("I think it's on the " + position[colorsHLS.get(0).first]);
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

    public BarcodeResult getResult(){
        return result;
    }

    private static final int hue = 60, sat = 100;

    private double colorScore(Scalar s) {
        return Math.abs(s.val[0] - hue) + Math.abs(s.val[1] - sat);
    }

    private static class Pair<T, V>{
        public final T first;
        public final V second;
        public Pair(T first, V second){
            this.first = first;
            this.second = second;
        }
    }
}
