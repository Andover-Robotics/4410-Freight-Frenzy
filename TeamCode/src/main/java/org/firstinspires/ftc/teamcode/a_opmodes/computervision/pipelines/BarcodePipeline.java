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
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.stream.Collectors;

public class BarcodePipeline extends OpenCvPipeline {//TODO: reminder https://acmerobotics.github.io/ftc-dashboard/features

    public enum BarcodeResult {
        LEFT,
        CENTER,
        RIGHT;

        //        @NonNull EOCV-Sim error
        @Override
        public String toString() {
            return super.toString();
        }
    }

    BarcodeResult result = BarcodeResult.LEFT;//TODO: add continuous updating
    Map<BarcodeResult, Long> totalTime = new HashMap<>();
    Telemetry telemetry;

    public BarcodePipeline() {
        isRed = GlobalConfig.alliance == GlobalConfig.Alliance.RED;
    }

    public BarcodePipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
        isRed = GlobalConfig.alliance == GlobalConfig.Alliance.RED;
    }

    Mat blur = new Mat(),
            a = new Mat(),
            contourFill = new Mat(),
            inputHLS = new Mat(),
            duck = new Mat(),
            barcode = new Mat(),
            barcode1 = new Mat(),
            barcode2 = new Mat(),
            barcode3 = new Mat(),
            output = new Mat();
    long startTime = System.currentTimeMillis();
    Size kernel = new Size(19, 19);

    //    Scalar lowerBoundDuck = new Scalar(10, 69, 140),
//            upperBoundDuck = new Scalar(71, 198, 255),
//            lowerBoundBarcode = new Scalar(80, 70, 45),
//            upperBoundBarcode = new Scalar(130, 200, 180);//blue
////            lowerBoundBarcode = new Scalar(141, 133, 78),//HSL -> HLS reversed
////            upperBoundBarcode = new Scalar(180, 255, 255);//red
    private static class GlobalConfig {
        public enum Alliance {
            RED,
            BLUE;
        }

        public static final Alliance alliance = Alliance.RED;
    }

    boolean doBothBarcodes = true,
            pause = false,
            isRed = GlobalConfig.alliance == GlobalConfig.Alliance.RED;//Checks both red and blue at the same time(for those who are lazy)
    Scalar redLower1 = new Scalar(141, 0, 60),
            redUpper1 = new Scalar(180, 255, 255),
            redLower2 = new Scalar(0, 0, 60),
            redUpper2 = new Scalar(40, 255, 255),
            blueLower = new Scalar(80, 70, 45),
            blueUpper = new Scalar(130, 200, 180);//TODO test values
    Scalar lowerBoundDuck = new Scalar(10, 69, 140),
            upperBoundDuck = new Scalar(71, 198, 255);

    @Override
    public Mat processFrame(Mat input) {
        input.convertTo(input, CV_8UC1);
        Imgproc.cvtColor(input, inputHLS, Imgproc.COLOR_RGB2HLS);
        Imgproc.GaussianBlur(inputHLS, blur, kernel, 0, 0);
        Core.inRange(inputHLS, lowerBoundDuck, upperBoundDuck, duck);
        if (!doBothBarcodes) {
            if (isRed) {
                Core.inRange(inputHLS, redLower1, redUpper1, barcode1);
                Core.inRange(inputHLS, redLower2, redUpper2, barcode2);
                Core.add(barcode1, barcode2, barcode);
            } else {
                Core.inRange(inputHLS, blueLower, blueUpper, barcode);
            }
        } else {
            Core.inRange(inputHLS, redLower1, redUpper1, barcode1);
            Core.inRange(inputHLS, redLower2, redUpper2, barcode2);
            Core.inRange(inputHLS, blueLower, blueUpper, barcode3);
            Core.add(barcode1, barcode2, barcode);
            Core.add(barcode, barcode3, barcode);
        }
        List<MatOfPoint> barcodeContours = new ArrayList<>(), duckContours = new ArrayList<>();
        Imgproc.findContours(barcode, barcodeContours, a, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(duck, duckContours, a, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : duckContours) {//separate duck contours
            if (contour.toList().stream().min(Comparator.comparingDouble((p) -> p.x)).get().x > inputHLS.size().width * 1 / 16 &&//TODO: tune these
                    contour.toList().stream().max(Comparator.comparingDouble((p) -> p.x)).get().x < inputHLS.size().width * 31 / 32 &&
                    Imgproc.contourArea(contour) < 1200 && Imgproc.contourArea(contour) > 100) {
                filteredContours.add(contour);
            }
        }
        double duckY = 0, duckX = 0;
        try {
            duckY = filteredContours.stream().mapToDouble(m -> m.toList().stream().min(Comparator.comparingDouble((p) -> p.y)).get().y).max().getAsDouble();
            duckX = filteredContours.stream().mapToDouble(m -> m.toList().stream().mapToDouble(p -> p.x).average().getAsDouble()).average().getAsDouble();
        } catch (NoSuchElementException e) {
            telemetry.addLine("No duck found");
        }
        for (MatOfPoint contour : barcodeContours) {//separate barcode contours
            if (contour.toList().stream().min(Comparator.comparingDouble((p) -> p.x)).get().x > inputHLS.size().width * 1 / 16 &&//TODO: tune these
                    contour.toList().stream().max(Comparator.comparingDouble((p) -> p.x)).get().x < inputHLS.size().width * 31 / 32 &&
                    contour.toList().stream().mapToDouble((p) -> p.y).sum() / contour.toList().size() > duckY - 20 &&
                    !(Math.abs(contour.toList().stream().mapToDouble(p -> p.x).average().getAsDouble() - duckX) < 3) &&
                    Imgproc.contourArea(contour) < 400 && Imgproc.contourArea(contour) > 20) {
                filteredContours.add(contour);
            }
        }

        if (filteredContours.isEmpty()) {
            return input;
        }

        double maxBarcode = filteredContours.stream().mapToDouble(m -> m.toList().stream().max(Comparator.comparingDouble(p -> p.y)).get().y).max().getAsDouble();
        filteredContours = filteredContours.stream().filter(m -> m.toList().stream().max(Comparator.comparingDouble(p -> p.y)).get().y > maxBarcode - 25).collect(Collectors.toList());

        Core.add(barcode, duck, contourFill);
        filteredContours.sort(Comparator.comparingDouble(a -> a.toList().stream().mapToDouble((p) -> p.x).sum() / a.toList().size()));
        ArrayList<Pair<Double, Scalar>> colorsHLS = new ArrayList<>();//0 left, 1 mid, 2 right
        Scalar[] colorsRGB = new Scalar[filteredContours.size()];
//        output = input;
        output = new Mat(input.size(), input.type(), new Scalar(0, 0, 0));
        Imgproc.drawContours(output, barcodeContours, -1, new Scalar(255, 0, 255));
        Imgproc.drawContours(output, duckContours, -1, new Scalar(255, 255, 0));
        for (int i = 0; i < filteredContours.size(); i++) {
            MatOfPoint contour = filteredContours.get(i);
            Mat temp = new Mat(input.size(), CV_8U, new Scalar(0, 0, 0));
            List<MatOfPoint> tempContours = new ArrayList<>();
            tempContours.add(contour);
            Imgproc.fillPoly(temp, tempContours, new Scalar(255, 255, 255));
            colorsHLS.add(new Pair<>(contour.toList().stream().mapToDouble(p -> p.x).average().getAsDouble(), Core.mean(inputHLS, temp)));
            colorsRGB[i] = Core.mean(input, temp);
            Imgproc.fillPoly(output, tempContours, colorsRGB[i]);
        }
        double middle = filteredContours.stream().mapToDouble(m -> m.toList().stream().mapToDouble(p -> p.x).average().getAsDouble()).average().getAsDouble();
        int posIndex = 0;
        colorsHLS.sort(Comparator.comparingDouble(a -> colorScore(a.second)));
        try {
            result = colorsHLS.get(0).first > middle + 5 ? BarcodeResult.RIGHT : colorsHLS.get(0).first < middle - 5 ? BarcodeResult.LEFT : BarcodeResult.CENTER;
            if (!pause) {
                try {
                    totalTime.replace(result, totalTime.get(result) + (System.currentTimeMillis() - startTime));
                } catch (NullPointerException e) {
                    totalTime.put(result, 0L);
                }
            }
            telemetry.addLine("total contours " + filteredContours.size());
        } catch (IndexOutOfBoundsException e) {
            result = BarcodeResult.LEFT;
            telemetry.addLine("No contours found");
        }
        telemetry.addLine("I think it's on the " + getResult());
        telemetry.addLine("this took " + (System.currentTimeMillis() - startTime) + " milliseconds");
        telemetry.addLine("sums:");
        for (Map.Entry<BarcodeResult, Long> e : totalTime.entrySet()) {
            telemetry.addData(e.getKey().toString(), e.getValue());
        }
        telemetry.update();
        startTime = System.currentTimeMillis();

        return output;
    }

    public BarcodeResult getResult() {
        return totalTime.entrySet().stream().max(Comparator.comparingLong(Map.Entry::getValue)).get().getKey();
    }

    @Override
    public void onViewportTapped() {
        totalTime.clear();
    }

    private static final int hue = 60, sat = 100;//Duck color

    private double colorScore(Scalar s) {
        return Math.abs(s.val[0] - hue) + Math.abs(s.val[1] - sat);
    }

    private static class Pair<T, V> {
        public final T first;
        public final V second;

        public Pair(T first, V second) {
            this.first = first;
            this.second = second;
        }
    }
}