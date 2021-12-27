package org.firstinspires.ftc.teamcode.a_opmodes.computervision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.a_opmodes.computervision.pipelines.BarcodePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
    OpenCvWebcam webcam;
    OpenCvWebcam camera;
    private boolean isOn = true;

    public Camera(OpMode opMode, String name) {
        webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, name));
        webcam.setPipeline(new BarcodePipeline());
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//TODO:check this
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    /*Using the Driver Station Camera Preview Feature
For the Control Hub, the DS camera preview feature may prove very helpful for you since it can show
the output of your pipeline without requiring an HDMI monitor or use of scrcpy.

To use this feature, simply make sure that you've started a streaming session during the OpMode initialization.
 When running your OpMode, do NOT press start. Only press INIT. While the OpMode is in the INIT phase,
 you can open the overflow menu on the DriverStation and select the "Camera Stream" option.
 This provides a tap-to-refresh view of the pipeline output.

     */


    public OpenCvWebcam getWebcam() {
        return webcam;
    }

    public void close(){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    /*
     * Specify the image processing pipeline we wish to invoke upon receipt
     * of a frame from the camera. Note that switching pipelines on-the-fly
     * (while a streaming session is in flight) *IS* supported.
     */
    public void setPipeline(OpenCvPipeline pipeline){
        webcam.setPipeline(pipeline);
    }
}
