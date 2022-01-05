package org.firstinspires.ftc.teamcode.a_opmodes.computervision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvPipeline;

public class Recognizer<T extends OpenCvPipeline> {
    private T pipeline;
    private Camera camera;

    public Recognizer(OpMode opMode, String name, T pipeline){
        this.pipeline = pipeline;
        camera = new Camera(opMode, name, pipeline);
    }
}