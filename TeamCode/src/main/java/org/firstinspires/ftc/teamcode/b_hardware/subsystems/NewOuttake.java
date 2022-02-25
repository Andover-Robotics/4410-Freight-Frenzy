package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;
@Config
public class NewOuttake {
    private final OpMode opMode;
    public final TurretMotor turret;
    public final SlideMotor slides;
    private final Servo bucketLeft, bucketRight, clamp;




    public static double BUCKET_RIGHT_DOWN = 0.17, BUCKET_RIGHT_OUTTAKE = 0.7, BUCKET_LEFT_DOWN = 0.23, BUCKET_LEFT_OUTTAKE = 0.75, CLAMP_CLOSED = 0.6, CLAMP_OPEN = 0.1;
    private boolean bucketDown = true, clampClosed = true;

    public static int TURRET_INTAKE = 0, TURRET_OUTTAKE = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? -400 : 400, SLIDES_INTAKE = -10, SLIDES_OUTTAKE = 500;
    
    public NewOuttake(OpMode opMode){
        this.opMode = opMode;

        slides = new SlideMotor(opMode, "slides", "slidesLimit", Motor.GoBILDA.RPM_1620, GlobalConfig.SLIDES_PID, DcMotorSimple.Direction.REVERSE, 15, 750, -20 , 1.4);
        turret = new TurretMotor(opMode, "turret", Motor.GoBILDA.RPM_312, GlobalConfig.TURRET_PID, DcMotorSimple.Direction.FORWARD, 16, 600, -600, 4);

        bucketLeft = opMode.hardwareMap.servo.get("bucketLeft");
        bucketLeft.setDirection(Servo.Direction.REVERSE);

        bucketRight = opMode.hardwareMap.servo.get("bucketRight");
        bucketRight.setDirection(Servo.Direction.FORWARD);

        clamp = opMode.hardwareMap.servo.get("clamp");
        clamp.setDirection(Servo.Direction.FORWARD);
    }

    public void init(){
        bucketDown();
        closeClamp();

        TURRET_OUTTAKE = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? -400 : 400;
    }

    public Command runToOuttake(){
        return new SequentialCommandGroup(
                new InstantCommand(this::closeClamp),
                new InstantCommand(this::bucketOuttake),
                runOutwards(TURRET_OUTTAKE, SLIDES_OUTTAKE)
        );
    }

    public Command runToIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(this::openClamp),
                new InstantCommand(this::bucketDown),
                runInwards(TURRET_INTAKE, SLIDES_INTAKE)
        );
    }

    public Command runOutwards(int turretPos, int slidePos){
        return new SequentialCommandGroup(
                turret.new RunTo(turretPos),
                slides.new RunTo(slidePos)
        );
    }

    public Command runInwards(int turretPos, int slidePos){
        return new SequentialCommandGroup(
                new InstantCommand(this::bucketDown),
                slides.new RunTo(slidePos),
                turret.new RunTo(turretPos)
        );
    }



    public void bucketDown(){
        bucketDown = true;
        bucketLeft.setPosition(BUCKET_LEFT_DOWN);
        bucketRight.setPosition(BUCKET_RIGHT_DOWN);
    }

    public void bucketOuttake(){
        bucketDown = false;
        bucketLeft.setPosition(BUCKET_LEFT_OUTTAKE);
        bucketRight.setPosition(BUCKET_RIGHT_OUTTAKE);
    }

    public void toggleBucket(){
        if(bucketDown){
            bucketOuttake();
        }else{
            bucketDown();
        }
    }

    public void closeClamp(){
        clampClosed = true;
        clamp.setPosition(CLAMP_CLOSED);
    }

    public void openClamp(){
        clampClosed = false;
        clamp.setPosition(CLAMP_OPEN);
    }

    public void toggleClamp(){
        if(clampClosed){
            openClamp();
        }else{
            closeClamp();
        }
    }

}
