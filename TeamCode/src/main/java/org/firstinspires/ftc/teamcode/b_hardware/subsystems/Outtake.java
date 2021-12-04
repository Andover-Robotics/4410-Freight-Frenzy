package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Outtake extends SubsystemBase {
    private MotorEx motor;
    private Servo bucket;
    private Servo flap;
    private int target;

    public static int armHigh = -1441;
    public static int armMid = -888;
    public static int armLow = -536;
    public static int armDown = -2763;
    public static double bucketUp = 0.1;
    public static double bucketDown = 0.5;
    public static double flapClose = 0.9;
    public static double flapOpen = 0.6;
    
    public static double kS = 0.05;
    public static double kV = 1;

    public Outtake(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "arm", Motor.GoBILDA.RPM_312);
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setFeedforwardCoefficients(kS, kV);
        motor.setPositionTolerance(50);

        bucket = opMode.hardwareMap.servo.get("bucket");
        flap = opMode.hardwareMap.servo.get("flap");
    }
    //TODO: test non four bar bucket

    private void goToPosition(int ticks){
        target = ticks;
        motor.setTargetPosition(ticks);
    }

    public void runPeriodic(){
        ((Runnable)() -> {
            while(!motor.atTargetPosition()){
                motor.set(0.3);
            }
            motor.stopMotor();
        }).run();
        return;
    }

    @Override
    public void periodic() {
        if(motor.atTargetPosition()) {
            motor.stopMotor();
        }else{
            int x = Math.abs(target - motor.getCurrentPosition());
            motor.set(Math.pow(Math.abs((x * x - 6000 * x) / 18000000.0), 2));//TODO: figure out "spline" power/motor path
            Log.d("armPower", Double.toString(Math.pow(Math.abs((x * x - 6000 * x) / 18000000.0), 2)));
        }
    }

    public void openFlap(){
        flap.setPosition(flapOpen);
    }

    public void closeFlap(){
        flap.setPosition(flapClose);
    }

    public void bucketDrop(){
        bucket.setPosition(bucketDown);
    }

    public void bucketUp(){
        bucket.setPosition(bucketUp);
    }
    
    

    public void resetArm(){
        bucketUp();
        openFlap();
        goToPosition(armDown);
    }

    public void armHigh(){
        closeFlap();
        bucketDrop();
        goToPosition(armHigh);
    }

    public void armMid(){
        closeFlap();
        bucketDrop();
        goToPosition(armMid);
    }

    public void armLow(){
        closeFlap();
        bucketDrop();
        goToPosition(armLow);
    }
}
