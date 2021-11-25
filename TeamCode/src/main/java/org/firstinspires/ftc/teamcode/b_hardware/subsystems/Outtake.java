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
    private int target;
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
    }
    //TODO: test non four bar bucket

    private void goToPosition(int ticks){
        target = ticks;
        motor.setTargetPosition(ticks);
    }

    @Override
    public void periodic() {
        if(motor.atTargetPosition()) {
            motor.stopMotor();
        }else{
            int x = Math.abs(target - motor.getCurrentPosition());
            motor.set(Math.pow(Math.abs((x * x - 6000 * x) / 18000000.0), 2));
            Log.d("armPower", Double.toString(Math.pow(Math.abs((x * x - 6000 * x) / 18000000.0), 2)));
        }
    }

    public void dropFreight(){
        bucket.setPosition(GlobalConfig.SubsystemValues.OuttakeValues.bucketDown);
    }

    public void bucketMid(){
        bucket.setPosition(GlobalConfig.SubsystemValues.OuttakeValues.bucketMid);
    }

    public void resetBucket(){
        bucket.setPosition(GlobalConfig.SubsystemValues.OuttakeValues.bucketUp);
    }

    public void resetArm(){
        resetBucket();
        goToPosition(GlobalConfig.SubsystemValues.OuttakeValues.armDown);
    }

    public void armHigh(){
        bucketMid();
        goToPosition(GlobalConfig.SubsystemValues.OuttakeValues.armHigh);
    }

    public void armMid(){
        bucketMid();
        goToPosition(GlobalConfig.SubsystemValues.OuttakeValues.armMid);
    }

    public void armLow(){
        bucketMid();
        goToPosition(GlobalConfig.SubsystemValues.OuttakeValues.armLow);
    }

    private void setBucket(int pos){
        bucket.setPosition(pos);
    }
}
