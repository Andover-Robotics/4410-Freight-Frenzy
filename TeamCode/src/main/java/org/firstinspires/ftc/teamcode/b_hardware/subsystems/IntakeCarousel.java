package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class IntakeCarousel extends SubsystemBase {
    private MotorEx motor;
    public static final int INTAKE = 1000;
    public static final int CAROUSEL_MAG = 750;
    private final int CAROUSEL;

    public IntakeCarousel(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);//TODO: find motor rpm
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setVeloCoefficients(5, 0, 0.3);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        CAROUSEL = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? CAROUSEL_MAG : -CAROUSEL_MAG;//TODO: test direction
    }

    public void runCarousel(){
        runCarousel(CAROUSEL);
    }

    public void runCarousel(int speed){
        motor.setVelocity(speed);
    }

    public void runIntake(){
        motor.setVelocity(INTAKE);
    }

    public void runIntake(double power){
        motor.motorEx.setPower(power);
    }

    public void stop(){
        motor.stopMotor();
    }
}
