package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class Carousel extends SubsystemBase {
    public static final double SPEED = GlobalConfig.SubsystemValues.carouselSpeed;
    private MotorEx motor;

    public Carousel(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, " carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(GlobalConfig.alliance == GlobalConfig.Alliance.RED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        motor.setVeloCoefficients(5, 0, 0.03);
    }

    public void run(){
        motor.setVelocity(SPEED);
    }

    public void stop(){
        motor.stopMotor();
    }

}
