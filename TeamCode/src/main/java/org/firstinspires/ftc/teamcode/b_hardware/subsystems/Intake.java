package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake extends SubsystemBase {
    public static final double SPEED = 0.8;
    public MotorEx motor;

    public Intake(OpMode opMode){
        motor = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void run(){
        motor.set(SPEED);
    }

    public void stop(){
        motor.set(0.0);
    }
}
