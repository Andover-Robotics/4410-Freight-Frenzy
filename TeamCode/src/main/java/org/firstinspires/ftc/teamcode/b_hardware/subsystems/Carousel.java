package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Carousel extends SubsystemBase {
    private final OpMode opMode;
    private MotorEx motor;
    private boolean running = false;

    public static int SPEED = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? 1000 : -1000;

    public Carousel (OpMode opMode) {
        this.opMode = opMode;

        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVeloCoefficients(1, 0, 0);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        SPEED = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? 1000 : -1000;
    }

    public void init(){
        SPEED = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? 1000 : -1000;
    }

    public void runCarousel(){
        running = true;
        motor.set(SPEED);
    }

    public void stopCarousel(){
        running = false;
        motor.set(0);
    }

    @Override
    public void periodic() {
        if(running){
            motor.set(SPEED);
        }else{
            motor.set(0);
        }
    }
}
