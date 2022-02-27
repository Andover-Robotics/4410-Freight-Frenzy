package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GlobalConfig;

@Config
public class Carousel extends SubsystemBase {
    private final OpMode opMode;
    private MotorEx motor;
    private boolean running = false;

    public static int SPEED = 10, STATIC_SPEED = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? 10 : -10;

    public Carousel (OpMode opMode) {
        this.opMode = opMode;

        motor = new MotorEx(opMode.hardwareMap, "carousel", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        init();
    }

    public void init(){
        motor.motor.setDirection(GlobalConfig.alliance == GlobalConfig.Alliance.RED ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        STATIC_SPEED = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? 10 : -10;
    }

    public void run(){
        if(!running) {
            motor.setVelocity(STATIC_SPEED);
        }
    }

    public void stop(){
        if(!running) {
            motor.stopMotor();
        }
    }

    public class RunCarousel extends CommandBase {
        private final OpMode opMode;
        private double startTime;
        private boolean canceled = false;

        public RunCarousel(OpMode opMode){
            this.opMode = opMode;
            startTime = opMode.time;
        }

        @Override
        public void initialize() {
            if(running){
                canceled = true;
                cancel();
            }else {
                running = true;
                startTime = opMode.time;
            }
            addRequirements(Carousel.this);
        }

        @Override
        public void execute() {
            double curTime = opMode.time - startTime;
            if(curTime < 0.7) {
                motor.setVelocity((Math.pow(1.5, curTime * 0.3) - 0.8) * motor.ACHIEVABLE_MAX_TICKS_PER_SECOND * SPEED);
            }
            else if (curTime < 1.2) {
                motor.setVelocity((Math.pow(4, curTime * 0.25) - 0.99) * motor.ACHIEVABLE_MAX_TICKS_PER_SECOND * SPEED);
            }
        }

        @Override
        public void end(boolean interrupted) {
            if(!canceled) {
                motor.stopMotor();
                running = false;
            }
        }

        @Override
        public boolean isFinished() {
            return opMode.time - startTime > 1.2;
        }
    }

}
