package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class SlideMotor extends SubsystemBase {
    private PIDMotor motor;
    private int target = 0;
    private final int UPPERBOUND,
        LOWERBOUND;
    private final double manualDiv;
    private double manualDrive = 0;
    private boolean manual = false, stopped = false;

    private TouchSensor limit;

    enum SlideState{
        IDLE,
        RETRACTING,
        RUNNING
    }
    SlideState slideState = SlideState.IDLE;

    public SlideMotor(OpMode opMode, String name, String limit, Motor.GoBILDA rpm, PIDController controller, DcMotorSimple.Direction direction, int tol, int ub, int lb, double md){
        motor = new PIDMotor(opMode.hardwareMap, name, rpm);
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(direction);
        motor.setController(controller);
        motor.setPositionTolerance(tol);
        UPPERBOUND = ub;
        LOWERBOUND = lb;
        manualDiv = md;

        this.limit = opMode.hardwareMap.touchSensor.get(limit);
    }

    public class RunTo extends CommandBase {
        private final int target;
        public RunTo(int t) {
            target = t;
            addRequirements(SlideMotor.this);
        }

        @Override
        public void initialize() {
            manual = false;
            setTarget(target);
        }

        @Override
        public void end(boolean interrupted) {
            motor.set(0);
            manual = true;
        }

        @Override
        public boolean isFinished() {
            return motor.atTargetPosition();
        }
    }

    public class RunIn extends CommandBase {
        public RunIn(){
            addRequirements(SlideMotor.this);
        }

        @Override
        public void initialize() {
            setTarget(0);
        }

        @Override
        public void execute() {
            if(motor.atTargetPosition()){
                motor.setTargetPosition(motor.getCurrentPosition() - 100);
            }
        }

        @Override
        public void end(boolean interrupted) {
            slideState = SlideState.IDLE;
            motor.setTargetPosition(motor.getCurrentPosition());
            manual = true;
        }

        @Override
        public boolean isFinished() {
            return limit.isPressed();
        }
    }

    private void setTarget(int t){
        motor.setTargetPosition(t);
        target = t;
    }

    public void driveManual(double p){
        if(Math.abs(p) < 0.1){
            manualDrive = 0;
            stopped = true;
        }else {
            stopped = false;
            if (manual && motor.getCurrentPosition() > LOWERBOUND && motor.getCurrentPosition() < UPPERBOUND) {
                manualDrive = p / manualDiv;
            } else if (manual && motor.getCurrentPosition() < LOWERBOUND && p > 0) {
                manualDrive = p / manualDiv;
            } else if (manual && motor.getCurrentPosition() > UPPERBOUND && p < 0) {
                manualDrive = p / manualDiv;
            }else{
                manualDrive = 0;
            }
        }
    }

    public void stop(){
        manualDrive = 0;
        stopped = true;
    }

    public boolean getManual(){
        return manual;
    }

    public void toggleManual(){
        manual = !manual;
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public int getPosition(){
        return motor.getCurrentPosition();
    }

    public int getTarget(){
        return target;
    }

    @Override
    public void periodic() {
        if(!manual) {
            if (motor.atTargetPosition()) {
                motor.set(0);
            } else {
                motor.set(0.3);
            }
        }else{
            if(stopped){
                motor.setRunMode(Motor.RunMode.PositionControl);
                if (motor.atTargetPosition()) {
                    motor.set(0);
                } else {
                    motor.set(0.3);
                }
            }else{
                motor.setRunMode(Motor.RunMode.RawPower);
                motor.set(manualDrive);
                target = motor.getCurrentPosition();
                motor.setTargetPosition(target);
            }
        }
        if(limit.isPressed()){
            motor.resetEncoder();
        }
    }

    public void reset(){
        motor.resetEncoder();
    }
}
