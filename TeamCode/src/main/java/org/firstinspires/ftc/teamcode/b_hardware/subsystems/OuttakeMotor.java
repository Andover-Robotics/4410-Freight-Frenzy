package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OuttakeMotor extends SubsystemBase {
    private MotorEx motor;
    private int target = 0;
    private final int TOLERANCE,
        UPPERBOUND,
        LOWERBOUND,
        manualDiv,
        staticGain,
        posGain,
        maxAccel;
    private boolean manual = false, stopped = false;

    
    public OuttakeMotor(OpMode opMode, String name, Motor.GoBILDA rpm, double kp, double ki, double kd, int tol, int ub, int lb, int md, int sg, int pg){
        motor = new MotorEx(opMode.hardwareMap, name, rpm);
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setVeloCoefficients(kp, ki, kd);
        TOLERANCE = tol;
        UPPERBOUND = ub;
        LOWERBOUND = lb;
        manualDiv = md;
        staticGain = sg;
        posGain = pg;
        maxAccel = 4;//TODO make this
    }

    public class RunTo extends CommandBase {
        private int target;
        public RunTo(int t) {
            target = t;
            addRequirements(OuttakeMotor.this);
        }

        @Override
        public void initialize() {
            manual = false;
            setTarget(target);
        }

        @Override
        public void end(boolean interrupted) {
            motor.stopMotor();
            manual = true;
        }

        @Override
        public boolean isFinished() {
            return atTargetPosition();
        }
    }

    private void setTarget(int t){
        target = t;
    }

    public void driveManual(double p){
        stopped = false;
        if(manual && motor.getCurrentPosition() > LOWERBOUND && motor.getCurrentPosition() < UPPERBOUND) {
            motor.set(p / manualDiv);
        }else if(motor.getCurrentPosition() < LOWERBOUND && p > 0){
            motor.set(p / manualDiv);
        }else if(motor.getCurrentPosition() > UPPERBOUND && p < 0){
            motor.set(p / manualDiv);
        }
    }

    public void stop(){
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
            if (atTargetPosition()) {
                motor.stopMotor();
            } else {
                motor.setVelocity(getDifference() * posGain + (Double.compare(getDifference(), 0) * staticGain));//TODO change this to more smooth
            }
        }else{
            target = motor.getCurrentPosition();
            if(stopped){
                if (atTargetPosition()) {
                    motor.set(0);
                    motor.stopMotor();
                } else {
                    motor.setVelocity(getDifference() * posGain + (Double.compare(getDifference(), 0) * staticGain));//TODO change this to more smooth
                }
            }
        }
    }

    public boolean atTargetPosition(){
        return Math.abs(getDifference()) < TOLERANCE;
    }

    private int getDifference(){
        return target - motor.getCurrentPosition();
    }
}
