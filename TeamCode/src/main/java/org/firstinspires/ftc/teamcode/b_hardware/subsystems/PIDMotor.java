package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.jetbrains.annotations.NotNull;

public class PIDMotor extends MotorEx {
    PIDController betterPositionController = new PIDController(1, 0, 0);


    public PIDMotor(@NonNull @NotNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public PIDMotor(@NonNull @NotNull HardwareMap hMap, String id, @NonNull @NotNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public PIDMotor(@NonNull @NotNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }



    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double velocity = veloController.calculate(getCorrectedVelocity(), speed) + feedforward.calculate(speed, getAcceleration());
            motorEx.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = betterPositionController.calculate(encoder.getPosition());
            motorEx.setPower(output * error);
        } else {
            motorEx.setPower(output);
        }
    }

    @Override
    public void setRunMode(RunMode runmode) {
        betterPositionController.reset();
        super.setRunMode(runmode);
    }

    @Override
    public boolean atTargetPosition() {
        return betterPositionController.atSetPoint();
    }

    @Override
    public void setTargetPosition(int target) {//very jank solution
        super.setTargetPosition(target);
        setTargetDistance(positionController.getSetPoint());
    }

    @Override
    public void setTargetDistance(double target) {
        super.setTargetDistance(target);
        betterPositionController.setSetPoint(target);
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        betterPositionController.setTolerance(tolerance);
    }

    public void setPositionTolerance(double positionTolerance, double velocityTolerance){
        betterPositionController.setTolerance(positionTolerance, velocityTolerance);
    }

    public void setBetterPositionController(double kp, double ki, double kd){
        betterPositionController.setPID(kp, ki, kd);
    }

    public void setController(PIDController controller){
        betterPositionController = controller;
        betterPositionController.reset();
    }

    public PIDController getController(){
        return betterPositionController;
    }
}
