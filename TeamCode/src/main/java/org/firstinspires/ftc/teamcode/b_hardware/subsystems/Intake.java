package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {

    private MotorEx intake;
    private Servo flip;
    private boolean isTransfering = false;
    public static final int INTAKE = 1000;
    public static final int HOLDING = 100;
    public static final int SPIT = -10000;
    public static final double FLIP_DOWN = 1.0, FLIP_UP = 0.2;

    public Intake (OpMode opMode) {
        intake = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_312);
        intake.setRunMode(Motor.RunMode.VelocityControl);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.setVeloCoefficients(5, 0, 0.3);
        intake.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        flip = opMode.hardwareMap.servo.get("flip");
        flip.setDirection(Servo.Direction.FORWARD);
    }

    public Command transferToOuttake = new SequentialCommandGroup(
            new InstantCommand(() -> isTransfering = true),
            new InstantCommand(this::holdIntake),
            new InstantCommand(this::flipUp),
            new WaitCommand(800),
            new ParallelRaceGroup(new RunCommand(this::spitIntake),
                    new WaitCommand(1000)),
            new InstantCommand(this::stopIntake),
            new InstantCommand(this::flipDown),
            new InstantCommand(() -> isTransfering = false)
    );


    public void runIntake() {
        if(!isTransfering) {
            intake.setVelocity(INTAKE);
        }
    }

    public void holdIntake(){
        intake.setVelocity(HOLDING);
    }

    public void stopIntake() {
        intake.stopMotor();
    }

    public void spitIntake() {
        intake.setVelocity(SPIT);
    }

    public void flipUp(){
        flip.setPosition(FLIP_UP);
    }

    public void flipDown(){
        flip.setPosition(FLIP_DOWN);
    }
}
