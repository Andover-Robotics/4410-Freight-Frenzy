package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends SubsystemBase {

    private final OpMode opMode;
    private MotorEx intake;
    private Servo flip;
    private ColorSensor sensor;

    private boolean isTransfering = false;
    public static double INTAKE = 0.9;
    public static double HOLDING = 0.5;
    public static double SPIT = -0.9;
    public static double FLIP_DOWN = 1.0, FLIP_UP = 0.25, FLIP_SHARED = 0.75;

    public Intake (OpMode opMode) {
        this.opMode = opMode;
        intake = new MotorEx(opMode.hardwareMap, "intake", Motor.GoBILDA.RPM_312);
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intake.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        flip = opMode.hardwareMap.servo.get("flip");
        flip.setDirection(Servo.Direction.FORWARD);

        sensor = opMode.hardwareMap.colorSensor.get("colorSensor");
        sensor.enableLed(true);

        init();
    }

    public void init(){
        flipDown();
    }

    public Command transferToOuttake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> isTransfering = true),
                new InstantCommand(this::flipUp),
                new ParallelRaceGroup(
                        new RunCommand(this::holdIntake),
                        new WaitCommand(1000)
                ),
                new ParallelRaceGroup(new RunCommand(this::spitIntake),
                        new WaitCommand(1000)),
                new InstantCommand(this::stopIntake),
                new InstantCommand(this::flipDown),
                new InstantCommand(() -> isTransfering = false)
        );
    }


    public void runIntake() {
        if(!isTransfering) {
            intake.setVelocity(INTAKE);
        }
    }

    public void flipShared(){
        flip.setPosition(FLIP_SHARED);
    }

    public void holdIntake(){
        intake.setVelocity(HOLDING);
    }

    public void stopIntake() {
        intake.stopMotor();
    }

    public void runSlow(){
        intake.set(-0.1);
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

    @Override
    public void periodic() {
//        if(sensor.alpha() > 2500 && !isTransfering){
//            isTransfering = true;
//            CommandScheduler.getInstance().schedule(transferToOuttake());
//        }
    }
}
