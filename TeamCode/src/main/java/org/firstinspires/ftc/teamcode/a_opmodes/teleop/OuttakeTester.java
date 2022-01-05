package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import android.icu.lang.UProperty;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name="outtakeTest", group = "testing")
public class OuttakeTester extends BaseOpMode{
    private double cycle = 0, prevRead = 0;
    private TimingScheduler timingScheduler;
    @Override
    void subInit() {
        timingScheduler = new TimingScheduler(this);
        bot.outtake.setManual(true);
    }

    @Override
    void subLoop() {
        cycle = 1.0/(time-prevRead);
        prevRead = time;
        telemetry.addData("cycle", cycle);
        timingScheduler.run();

        if(gamepad1.b){
            bot.intakeCarousel.runIntake();
        }else{
            bot.intakeCarousel.stop();
        }

        if(bot.outtake.getManual()) {
            if (Math.abs(stickSignal(Direction.RIGHT).getX()) > 0.1) {
                bot.outtake.turret.driveManual(-stickSignal(Direction.RIGHT).getX());
            } else {
                telemetry.addLine("turret stopped");
                bot.outtake.turret.stop();
            }

            if (Math.abs(stickSignal(Direction.RIGHT).getY()) > 0.1) {
                bot.outtake.arm.driveManual(-stickSignal(Direction.RIGHT).getY());
            } else {
                telemetry.addLine("arm stopped");
                bot.outtake.arm.stop();
            }

            if (Math.abs(stickSignal(Direction.LEFT).getY()) > 0.1) {
                bot.outtake.slides.driveManual(stickSignal(Direction.LEFT).getY());
            } else {
                telemetry.addLine("slides stopped");
                bot.outtake.slides.stop();
            }
        }

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            CommandScheduler.getInstance().schedule(bot.outtake.slides.new RunTo(Outtake.slidesClosed));
        }

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            CommandScheduler.getInstance().schedule(bot.outtake.runToIntake());
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            CommandScheduler.getInstance().schedule(bot.outtake.runToOuttake());
        }

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.Y)){
            bot.outtake.toggleManual();
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.A)){
            bot.outtake.toggleClaw();
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.X)){
            bot.outtake.toggleBucket();
        }
        telemetry.addLine("a to toggle claw, x to reset bucket");
        telemetry.addData("turret", bot.outtake.turret.getTarget());
        telemetry.addData("turret current", bot.outtake.turret.getPosition());
        telemetry.addData("arm", bot.outtake.arm.getTarget());
        telemetry.addData("arm current", bot.outtake.arm.getPosition());
        telemetry.addData("slides", bot.outtake.slides.getTarget());
        telemetry.addData("slides current", bot.outtake.slides.getPosition());
        telemetry.addData("manual", bot.outtake.getManual());
        telemetry.update();
        CommandScheduler.getInstance().run();

    }
}
