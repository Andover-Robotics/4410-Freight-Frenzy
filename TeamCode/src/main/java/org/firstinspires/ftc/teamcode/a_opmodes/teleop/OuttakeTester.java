package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name="outtakeTest", group = "testing")
public class OuttakeTester extends BaseOpMode{
    private double cycle = 0, prevRead = 0;
    private TimingScheduler timingScheduler;
    @Override
    void subInit() {
        timingScheduler = new TimingScheduler(this);
    }

    @Override
    void subLoop() {
        cycle = 1.0/(time-prevRead);
        prevRead = time;
        telemetry.addData("cycle", cycle);
        timingScheduler.run();

        if(gamepad1.b){
            bot.intake.runIntake();
        }else if(gamepad1.left_bumper){
            bot.intake.spitIntake();
        }else{
            bot.intake.stopIntake();
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            bot.intake.flipUp();
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            bot.intake.flipDown();
        }

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            CommandScheduler.getInstance().schedule(bot.intake.transferToOuttake());
        }
        bot.outtake.turret.driveManual(-stickSignal(Direction.RIGHT).getX());
        bot.outtake.slides.driveManual(stickSignal(Direction.LEFT).getY());
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            CommandScheduler.getInstance().schedule(bot.outtake.runToIntake());
        }
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
            CommandScheduler.getInstance().schedule(bot.outtake.runToOuttake());
        }

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.X)){
            bot.outtake.toggleBucket();
        }
        telemetry.addLine("a to toggle claw, x to reset bucket");
        telemetry.addData("turret", bot.outtake.turret.getTarget());
        telemetry.addData("turret current", bot.outtake.turret.getPosition());
        telemetry.addData("slides", bot.outtake.slides.getTarget());
        telemetry.addData("slides current", bot.outtake.slides.getPosition());
        telemetry.update();
        CommandScheduler.getInstance().run();

    }
}
