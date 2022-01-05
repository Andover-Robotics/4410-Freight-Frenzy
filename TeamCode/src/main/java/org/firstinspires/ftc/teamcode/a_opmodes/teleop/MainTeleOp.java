package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.util.Direction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive.Mode;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.TimingScheduler;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {//required vars here
  private double cycle = 0;
  private double prevRead = 0;
  private TimingScheduler timingScheduler;
  private boolean centricity = false;
  private boolean isManual = true;
  private int percent = 1, part = 0;
  private boolean duck = false;




  //config? stuff here =========================================================================

  private double fieldCentricOffset = -90.0;
  public enum TemplateState{
    INTAKE(0.5),
    TRANSPORT(0.5),
    OUTTAKE(0.5);

    public final double progressRate;

    TemplateState(double progressRate){this.progressRate = progressRate;}
  }

  //opmode vars here ==============================================================================================
  //If there is a module-specific var, put it in the module class ie slideStage goes in the slides module



  void subInit() {
    timingScheduler = new TimingScheduler(this);
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement =================================================================================================
    //TODO: change depending on mode
    driveSpeed = 1 - 0.4 * (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//
//    if(justPressed(Button.BACK)){
//      isManual = !isManual;
//    }
//



    if(isManual) {
      drive();
    }else{
      followPath();
    }

    if(buttonSignal(Button.RIGHT_BUMPER)){
      bot.intakeCarousel.runCarousel();
    }else if(buttonSignal(Button.B)){
      bot.intakeCarousel.runIntake();
    }else{
      bot.intakeCarousel.stop();
    }

    if(bot.outtake.getManual()) {
      if (Math.abs(gamepadEx2.getRightX()) > 0.1) {
        bot.outtake.turret.driveManual(-gamepadEx2.getRightX());
      } else {
        telemetry.addLine("turret stopped");
        bot.outtake.turret.stop();
      }

      if (Math.abs(gamepadEx2.getRightY()) > 0.1) {
        bot.outtake.arm.driveManual(-gamepadEx2.getRightY());
      } else {
        telemetry.addLine("arm stopped");
        bot.outtake.arm.stop();
      }

      if (Math.abs(gamepadEx2.getLeftY()) > 0.1) {
        bot.outtake.slides.driveManual(gamepadEx2.getLeftY());
      } else {
        telemetry.addLine("slides stopped");
        bot.outtake.slides.stop();
      }
    }

    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
      CommandScheduler.getInstance().schedule(bot.outtake.runToIntake());
    }
    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
      CommandScheduler.getInstance().schedule(bot.outtake.runToOuttake());
    }

    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.Y)){
      bot.outtake.toggleManual();
    }
    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.A)){
      bot.outtake.toggleClaw();
    }
    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.X)){
      bot.outtake.toggleBucket();
    }




    /*//TODO: make control scheme
    Controller 1
    A:      B:      X:      Y:
    DPAD
    L:      D:     U:      R:
    Joystick
    L:Field centric movement
    R:Set orientation / Rotation (Determine through practice)
    Trigger L/R: slow driving (maybe)
    Bumper
    L:none/switch to previous path      R:none/switch to next path
    Other
    Start:  Back:switch between automation and driving

    Controller 2
    A:      B:      X:      Y:
    DPAD
    L:      D:     U:      R:
    Joystick
    L:movement/reset field centric or progress automation
    R:movement/switch robotfield centric or none
    Trigger L/R: slow driving
    Bumper
    L:none/switch to previous path      R:none/switch to next path
    Other
    Start:  Back:switch between automation and driving
     */


    /*
    AUTOMATION CONTROL SCHEME

     */



    CommandScheduler.getInstance().run();

    // TODO organize this test code
    updateLocalization();
    telemetry.addData("percent", percent);
    telemetry.addData("part", part);
    telemetry.addData("cycle", cycle);
    telemetry.addData("x", bot.roadRunner.getPoseEstimate().getX());
    telemetry.addData("y", bot.roadRunner.getPoseEstimate().getY());
    telemetry.addData("heading", bot.roadRunner.getPoseEstimate().getHeading());
    telemetry.addData("current raw angle", bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle);

    telemetry.addData("turret", bot.outtake.turret.getTarget());
    telemetry.addData("turret current", bot.outtake.turret.getPosition());
    telemetry.addData("arm", bot.outtake.arm.getTarget());
    telemetry.addData("arm current", bot.outtake.arm.getPosition());
    telemetry.addData("slides", bot.outtake.slides.getTarget());
    telemetry.addData("slides current", bot.outtake.slides.getPosition());
    telemetry.addData("manual", bot.outtake.getManual());
  }


  private void drive(){//Driving ===================================================================================
    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).secondAngle//TODO: make sure that the orientation + imu axes is correct
            - fieldCentricOffset;
    Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),//TODO switch to only driver 1
        turnVector = new Vector2d(
            gamepadEx1.getRightX() * Math.abs(gamepadEx1.getRightX()),
            0);
    if (bot.roadRunner.mode == Mode.IDLE) {
      if (centricity)//epic java syntax
        bot.drive.driveFieldCentric(
            driveVector.getX() * driveSpeed,
            driveVector.getY() * driveSpeed,
            turnVector.getX() * driveSpeed,
            gyroAngle);
      else
        bot.drive.driveRobotCentric(
            driveVector.getX() * driveSpeed,
            driveVector.getY() * driveSpeed,
            turnVector.getX() * driveSpeed
        );
    }
    if (justPressed(Button.LEFT_STICK_BUTTON)) {
      fieldCentricOffset = bot.imu.getAngularOrientation()
          .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }
    if(justPressed(Button.RIGHT_STICK_BUTTON)){
      centricity = !centricity;
    }
  }

  private void followPath(){//Path following ===================================================================================

  }


  private void updateLocalization() {
    bot.roadRunner.update();
  }
}
