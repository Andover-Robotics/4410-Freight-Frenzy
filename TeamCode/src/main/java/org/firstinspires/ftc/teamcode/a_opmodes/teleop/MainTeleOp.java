package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
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
    driveSpeed = 1 - 0.4 * ((buttonSignal(Button.LEFT_BUMPER) ? 1 : 0) + (buttonSignal(Button.RIGHT_STICK_BUTTON) ? 1 : 0));
//
//    if(justPressed(Button.BACK)){
//      isManual = !isManual;
//    }

    if(isManual) {
      drive();
    }else{
      followPath();
    }

    if(stickSignal(Direction.RIGHT).magnitude() > 0.01){
      bot.outtake.moveTurret((int)(stickSignal(Direction.RIGHT).getX() * 20));
      bot.outtake.moveArm((int)(stickSignal(Direction.RIGHT).getY() * 20));
    }

    if(Math.abs(stickSignal(Direction.LEFT).getY()) > 0.01){
      bot.outtake.moveSlides((int)(stickSignal(Direction.LEFT).getY() * 20));
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
  }


  private void drive(){//Driving ===================================================================================
    final double gyroAngle =
        bot.imu.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).secondAngle//TODO: make sure that the orientation + imu axes is correct
            - fieldCentricOffset;
    Vector2d driveVector = stickSignal(Direction.LEFT),
        turnVector = new Vector2d(
            stickSignal(Direction.RIGHT).getX() * Math.abs(stickSignal(Direction.RIGHT).getX()),
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
