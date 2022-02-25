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
    bot.init();
  }

  @Override
  public void subLoop() {
    //update stuff=================================================================================================
    cycle = 1.0/(time-prevRead);
    prevRead = time;
    timingScheduler.run();

    //Movement =================================================================================================
    //TODO: change depending on mode
    driveSpeed = 1 - 0.35 * (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
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


    // First man ----------
    if(gamepad1.a){
      bot.intake.runIntake();
    }else if(gamepad1.b){
      bot.intake.spitIntake();
    }else if(gamepad1.dpad_right){
      bot.intake.runSlow();
    }else{
      if(!CommandScheduler.getInstance().isScheduled(bot.intake.transferToOuttake())) {
        bot.intake.stopIntake();
      }
    }

    if(gamepad1.y || gamepad1.dpad_left){
      bot.carousel.runCarousel();
    }else{
      bot.carousel.stopCarousel();
    }

    if(gamepadEx1.wasJustPressed(Button.X)){
      CommandScheduler.getInstance().schedule(bot.intake.transferToOuttake());
    }

    // Second man ----------
    bot.outtake.slides.driveManual(gamepadEx2.getLeftY());
    bot.outtake.turret.driveManual(-gamepadEx2.getRightX());

    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.B)){
      CommandScheduler.getInstance().schedule(bot.outtake.runToIntake());
    }
    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.A)){
      CommandScheduler.getInstance().schedule(bot.outtake.runToOuttake());
    }

    if(gamepadEx2.wasJustPressed(GamepadKeys.Button.X)){
      bot.outtake.toggleBucket();
    }
    if(gamepadEx2.wasJustPressed(Button.Y)){
      bot.outtake.toggleClamp();
    }

    if(gamepadEx1.wasJustPressed(Button.DPAD_UP)){
      bot.intake.flipShared();
    }else if(gamepadEx1.wasJustPressed(Button.DPAD_DOWN)){
      bot.intake.flipDown();
    }




    /*//TODO: make control scheme
    Controller 1
    X:
    O:
    Square: switch robot field centric or none
    Triangle: Carousels
    DPAD: Field centric movement
    Joystick
    L: Field centric movement
    R: Set orientation / Rotation (Determine through practice)
    L Trigger: Eject (Reverse Intake)
    R Trigger: Take in (Forward Intake)
    L Bumper: 66% movement speed
    R Bumper: 66% movement speed
    NOTE - Both Bumpers: 33% movement speed (really only for mega precise times)
    Other
    Start:  reset field centric or progress automation 
    Back: switch between automation and driving

    Controller 2
    X: Flip outtake bucket (Hold)
    O:
    Square: Flip Intake upward (Hold)
    Triangle:
    DPAD
    L:      D: Detract slides    U: Extend slides     R:
    Joystick
    L: Turret (Set Direction) (Deadzone = rotate with robot base)
    R: Arm (Direction = Driver's preferance)
    Trigger L/R: slow driving
    Bumper
    L: none/switch to previous path      R:none/switch to next path
    Other
    Start: reset field centric or progress automation 
    Back: switch between automation and driving
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
    telemetry.addData("slides", bot.outtake.slides.getTarget());
    telemetry.addData("slides current", bot.outtake.slides.getPosition());
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
