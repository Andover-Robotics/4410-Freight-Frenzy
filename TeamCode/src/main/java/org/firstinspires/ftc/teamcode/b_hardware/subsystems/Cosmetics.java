package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class Cosmetics {//Literally a bunch of servos with stuff on them
  private Servo flagArm;
  private CRServo duck;

  public static double flagUp = 0.65;
  public static double flagDown = 0;

  public Cosmetics(OpMode opMode){
    flagArm = opMode.hardwareMap.servo.get("flagArm");
    duck = opMode.hardwareMap.crservo.get("duck");

  }

  public void runDuck(){
    duck.setPower(1);
  }

  public void stopDuck(){
    duck.setPower(0);
  }

  public void raiseFlag(){
    flagArm.setPosition(flagUp);
  }

  public void lowerFlag(){
    flagArm.setPosition(flagDown);
  }
}
