package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths;
import org.firstinspires.ftc.teamcode.c_drive.DriveConstants;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.Encoder.Direction;
@Config
public class GlobalConfig {//make all fields final
  //control + click class to go to it
  DriveConstants controlClickForDriveConstants;
  public enum Alliance{
    RED,
    BLUE;

    @NonNull
    @Override
    public String toString() {
      return super.toString();
    }
  }

  //TODO add more config stuff
  public static final String motorFL = "motorFL", motorFR = "motorFR", motorBL = "motorBL", motorBR = "motorBR";
  public static Alliance alliance = Alliance.RED;
  public static AutoPaths.AutoType autoType = AutoPaths.AutoType.CYCLE;

  public static PIDController TURRET_PID = new PIDController(0.006, 0.05, 0.0004);
  public static PIDController SLIDES_PID = new PIDController(0.03, 0.005, 0.0004);


  public static class SensorFusionValues{
    public static final double[] sensorFusionHeadingWeights = {0.225, 0.225, 0.225, 0.225, 0},
        sensorFusionPositionWeights = {0.15, 0.15, 0.15, 0.15, 0.4};
    public static boolean averagePose = false;
  }

  public static class EncoderValues{
    public static final Pose2d sideEncoder = new Pose2d(0.1965, 5.5, 0), centerEncoder = new Pose2d(-5.2, 0.75, Math.toRadians(90));
    public static final String leftEncoderPort = "motorFL", rightEncoderPort = "motorBL", centerEncoderPort = "motorFR";
    public static final Direction leftEncoderDirection = Direction.FORWARD, rightEncoderDirection = Direction.REVERSE, centerEncoderDirection = Direction.REVERSE;
  }
}
