package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.c_drive.DriveConstants;
import org.firstinspires.ftc.teamcode.d_util.utilclasses.Encoder.Direction;

@Config
public final class GlobalConfig {//make all fields final
    //control + click class to go to it
    DriveConstants controlClickForDriveConstants;

    public enum Alliance {
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

    public static  class SubsystemValues {
        public static double intakeSpeed = 0.45;
        public static int carouselSpeed = 4410;

        public static class CosmeticsValues {
            public static double flagUp = 0.65;
            public static double flagDown = 0;
        }

        public static final class OuttakeValues {
            public static int armHigh = -1366;
            public static int armMid = -752;
            public static int armLow = -323;
            public static int armDown = -2763;
            public static double bucketUp = 0.35;
            public static double bucketMid = 0.6;
            public static double bucketDown = 1.0;
        }
    }


    public static final class SensorFusionValues {
        public static final double[] sensorFusionHeadingWeights = {0.225, 0.225, 0.225, 0.225, 0},
                sensorFusionPositionWeights = {0.15, 0.15, 0.15, 0.15, 0.4};
        public static final boolean averagePose = true;
    }

    public static final class EncoderValues {
        public static final Pose2d sideEncoder = new Pose2d(-1, 4.25, 0), centerEncoder = new Pose2d(-7, -1, Math.toRadians(90));
        public static final String leftEncoderPort = "motorFL", rightEncoderPort = "motorBL", centerEncoderPort = "motorFR";
        public static final Direction leftEncoderDirection = Direction.FORWARD, rightEncoderDirection = Direction.FORWARD, centerEncoderDirection = Direction.REVERSE;
    }
}
