package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class Outtake {
    private static boolean manual = false;

    private Servo claw, bucket;

    public final OuttakeMotor turret, arm, slides;

    private double clawOpen = 0.35, clawClosed = 0.6;
    private double bucketIntake = 0.6, bucketDown = 0.4;
    private boolean isClawOpen = false, isBucketUp = false;

    public static final int turretIntake = 0, armIntake = 0, slidesIntake = 400,
            slidesClosed = 0, armClosed = 500,
            armOuttake = 888, slidesOuttake = 1000;
    public static final int turretOuttake = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? -600 : 600;

    public Outtake(OpMode opMode){
        turret = new OuttakeMotor(opMode, "turret", Motor.GoBILDA.RPM_312, 1, 0, 0.02, 7, 1000, -1000, 4, 25, 7);//TODO test i coefficient
        arm = new OuttakeMotor(opMode, "arm", Motor.GoBILDA.RPM_312, 1, 0, 0.015, 30, 1200, -20, 3, 350, 7);
        slides = new OuttakeMotor(opMode, "slides", Motor.GoBILDA.RPM_312, 1, 0, 0.02, 20, 1500, -20, 3, 35, 7);

        claw = opMode.hardwareMap.servo.get("claw");
        claw.setDirection(Servo.Direction.FORWARD);

        bucket = opMode.hardwareMap.servo.get("bucket");
        bucket.setDirection(Servo.Direction.FORWARD);
    }

    public Command runToIntake() {
        return new SequentialCommandGroup(
                new InstantCommand(this::bucketDown),
                new InstantCommand(this::closeClaw),
//                new WaitCommand(1000),
                slides.new RunTo(slidesClosed),
                arm.new RunTo(armClosed),
//                new WaitCommand(1000),
                turret.new RunTo(turretIntake),
//                new WaitCommand(1000),
                new InstantCommand(this::bucketUp),
                new WaitCommand(300),
                arm.new RunTo(armIntake),
//                new WaitCommand(1000),
                slides.new RunTo(slidesIntake),
                new InstantCommand(this::openClaw)
        );
    }

    public Command runToOuttake(){
        return new SequentialCommandGroup(
                new InstantCommand(this::closeClaw),
//                new WaitCommand(1000),
                slides.new RunTo(slidesClosed),//TODO maintain position
                new WaitCommand(300),
                arm.new RunTo(armClosed),
                new InstantCommand(this::bucketDown),
                new WaitCommand(300),
                turret.new RunTo(turretOuttake),
                arm.new RunTo(armOuttake),
                slides.new RunTo(slidesOuttake)
        );
    }
    public Command runToPosition(int turretPos, int armPos, int slidesPos){
        return new SequentialCommandGroup(
                new InstantCommand(() -> setManual(false)),
                new ParallelCommandGroup(turret.new RunTo(turretPos), arm.new RunTo(armPos), slides.new RunTo(slidesPos)),
                new InstantCommand(() -> setManual(true)));
    }

    public boolean atTargetPosition(){
        return turret.atTargetPosition() && arm.atTargetPosition() && slides.atTargetPosition();
    }
    
    public void toggleManual(){
        manual = !manual;
        turret.setManual(manual);
        arm.setManual(manual);
        slides.setManual(manual);
    }
    
    public boolean getManual(){
        return manual;
    }

    public void setManual(boolean m){
        if(manual != m){
            toggleManual();
        }
    }

    public void closeClaw(){
        isClawOpen = false;
        claw.setPosition(clawClosed);
    }

    public void openClaw() {
        isClawOpen = true;
        claw.setPosition(clawOpen);
    }

    public void toggleClaw(){
        isClawOpen = !isClawOpen;
        if(isClawOpen){
            openClaw();
        }else{
            closeClaw();
        }
    }

    public void bucketUp(){
        isBucketUp = true;
        bucket.setPosition(bucketIntake);
    }

    public void bucketDown() {
        isBucketUp = false;
        bucket.setPosition(bucketDown);
    }

    public void toggleBucket(){
        isBucketUp = !isBucketUp;
        if(isBucketUp){
            bucketUp();
        }else{
            bucketDown();
        }
    }

    public double toTheta(double y, double x){
        return x > 0 ? Math.atan(y/x) : x < 0 ? -Math.atan(y/x) : y > 0 ? Math.PI/2 : -Math.PI/2;
    }

    public class Pose3d{//RECTANGULAR
        public final double x, y, z;

        public Pose3d(){
            this(0, 0, 0);
        }

        public Pose3d(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public Cyl3d toCyl(){
            return new Cyl3d(Math.sqrt(x*x + y*y), toTheta(y, x), z);//TODO check small values of x
        }

        public Sph3d toSph(){
            return new Sph3d(Math.sqrt(x*x + y*y + z*z), toTheta(y, x), Math.atan(z/Math.sqrt(x*x + y*y)));
        }
    }

    public class Cyl3d{//CYLINDRICAL in degrees
        public final double r, theta, z;

        public Cyl3d(){
            this(0, 0, 0);
        }

        public Cyl3d(double r, double theta, double z) {
            this.r = r;
            this.theta = theta;
            this.z = z;
        }

        public Pose3d toPose(){
            return new Pose3d(r * Math.cos(theta), r * Math.sin(theta), z);
        }

        public Sph3d toSph(){
            return new Sph3d(Math.sqrt(r*r + z*z), theta, Math.atan(z/r));
        }
    }

    public class Sph3d{//SPHERICAL in degrees
        public final double r, theta, phi;

        public Sph3d(){
            this(0, 0, 0);
        }

        public Sph3d(double r, double theta, double phi) {
            this.r = r;
            this.theta = theta;
            this.phi = phi;
        }

        public Pose3d toPose(){
            return new Pose3d(r * Math.cos(phi) * Math.cos(theta), r * Math.cos(phi) * Math.sin(theta), r * Math.sin(phi));
        }

        public Cyl3d toCyl(){
            return new Cyl3d(r * Math.cos(phi), theta, r * Math.sin(phi));
        }
    }
}
