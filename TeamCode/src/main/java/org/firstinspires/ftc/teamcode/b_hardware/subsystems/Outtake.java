package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake extends SubsystemBase {
    private MotorEx turret, arm, slides;
    private Servo leftClaw, rightClaw, bucket;

    private int turretTarget = 0, armTarget = 0, slidesTarget = 0;

    public static int slideMin = -10,//TODO find values
            slideMax = 100;
    public static double clawClosed = 0,
            clawOpen = 0.25,
            armHeight = 100,//in mm
            slideTicksPerMM = 100,
            turretTicksPerDegree = Motor.GoBILDA.RPM_312.getCPR()*4/360,
            armTicksPerDegree = Motor.GoBILDA.RPM_1150.getCPR()*24/360;//TODO: use this and maxachievableticks()


    public Outtake(OpMode opMode){
        turret = new MotorEx(opMode.hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        turret.setRunMode(Motor.RunMode.PositionControl);
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setPositionTolerance(50);

        arm = new MotorEx(opMode.hardwareMap, "arm", Motor.GoBILDA.RPM_1150);//TODO: check tolerances and directions
        arm.setRunMode(Motor.RunMode.PositionControl);
        arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        arm.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setPositionTolerance(50);

        slides = new MotorEx(opMode.hardwareMap, "slides", Motor.GoBILDA.RPM_312);
        slides.setRunMode(Motor.RunMode.PositionControl);
        slides.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slides.motor.setDirection(DcMotorSimple.Direction.FORWARD);
        slides.setPositionTolerance(50);

        leftClaw = opMode.hardwareMap.servo.get("leftClaw");
        rightClaw = opMode.hardwareMap.servo.get("rightClaw");
        bucket = opMode.hardwareMap.servo.get("bucket");
    }

    public class ResetToIntake extends CommandBase{
        private final Sph3d target = new Sph3d(10, 1, 1);

        public ResetToIntake(){
        }

        @Override
        public void initialize() {
            goToRelative(target);
        }

        @Override
        public boolean isFinished() {
            return atTargetPosition();
        }
    }

    public class RunToPositionRaw extends CommandBase{
        private final Sph3d rawTarget;

        public RunToPositionRaw(Sph3d target){
            rawTarget = target;
        }

        @Override
        public void initialize() {
            goToRaw(rawTarget);
        }

        @Override
        public boolean isFinished() {
            return atTargetPosition();
        }
    }

    public class RunToPosition extends CommandBase{
        private final Sph3d target;

        public RunToPosition(Pose3d target){
            this.target = target.toSph();
        }

        public RunToPosition(Sph3d target){
            this.target = target;
        }

        @Override
        public void initialize() {
            goToRelative(target);
        }

        @Override
        public boolean isFinished() {
            return atTargetPosition();
        }
    }

    public boolean atTargetPosition(){
        return turret.atTargetPosition() && arm.atTargetPosition() && slides.atTargetPosition();
    }

    public void setTurret(int pos){
        turretTarget = pos;
        turret.setTargetPosition(pos);
    }

    public void setArm(int pos){
        armTarget = pos;
        arm.setTargetPosition(pos);
    }

    public void setSlides(int pos){
        pos = Math.min(Math.max(slideMin, pos), slideMax);
        slidesTarget = pos;
        slides.setTargetPosition(pos);
    }

    private void goToRaw(Sph3d sph3d){
        setTurret(turretDegToTicks(sph3d.theta));
        setArm(armDegToTicks(sph3d.phi));
        setSlides((int)sph3d.r);
    }

    private Sph3d calcRel(Sph3d sph3d){
        //TODO calculate relative to arm center
        return sph3d;
    }

    public void goToRelative(Sph3d sph3d){
        goToRaw(calcRel(sph3d));
    }

    public void goToRelative(Pose3d pose3d){
        goToRelative(pose3d.toSph());
    }

    private int turretDegToTicks(double deg){
        return (int)(deg * turretTicksPerDegree);//TODO find ticks
    }

    private int armDegToTicks(double deg){
        return (int)(deg * armTicksPerDegree);
    }

    private int slideToTicks(double lengthInMM){
        return (int)(lengthInMM * slideTicksPerMM);//TODO find this
    }

    public void moveSlides(int d){//testing only
        setSlides(slidesTarget + d);
    }

    public void moveArm(int d){
        setArm(armTarget + d);
    }

    public void moveTurret(int d){
        setTurret(turretTarget + d);
    }

    @Override
    public void periodic() {
        if(turret.atTargetPosition()){
            turret.stopMotor();
        }else{
            turret.set(0.3);
        }

        if(arm.atTargetPosition()) {
            arm.stopMotor();
        }else{
            arm.set(0.3);
//            int x = Math.abs(target - arm.getCurrentPosition());
//            arm.set(Math.pow(Math.abs((x * x - 6000 * x) / 18000000.0), 2));//TODO: figure out "spline" power/motor path
//            Log.d("armPower", Double.toString(Math.pow(Math.abs((x * x - 6000 * x) / 18000000.0), 2)));

        }

        if(slides.atTargetPosition()){
            slides.stopMotor();
        }else{
            slides.set(0.3);
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
