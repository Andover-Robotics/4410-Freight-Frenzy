package org.firstinspires.ftc.teamcode.b_hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

public class Carousel extends SubsystemBase {
    private CRServo carouselRed;
    private CRServo carouselBlue;

    public Carousel (OpMode opMode) {
       carouselRed = opMode.hardwareMap.crservo.get("carouselRed");
       carouselBlue = opMode.hardwareMap.crservo.get("carouselBlue");
    }

    public void runCarousel(){
       carouselRed.setPower(1);
       carouselBlue.setPower(1);
    }

    public void stopCarousel(){
       carouselRed.setPower(0);
       carouselBlue.setPower(0);
    }

}
