package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "reset scheduler", group = "Experimental")
public class ResetScheduler extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
    }
}
