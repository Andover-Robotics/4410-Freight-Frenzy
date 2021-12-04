package org.firstinspires.ftc.teamcode.a_opmodes.auto;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoPaths;

@Autonomous(name = "Configure Auto", group = "Competition")
public class ConfigureAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int typeSelect = 0;
        AutoPaths.AutoType[] types = AutoPaths.AutoType.values();
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        while(!isStarted()){
            telemetry.addLine("y(top) for alliance, a(bottom) for type");
            telemetry.addLine("alliance " + GlobalConfig.alliance);
            telemetry.addLine("types " + types[typeSelect]);
            telemetry.update();
            gamepadEx.readButtons();
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y))
                GlobalConfig.alliance = GlobalConfig.alliance == GlobalConfig.Alliance.RED ? GlobalConfig.Alliance.BLUE : GlobalConfig.Alliance.RED;
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A))
                typeSelect = (typeSelect + 1) % types.length;

        }
        GlobalConfig.autoType = types[typeSelect];
    }
}
