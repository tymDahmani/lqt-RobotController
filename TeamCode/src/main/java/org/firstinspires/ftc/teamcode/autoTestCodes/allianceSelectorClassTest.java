package org.firstinspires.ftc.teamcode.autoTestCodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class allianceSelectorClassTest {

    public String[] autoSelector(){
        // Auto Selector
        String alliance = "BLUE_ALLIANCE";
        String spike = "LEFT_SPIKE";
        String side = "SIDE_CLOSE";

//        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.x){
                alliance = "BLUE_ALLIANCE";
            }else if (gamepad1.b){
                alliance = "RED_ALLIANCE";
            }
            telemetry.addData("Select Alliance (Gamepad1 X = Blue, Gamepad1 B = Red)", "");
            telemetry.addData("Current Alliance Selected : ", alliance.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.dpad_left){
                spike = "LEFT_SPIKE";
            }else if (gamepad1.dpad_right){
                spike = "RIGHT_SPIKE";
            }else if (gamepad1.dpad_up){
                spike = "MID_SPIKE";
            }
            telemetry.addData("Select Spike Mark (Gamepad1 D-PAD Left = Left Spike, Gamepad1 D-PAD Up = Center Spike, Gamepad1 D-PAD Right = Right Spike)", "");
            telemetry.addData("Current Spike Mark Selected : ", spike.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.y){
                side = "SIDE_FAR";
            }else if (gamepad1.a){
                side = "SIDE_CLOSE";
            }
            telemetry.addData("Select Side (Gamepad1 Y = Far, Gamepad1 A = Close)", "");
            telemetry.addData("Current Side Selected : ", side.toUpperCase());
            telemetry.addData("", "");

            telemetry.update();
//        }

        return new String[] {alliance, spike, side};

    }
}
