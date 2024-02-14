package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autoSelectorTest extends LinearOpMode {

    allianceSelectorClassTest selector = new allianceSelectorClassTest();

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()) {
            selector.autoSelector();
        }
    }
}
