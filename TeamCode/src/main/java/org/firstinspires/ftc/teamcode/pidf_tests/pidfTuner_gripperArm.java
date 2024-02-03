package org.firstinspires.ftc.teamcode.pidf_tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class pidfTuner_gripperArm extends OpMode {

    PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    final double ticks_per_degree = 0 / 180.0; // check docs and fill this value

    DcMotor gripperArm;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gripperArm = hardwareMap.get(DcMotor.class, "armBase");

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = gripperArm.getCurrentPosition();

        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * f;

        double power = pid + ff;

        gripperArm.setPower(power);

        telemetry.addData("position of the arm motor ", armPos);
        telemetry.addData("target position ", target);
        telemetry.update();

    }
}
