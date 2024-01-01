package org.firstinspires.ftc.teamcode.autoTestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


    // NOTE: the driveEncoder method can be used instead of the move method to move the robot.
// the driveEncoder method takes the inputs: motor speed, left motor meters (for both front and back), right meters, timeout. and calculates the ticks needed and
// applies the to the motors and gives the speed in the parameters and makes the motors move.


public class auto1test extends LinearOpMode {
        // robot actuators
    // drive train
    DcMotorEx leftF;
    DcMotorEx leftB;
    DcMotorEx rightF;
    DcMotorEx rightB;

    // slides
    DcMotor slideL;
    DcMotor slideR;

    // arm base core
    DcMotor armBase;

    // gripper arm core
    DcMotor gripperArm;

    // gripper tilting servo
    Servo tilting;

    // grippers
    Servo gripperL;
    Servo gripperR;

    //elapsed time
    ElapsedTime runtime = new ElapsedTime();




        // vars
    // servo position
    int servoPos = 0;


    // pixel pos (left, mid, or right - 1, 2 ,3)
    int tpPos = 2;

    // vars for the driveEncoder method - which calculates the ticks needed for the given distance, time and which motors
    static final double tpr = 3000; // ticks per revolution
    
    static final double MOTOR_GEAR_REDUCTION = 1.0; // sth about the motor gears and ratios (This is < 1.0 if geared UP)
    static final double WHEEL_DIAMETER_CM = 7.0; // For figuring circumference
    static final double cpMeter = (tpr * MOTOR_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415); // cmp = counts per meter
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;






    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeInInit()) {
            // hardware map
            slideL = hardwareMap.get(DcMotor.class, "slide1");
            slideR = hardwareMap.get(DcMotor.class,"slide2");
            armBase = hardwareMap.get(DcMotor.class,"armBase");
            gripperArm = hardwareMap.get(DcMotor.class,"gripper arm");
            tilting = hardwareMap.get(Servo.class,"tilting servo");
            gripperL = hardwareMap.get(Servo.class,"gripper1");
            gripperR = hardwareMap.get(Servo.class,"gripper2");

            // reversed motors
            rightB.setDirection(DcMotorSimple.Direction.REVERSE);
            rightF.setDirection(DcMotorSimple.Direction.REVERSE);
            gripperL.setDirection(Servo.Direction.REVERSE);

            // stop and reset encoder
            leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // run using encoder
            leftF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            gripperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
        waitForStart();

        while (opModeIsActive()) {
            // turn on the camera. + init the eocv pipeline class or whatever so it would be ready to work - better use a function for this or a whole class

            // move forward to read the tp
            move(0, 0);

            // read the team prop (either w eocv or ml)
                // if the camera read nothing put it in the mid (default pos), which means the default pos num will be 2
            // make sure to store the class's returnings for the tp pos in this var (1=left, 2=mid, 3=right):
            tpPos = 2;

            // turn to the tp pos detected by the camera
            move(0, 0);

            // put the purple pixel depending on the index given by the reads of the camera
            putPurplePixel(servoPos);

            // go to the back drop
            move(0, 0);

            // move the gripper arm and use the gripper to drop the pixel
            backDrop(0, 0, 0);

            // park
            move(0, 0);

        }
    }

    void move(int dis, int vel) {

        leftF.setTargetPosition(dis);
        leftB.setTargetPosition(dis);
        rightF.setTargetPosition(dis);
        rightB.setTargetPosition(dis);

        leftF.setVelocity(vel);
        leftB.setVelocity(vel);
        rightF.setVelocity(vel);
        rightB.setVelocity(vel);

        while (leftF.isBusy()) {
            idle();
        }

        // let the robot calm down
        sleep(100);

    }


    void putPurplePixel (int pos) {

        // open
        gripperR.setPosition(pos);

        // close
        gripperR.setPosition(-pos);


        sleep(100);

    }

    void backDrop (int dis, int pwr, int pos) {

        // arm tar pos to be on the back drop
        gripperArm.setTargetPosition(dis);

        // move arm
        gripperArm.setPower(pwr);

        while (gripperArm.isBusy()) {
            idle();
        }

        // yellow pixel servo move
        gripperL.setPosition(pos);

        // close
        gripperL.setPosition(-pos);


        sleep(100);

    }

    public void encoderDrive(double speed, double leftMeters, double rightMeters, double timeoutS) {

        int leftFTar;
        int leftBTar;
        int rightFTar;
        int rightBTar;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftFTar = leftF.getCurrentPosition() + (int) (leftMeters * cpMeter);
            leftBTar = leftB.getCurrentPosition() + (int) (leftMeters * cpMeter);
            rightFTar = leftF.getCurrentPosition() + (int) (rightMeters * cpMeter);
            rightBTar = leftB.getCurrentPosition() + (int) (rightMeters * cpMeter);
            leftF.setTargetPosition(leftFTar);
            leftB.setTargetPosition(leftBTar);
            rightF.setTargetPosition(rightFTar);
            rightB.setTargetPosition(rightBTar);

            // Turn On RUN_TO_POSITION
            leftF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftF.setPower(Math.abs(speed));
            leftB.setPower(Math.abs(speed));
            rightF.setPower(Math.abs(speed));
            rightB.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop. This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftF.isBusy() && leftB.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Ticks needed to be ticked", "Running to %7d :%7d", leftFTar, rightFTar);
                telemetry.addData("ticks ticked", "Running at %7d :%7d", leftF.getCurrentPosition(), rightF.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
