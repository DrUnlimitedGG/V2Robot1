package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmodes.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmodes.SampleMecanumDrive;

@Config
@TeleOp(name="SlidesDisabledDrive", group="TeleOp")
@Disabled
public class SlidesDisabledDrive extends OpMode
{
    // Drivetrain motors
    private DcMotorEx LF = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx LB = null;

    //private DcMotorEx LeftSlide = null;
    //private DcMotorEx RightSlide = null;

    private TouchSensor clawTouch = null;

    private Servo claw = null;
    private Servo LeftServo = null;
    private Servo RightServo = null;

    public static double powerOffset;
    public static double GoUpSpeed = 0.9;
    public static double GoDownSpeed = 0.55;

    public static int targetPosition = 0;

    boolean initArm = true;
    boolean initSlide = true;

    boolean extending = false;
    boolean retracting = false;

    //ElapsedTime slideTime = new ElapsedTime();

    SampleMecanumDrive drive = null;

    @Override
    public void init() {
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");

        clawTouch = hardwareMap.get(TouchSensor.class, "clawTouch");

        //LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        //RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");

        claw = hardwareMap.get(Servo.class, "claw");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PoseStorage.currentPose);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override

    public void start() {
        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
        LF.setPower(0);

        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);

        //RightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        RightServo.setDirection(Servo.Direction.REVERSE);

        /*LeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);*/

        powerOffset = 0.8;
    }

    /*
     * eqwCode to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        float spin_power = 0.30f; // The spin power

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        if ((gamepad1.right_bumper && gamepad1.y) && !gamepad1.left_bumper) {
            powerOffset = 1;
        } else {
            powerOffset = 0.8;
        }

        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            powerOffset = 0.25;
        } else if ((gamepad1.right_bumper && gamepad1.y) && !gamepad1.left_bumper) {
            powerOffset = 1;
        } else {
            powerOffset = 0.8;
        }

        LF.setPower(frontLeftPower * -powerOffset);
        LB.setPower(backLeftPower * -powerOffset);
        RF.setPower(frontRightPower * -powerOffset);
        RB.setPower(backRightPower * -powerOffset);



       /* telemetry.addData("Target Position: ", targetPosition);
        // SLIDE CODE
        if (gamepad2.y) {
           /* if ((targetPosition + 20) > 1120) {
                targetPosition = 1120;
            } else {
                targetPosition = targetPosition + 20;
            }*/

         /*   targetPosition = targetPosition + 20;


            if (initSlide == true) {
                LeftServo.setPosition(1);
                RightServo.setPosition(1);

                initArm = false;
                initSlide = false;
            }

            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

            LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoUpSpeed);
            RightSlide.setPower(GoUpSpeed);

        }

        if (gamepad2.x) {
            if ((targetPosition - 20) < 0) {
                targetPosition = 0;
            } else {
                targetPosition = targetPosition - 20;
            }

            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

            LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoDownSpeed);
            RightSlide.setPower(GoDownSpeed);

        }*/

        // V4B CODE
        if (!gamepad2.left_bumper && gamepad2.right_bumper) {
            if (targetPosition > 500) {
                claw.setPosition(0.8);

                LeftServo.setPosition(1);
                RightServo.setPosition(1);
            } else if (initArm == true) {
                claw.setPosition(0.8);

                LeftServo.setPosition(1);
                RightServo.setPosition(1);


                initArm = false;
                initSlide = false;
            } else {
                telemetry.addData("Error: ", "Raise slides to move the V4B!");
            }
        }

        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            claw.setPosition(0.8);

            LeftServo.setPosition(0.8);
            RightServo.setPosition(0.8);
        }

        if (!gamepad2.right_bumper && gamepad2.left_bumper) {
            if (targetPosition > 500) {
                claw.setPosition(0.8);

                LeftServo.setPosition(0);
                RightServo.setPosition(0);
            } else {
                telemetry.addData("Error: ", "Raise slides to move the V4B!");
            }
        }



        // CLAW CODE
        if ((gamepad2.left_trigger > 0.4) && (gamepad2.right_trigger < 0.4)) { // OPEN Claw
            claw.setPosition(0);

            // telemetry.addData("Claw: ", "Closing");

        }

        if ((gamepad2.right_trigger) > 0.4 && (gamepad2.left_trigger < 0.4)) { // CLOSE Claw
            claw.setPosition(0.8);

            // telemetry.addData("Claw: ", "Opening");

        }

        if (clawTouch.isPressed()) {
            claw.setPosition(0.8);
        }



        // MACROS
        /*if (gamepad1.y) { // 180 degree turn (clockwise)
            drive.turn(Math.toRadians(180) - 1e-6);
        }*/

        /*if (gamepad2.dpad_up) { // Slide at high goal
            targetPosition = 940;
            extending = true;



            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

            LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoUpSpeed);
            RightSlide.setPower(GoUpSpeed);

        }

        if (gamepad2.dpad_right) { // Slide at high goal
            targetPosition = 940;
            extending = true;

            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

            LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoUpSpeed);
            RightSlide.setPower(GoUpSpeed);


        }

        if (gamepad2.dpad_left) { // Slide retraction
            targetPosition = 0;
            retracting = true;

            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

            LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoDownSpeed);
            RightSlide.setPower(GoDownSpeed);

        }*/

        /*if (extending == true) {
            if ((LeftSlide.getCurrentPosition() > 500) && (RightSlide.getCurrentPosition() > 500)) {
                claw.setPosition(0.8);
                LeftServo.setPosition(0);
                RightServo.setPosition(0);

                extending = false;
            }
        }

        if (retracting == true) {
            if ((LeftSlide.getCurrentPosition() > 500) && (RightSlide.getCurrentPosition() > 500)) {
                claw.setPosition(0.8);

                LeftServo.setPosition(1);
                RightServo.setPosition(1);

                retracting = false;
            }
        }*/


        // Telemetry
        drive.update();
        Pose2d robotPose = drive.getPoseEstimate();
        //double heading = robotPose.getHeading();

        telemetry.addData("X: ", robotPose.getX());
        telemetry.addData("Y: ", robotPose.getY());
        telemetry.addData("Heading: ", Math.toDegrees(robotPose.getHeading()));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
