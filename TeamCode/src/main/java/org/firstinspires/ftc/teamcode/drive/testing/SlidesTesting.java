

package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group="Testing")
public class SlidesTesting extends OpMode
{
    private DcMotorEx LeftSlide = null;
    private DcMotorEx RightSlide = null;

    private Servo LeftServo = null;
    private Servo RightServo = null;

    public static double GoUpSpeed = 0.9;
    public static double GoDownSpeed = 0.2;
    public static int targetPosition = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LeftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        RightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");

        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        RightSlide.setDirection(DcMotorEx.Direction.REVERSE);
        RightServo.setDirection(Servo.Direction.REVERSE);

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
        LeftSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LeftSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RightSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        LeftSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad2.x) {
            targetPosition = targetPosition + 5;

            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

           LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoUpSpeed);
            RightSlide.setPower(GoUpSpeed);

        }

        if (gamepad2.y) {
            if ((targetPosition - 10) < 0) {
                targetPosition = 0;
            } else {
                targetPosition = targetPosition - 10;
            }

            LeftSlide.setTargetPosition(targetPosition);
            RightSlide.setTargetPosition(targetPosition);

            LeftSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftSlide.setPower(GoDownSpeed);
            RightSlide.setPower(GoDownSpeed);

        }

        if (gamepad1.right_bumper) {
            RightSlide.setPower(GoUpSpeed);
        }

        if (gamepad1.left_bumper) {
            RightSlide.setPower(GoDownSpeed);
        }

        if (gamepad2.right_bumper) {
           RightServo.setPosition(1);
           LeftServo.setPosition(1);
        }

        if (gamepad2.left_bumper) {
           RightServo.setPosition(0);
           LeftServo.setPosition(0);
        }

        telemetry.addData("Target Position: ", targetPosition);
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}