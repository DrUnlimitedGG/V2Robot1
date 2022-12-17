package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name="MecanumDrive", group="TeleOp")
public class MecanumDrive extends OpMode
{
    // Drivetrain motors
    // TODO: Hi
    private DcMotorEx LF = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx LB = null;

    public static double powerOffset;

    @Override
    public void init() {
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
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

    }

    /*
     * eqwCode to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        float spin_power = 0.30f; // The spin power

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        powerOffset = 0.6;

        LF.setPower(frontLeftPower * powerOffset);
        LB.setPower(backLeftPower * powerOffset);
        RF.setPower(frontRightPower * powerOffset);
        RB.setPower(backRightPower * powerOffset);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
