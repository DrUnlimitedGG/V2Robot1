package org.firstinspires.ftc.teamcode.drive.showcase;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name="DriveForward", group="Showcase")
public class DriveForward extends OpMode
{
    // Drivetrain motors
    private DcMotorEx LF = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx LB = null;

    public static double powerOffset;
    public static long driveTime;

    @Override
    public void init() {
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");

        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);
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
        powerOffset = 0.6;
        driveTime = 1000;

        LF.setPower(powerOffset);
        LB.setPower(powerOffset);
        RF.setPower(powerOffset);
        RB.setPower(powerOffset);

        try {
            Thread.sleep(driveTime);
        } catch (InterruptedException e) {
            telemetry.addData("lol: ", "lolling");
        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}