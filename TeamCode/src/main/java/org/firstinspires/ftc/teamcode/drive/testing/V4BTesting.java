

package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="V4BTesting", group="Testing")
public class V4BTesting extends OpMode
{
    private Servo LeftServo = null;
    private Servo RightServo = null;

    public static double UpLeftPos = 1;
    public static double UpRightPos = 0.98;

    public static double MiddleLeftPos = 0.72;
    public static double MiddleRightPos = 0.7;

    public static double DownLeftPos = 0.02;
    public static double DownRightPos = 0;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        RightServo = hardwareMap.get(Servo.class, "RightServo");

        /*
        TODO: Fine tune Servo directions
         */
        LeftServo.setDirection(Servo.Direction.FORWARD);
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            LeftServo.setPosition(UpLeftPos);
            RightServo.setPosition(UpRightPos);
        }

        if (gamepad1.y) {
            LeftServo.setPosition(UpLeftPos);
            //RightServo.setPosition(UpRightPos);
        }

        if (gamepad1.x) {
            //LeftServo.setPosition(UpLeftPos);
            RightServo.setPosition(UpRightPos);
        }

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            LeftServo.setPosition(MiddleLeftPos);
            RightServo.setPosition(MiddleRightPos);
        }

        if (gamepad1.dpad_down) {
            LeftServo.setPosition(DownLeftPos);
            RightServo.setPosition(DownRightPos);
        }

       LeftServo.setPosition(UpLeftPos);
       RightServo.setPosition(UpLeftPos);

       telemetry.addData("Left Servo Connection: ", LeftServo.getConnectionInfo());
       telemetry.addData("Left Servo Position: ", LeftServo.getPosition());
        
        telemetry.addData("Right Servo Connection: ", RightServo.getConnectionInfo());
        telemetry.addData("Right Servo Position: ", RightServo.getPosition());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}