

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

    public static double StartLeft = 0;
    public static double StartRight = 0.2;

    public static double EndLeft = 0.8;
    public static double EndRight = 1;


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
        if (gamepad1.x && !gamepad1.y) {
            LeftServo.setPosition(0);
            RightServo.setPosition(1);
        }

        if (gamepad1.y && !gamepad1.x) {
            LeftServo.setPosition(1);
            RightServo.setPosition(0);
        }

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