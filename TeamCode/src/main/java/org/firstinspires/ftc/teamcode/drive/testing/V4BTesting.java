

package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="V4BTesting", group="Testing")
public class V4BTesting extends OpMode
{
    private Servo LeftServo = null; // Belt motor
    private Servo RightServo = null;

    public static double LeftPosition = 0;
    public static double RightPosition = 1;

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

        LeftServo.setPosition(1);
        RightServo.setPosition(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad2.dpad_left) {
            LeftServo.setPosition(LeftPosition);
            RightServo.setPosition(LeftPosition);
        }

        if (gamepad2.dpad_right) {
            LeftServo.setPosition(RightPosition);
            RightServo.setPosition(RightPosition);

        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}