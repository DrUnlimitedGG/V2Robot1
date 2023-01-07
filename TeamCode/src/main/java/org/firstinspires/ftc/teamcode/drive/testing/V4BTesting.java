

package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ServoTesting", group="Testing")
public class V4BTesting extends OpMode
{
    private Servo LeftServo = null;
    private Servo RightServo = null;

    //public static double StartPosition = 0;
    //public static double EndPosition = 1;

    public static double LeftPosition = 0.5;
    public static double RightPosition = 0.5;


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
        RightServo.setPosition(RightPosition);
        LeftServo.setPosition(LeftPosition);

        telemetry.addData("Servo Connection: ", LeftServo.getConnectionInfo());
        telemetry.addData("Servo Position: ", LeftServo.getPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}