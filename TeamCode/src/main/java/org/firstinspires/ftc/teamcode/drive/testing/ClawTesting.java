

package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ClawTesting", group="Testing")
public class ClawTesting extends OpMode
{
    private Servo claw = null;
    private Servo wrist = null;

    public static double StartingClawPosition = 0.15;
    public static double StartingWristPosition = 0.1;

    public static double OpenClawPosition = 0;
    public static double ClosedClawPosition = 0.3;

    public static double StopClawPosition = 0.1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.FORWARD);

        //wrist = hardwareMap.get(Servo.class, "wrist");
        //wrist.setDirection(Servo.Direction.REVERSE);

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
        claw.setPosition(StartingClawPosition);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad2.right_bumper && !gamepad2.left_bumper) { // OPEN Claw
            claw.setPosition(OpenClawPosition);

            telemetry.addData("Claw: ", "Opening");

        }

        if (gamepad2.left_bumper && !gamepad2.right_bumper) { // CLOSE Claw
            claw.setPosition(ClosedClawPosition);

            telemetry.addData("Claw: ", "Closing");

        }

        telemetry.addData("Claw: ", claw.getConnectionInfo());
        telemetry.addData("Position: ", "Hello");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        claw.setPosition(StopClawPosition);
    }

}