/*package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



public class SlidesTesting extends LinearOpMode {

    private DcMotorEx linearSlide = null;
    private DcMotorEx linearSlide2 = null;

    double linearPower = 0;
    boolean brakeOn = false;


    @Override
    public void runOpMode() {

        if(gamepad2.left_trigger==1&&gamepad2.right_trigger==0) {

            linearPower = 1;
        }
        else if(gamepad2.left_trigger==0&&gamepad2.right_trigger==1){
            linearPower = -1;

        }
        else{
            linearPower = 0;
            double downPower = 0;


            public void setBrake() {
                if(!brakeOn) {
                    linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    brakeOn = true;
                }else{
                    linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    brakeOn = false;
                }

            }

        }


 */