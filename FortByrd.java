package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="FortByrd")
@Disabled
public class FortByrd extends OpMode{

    DcMotor fr; // Front Right Motor // Runs in Y Direction //
    DcMotor fl; // Front Left Motor  // Runs in X Direction //
    DcMotor br; // Back Right Motor  // Runs in X Direction //
    DcMotor bl; // Back Left Motor   // Runs in Y Direction //
    DcMotor rightShot; // Right Shooting Wheel //
    DcMotor leftShot;  // Left Shooting Wheel  //
    DcMotor channel;   // Channel + Broom      //
    DcMotor lift;      // Exercise Ball Lift   //

    ModernRoboticsI2cGyro gyro; // Gyroscope Sensor //

    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("fr");
        fl = hardwareMap.dcMotor.get("fl");
        br = hardwareMap.dcMotor.get("br");
        bl = hardwareMap.dcMotor.get("bl");

        rightShot = hardwareMap.dcMotor.get("rshot");
        leftShot = hardwareMap.dcMotor.get("lshot");
        channel = hardwareMap.dcMotor.get("channel");
        lift = hardwareMap.dcMotor.get("lift");

        leftShot.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = gamepad1.left_stick_y;
        float r = gamepad1.right_stick_x;

        fl.setPower(x+r);  // Set wheels equal to left stick //
        fr.setPower(y+r);  // direction plus amount of turn  //
        br.setPower(r-x);
        bl.setPower(r-y);

        if(gamepad2.right_trigger > .1) {
            channel.setPower(1); // If right trigger is pressed, //
        }else if (gamepad2.left_trigger > .1) {
            channel.setPower(-1);  // run both wheels of shooter.  //
        }
        else {channel.setPower(0);}

        if(gamepad2.right_bumper) {rightShot.setPower(.27); leftShot.setPower(.27);}      // Right and left bumpers control ////      direction of channel      //*/
        else{leftShot.setPower(0); rightShot.setPower(0);}

        if(gamepad1.dpad_up) {lift.setPower(1);}                // Dpad up and down //
        else if(gamepad1.dpad_down) {lift.setPower(-1);}        //   control lift   //
        else {lift.setPower(0);}
    }
}