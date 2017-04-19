package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="FortonamousOmniTester")
public class FortonamousOmniTester extends LinearOpMode {

    DcMotor fr; // Front Right Motor // Runs in Y Direction //
    DcMotor fl; // Front Left Motor  // Runs in X Direction //
    DcMotor br; // Back Right Motor  // Runs in X Direction //
    DcMotor bl; // Back Left Motor   // Runs in Y Direction //
    DcMotor leftShot;
    DcMotor rightShot;
    DcMotor channel;
    DcMotor lift;
    OpticalDistanceSensor odsIn;
    OpticalDistanceSensor odsOut;
    ColorSensor color;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor range;

    void move(double angle, double power) throws InterruptedException {
        if (!opModeIsActive()) {
            stopMoving();
            return;
        }
        straighten();
        fr.setPower(-power * Math.sin((Math.PI / 180) * angle));
        bl.setPower(power * Math.sin((Math.PI / 180) * angle));
        fl.setPower(power * Math.cos((Math.PI / 180) * angle));
        br.setPower(-power * Math.cos((Math.PI / 180) * angle));
    }

    void moveWithoutStraighten(double angle, double power) {
        if (!opModeIsActive()) {
            stopMoving();
            return;
        }
        fr.setPower(-power * Math.sin((Math.PI / 180) * angle));
        bl.setPower(power * Math.sin((Math.PI / 180) * angle));
        fl.setPower(power * Math.cos((Math.PI / 180) * angle));
        br.setPower(-power * Math.cos((Math.PI / 180) * angle));
    }

    void stopMoving() {
        fr.setPower(0);
        fl.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    void turnToAngleZ(double angle, double power) {
        if (!opModeIsActive()) {
            stopMoving();
            return;
        }
        if (angle != 0) {
            angle = angle / Math.abs(angle);
        }
        fr.setPower(power * angle);
        br.setPower(power * angle);
        fl.setPower(power * angle);
        bl.setPower(power * angle);
    }

    void turnToAngle(double angle, double power) throws InterruptedException {
        if (!opModeIsActive()) {
            stopMoving();
            return;
        }
        double mod = Math.abs(angle) / angle;
        fr.setPower(power * mod);
        br.setPower(power * mod);
        fl.setPower(power * mod);
        bl.setPower(power * mod);
        while (Math.abs(Math.abs(gyro.getIntegratedZValue()) - Math.abs(angle)) >= 5) {
            telemetry.addData(">Goal: ", angle);
            telemetry.addData(">At: ", gyro.getIntegratedZValue());
            telemetry.update();
            sleep(5);
            if (!opModeIsActive()) {
                stopMoving();
                return;
            }
        }
        stopMoving();
    }

    void straighten() throws InterruptedException {
        if (!opModeIsActive()) {
            stopMoving();
            return;
        }
        int angleZ = 0;
        do {
            turnToAngleZ(angleZ, 0.2);
            sleep(25);
            stopMoving();
            sleep(5);

            angleZ = gyro.getIntegratedZValue();
        } while (Math.abs(angleZ) >= 2);
        stopMoving();
    }

    boolean isRed() {
        if (color.red() > color.blue()) {
            return true;
        }
        return false;
    }
    void pressBeacon() throws InterruptedException {
        if(!opModeIsActive()) {stopMoving(); return;}
        if (isRed()) {
            move(270, 0.4);
            sleep(300);
            stopMoving();
            sleep(50);
            move(180, 1);
            sleep(900);
            stopMoving();
            sleep(50);
            move(0, 0.75);
            sleep(300);
            stopMoving();
            move(90, 0.6);
            sleep(200);
            stopMoving();
            sleep(100);
            if(!isRed()) {
                move(270, 0.3);
                while (odsIn.getRawLightDetected() < 1.5) {
                    sleep(5);
                }
                stopMoving();
            }
            while (!isRed()) {
                if (!isRed()) {sleep(4700);}
                move(180, 1);
                sleep(700);
                stopMoving();
                sleep(50);
                move(0, 0.75);
                sleep(300);
                stopMoving();
                move(90, 0.6);
                sleep(200);
                stopMoving();
                sleep(100);
                move(270, 0.3);
                while(odsIn.getRawLightDetected() < 1.5) {
                    sleep(5);
                }
                stopMoving();
            }
        }
        else {
            sleep(100);
            move(90, 0.3);
            while (odsOut.getRawLightDetected() < 1.5) {
                sleep(5);
            }
            stopMoving();
            move(90, 0.4);
            sleep(100);
            stopMoving();
            sleep(50);
            move(180, 1);
            sleep(900);
            stopMoving();
            sleep(50);
            move(0, 0.75);
            sleep(300);
            stopMoving();
            move(90, 0.6);
            sleep(200);
            stopMoving();
            sleep(100);
            if (!isRed()) {
                move(270, 0.3);
                while (odsIn.getRawLightDetected() < 1.5) {
                    sleep(5);
                }
                stopMoving();
            }
            while (!isRed()) {
                if (!isRed()) {sleep(4700);}
                move(180, 1);
                sleep(700);
                stopMoving();
                sleep(50);
                move(0, 0.75);
                sleep(300);
                stopMoving();
                move(90, 0.6);
                sleep(200);
                stopMoving();
                sleep(100);
                move(270, 0.3);
                while(odsIn.getRawLightDetected() < 1.5) {
                    sleep(5);
                }
                stopMoving();
            }
        }
        stopMoving();
    }
    void shoot() throws InterruptedException {
        if (!opModeIsActive()) {
            stopMoving();
            return;
        }
        leftShot.setPower(0.3);
        rightShot.setPower(0.3);
        sleep(300);
        channel.setPower(1);
        sleep(3700);
        channel.setPower(0);
        leftShot.setPower(0);
        rightShot.setPower(0);
    }

    // Sense if the white line is below
    boolean AboveWhiteLine() {
        double lightThreshold = 1.5; // Swag at the threshold for the light reflected off white line

        // Send the info back to driver station using telemetry function.
        telemetry.addData("Raw odsIn", odsIn.getRawLightDetected());
        telemetry.addData("Normal odsIn", odsIn.getLightDetected());
        telemetry.addData("Raw odsIn", odsIn.getRawLightDetected());
        telemetry.addData("Normal odsIn", odsIn.getLightDetected());
        telemetry.update();

        return (odsIn.getRawLightDetected() >= lightThreshold);
    }

    // Move forward at speed until we sense the white line
    void MoveToWhiteLine(double speed) throws InterruptedException {
        while (!AboveWhiteLine()) {
            sleep(2);
        } // Until we sense the white line
        stopMoving(); // Stop moving after sensing the line
    }
    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("bl");
        bl = hardwareMap.dcMotor.get("fl");

        rightShot = hardwareMap.dcMotor.get("rshot");
        leftShot = hardwareMap.dcMotor.get("lshot");
        channel = hardwareMap.dcMotor.get("channel");
        lift = hardwareMap.dcMotor.get("lift");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        odsIn = hardwareMap.opticalDistanceSensor.get("odsOut");
        odsOut = hardwareMap.opticalDistanceSensor.get("odsIn");
        color = hardwareMap.colorSensor.get("color");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        leftShot.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        color.enableLed(false);

        waitForStart();

        gyro.resetZAxisIntegrator();

        stopMoving();
        while(true){
            telemetry.addData("Gyro position: ", gyro.getIntegratedZValue());
            telemetry.addData("Raw odsIn: ", odsIn.getRawLightDetected());
            telemetry.addData("Normal odsIn: ", odsIn.getLightDetected());
            telemetry.addData("Raw odsOut: ", odsOut.getRawLightDetected());
            telemetry.addData("Normal odsOut: ", odsOut.getLightDetected());
            telemetry.addData("Color Red: ", color.red());
            telemetry.addData("Color Blue: ", color.blue());
            telemetry.addData("CM Distance: ", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}