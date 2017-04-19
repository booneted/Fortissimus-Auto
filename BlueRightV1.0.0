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

@Autonomous(name="FortonamousOmniV2.7.0_BlueRight")
public class FortonamousOmniV10BlueRight extends LinearOpMode {

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
    double mod;
    int time;

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
        if(angle<0){mod=-1;}
        if(angle>0){mod=1;}
        angle = gyro.getIntegratedZValue()-angle;
        fr.setPower(power * mod);
        br.setPower(power * mod);
        fl.setPower(power * mod);
        bl.setPower(power * mod);
        while (mod < 0 && gyro.getIntegratedZValue() <= angle) {
            sleep(5);
            telemetry.addData(">Mod <0:", mod);
            telemetry.addData(">Current angle:", gyro.getIntegratedZValue());
            telemetry.addData(">Angle:", angle);
            telemetry.update();
            if (!opModeIsActive()) {
                stopMoving();
                return;
            }
        }
        while (mod > 0 && gyro.getIntegratedZValue() >= angle) {
            sleep(5);
            telemetry.addData(">Mod>0:", mod);
            telemetry.addData(">Current angle:", gyro.getIntegratedZValue());
            telemetry.addData(">Angle:", angle);
            telemetry.update();
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
        if (color.red() < color.blue()) {
            return true;
        }
        return false;
    }
    void pressBeacon() throws InterruptedException {
        if(!opModeIsActive()) {stopMoving(); return;}
        move(180,0.4);
        time = 0;
        sleep(200);
        /*while(range.getDistance(DistanceUnit.CM)>10 && time < 1000){sleep(5); time+=5;} {
            sleep(5);
            telemetry.addData(">Raw:", odsIn.getRawLightDetected());
            telemetry.update();}*/
        stopMoving();
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
                while (odsOut.getRawLightDetected() < 1.5) {
                    sleep(5);
                }
                stopMoving();
                move(0,0.4);
                while(range.getDistance(DistanceUnit.CM)<9) {
                    sleep(5);
                    telemetry.addData(">Raw:", odsIn.getRawLightDetected());
                    telemetry.update();}
                stopMoving();
                move(180,0.4);
                time = 0;
                while(range.getDistance(DistanceUnit.CM)>10 && time < 200){sleep(5); time+=5;} {
                    sleep(5);
                    telemetry.addData(">Raw:", odsIn.getRawLightDetected());
                    telemetry.update();}
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
            sleep(200);
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
                move(0,0.4);
                while(range.getDistance(DistanceUnit.CM)<9) {
                    sleep(5);
                    telemetry.addData(">Raw:", odsIn.getRawLightDetected());
                    telemetry.update();}
                stopMoving();
                move(180,0.4);
                time = 0;
                while(range.getDistance(DistanceUnit.CM)>10 && time < 1000){sleep(5); time+=5;} {
                    sleep(5);
                    telemetry.addData(">Raw:", odsIn.getRawLightDetected());
                    telemetry.update();}
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
        sleep(500);
        channel.setPower(1);
        sleep(3700);
        channel.setPower(0);
        leftShot.setPower(0);
        rightShot.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        fr = hardwareMap.dcMotor.get("fl");
        fl = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("bl");
        bl = hardwareMap.dcMotor.get("br");

        rightShot = hardwareMap.dcMotor.get("rshot");
        leftShot = hardwareMap.dcMotor.get("lshot");
        channel = hardwareMap.dcMotor.get("channel");
        lift = hardwareMap.dcMotor.get("lift");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        odsOut = hardwareMap.opticalDistanceSensor.get("odsOut");
        odsIn = hardwareMap.opticalDistanceSensor.get("odsIn");
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

        move(90, 0.5);
        sleep(1000);
        stopMoving();
        sleep(100);
        turnToAngle(-80,.2);
        shoot();
        stopMoving();
        sleep(100);
        turnToAngle(80,.2);
        move(180, 0.5);
        sleep(1000);
        stopMoving();
        sleep(100);
        move(90, 0.5);
        sleep(900);
        stopMoving();
        sleep(100);
        move(180, 0.5);
        sleep(1300);
        stopMoving();
        sleep(100);
        move(100, 0.3);
        while (odsOut.getRawLightDetected() < 1.5) {
            telemetry.addData(">Raw:", odsIn.getRawLightDetected());
            telemetry.update();
            sleep(5);
        }
        pressBeacon();
        sleep(100);
        move(0, 0.3);
        sleep(200);
        move(90, 1);
        sleep(1000);
        move(90, 0.3);
        sleep(100);
        move(90, 0.3);
        while (odsOut.getRawLightDetected() < 1.5) {
            sleep(5);
        }
        pressBeacon();
        sleep(100);
        move(90, 0.3);
        sleep(300);
        move(270, 0.3);
        while (odsOut.getRawLightDetected() < 1.5) {
            sleep(5);
        }
        stopMoving();
        move(90, 0.25);
        while (odsOut.getRawLightDetected() < 1.5) {
            sleep(5);
        }
        stopMoving();
        move(0, 1);
        sleep(200);
        stopMoving();
        turnToAngle(-35, 0.2);
        sleep(100);
        gyro.resetZAxisIntegrator();
        move(0, 1);
        sleep(2200);
        stopMoving();
    }
}
