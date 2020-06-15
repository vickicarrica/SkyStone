package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="Foundation Red", group="Autonomous")
public class FoundationRed extends LinearOpMode{

    Robot_OmniDrive robot = new Robot_OmniDrive();
    private Servo leftServo = null;
    private Servo rightServo = null;

    @Override
    public void runOpMode() {
        final long WAIT = 1000;
        robot.initDrive(this);
        leftServo = hardwareMap.get(Servo.class, "left_servo");

        waitForStart();
        robot.encoderDrive(.06, -10, -10, 10, 10, 1);
        robot.stopMotor();
        sleep(WAIT);
        robot.encoderDrive(.06, -10, -10, 10, 10, 1);
        robot.stopMotor();
        sleep(WAIT);
        rightServo.setPosition(1);
        leftServo.setPosition(1);
        robot.stopMotor();
        sleep(WAIT);
        robot.encoderDrive(.06, 10, -10, 10, -10, 1.9);
        robot.stopMotor();
        sleep(WAIT);
        rightServo.setPosition(0);
        leftServo.setPosition(0);
        robot.stopMotor();
        sleep(WAIT);
        robot.encoderDrive(.06, 10, 10, 10, -10, 2.2);
        sleep(WAIT);
    }
}
