package org.firstinspires.ftc.teamcode.ConceptTests;
/*
This program was created by Vicki Carrica *
More information at the bottom of the code.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BasicTeleOp extends LinearOpMode {

    //initialize FTC variables
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //initialize hardware variables
        left  = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        //"Most robots need the motor on one side to be reversed to drive forward"
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        //run until the end of the match:
        while(opModeIsActive()){
            double leftPower;
            double rightPower;

            //POV mode:
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(/*number:*/ drive + turn, /*min:*/ -1, /*max:*/ 1);
            rightPower = Range.clip(/*number:*/ drive - turn, /*min:*/ -1, /*max:*/ 1);

            /*
            Tank mode: (commented out to avoid errors)
            leftPower = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;
            Uncomment if you choose to use tank mode
             */

            left.setPower(leftPower);
            right.setPower(rightPower);

            //Final game time and game power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
/*
*All Rights Reserved

All files can be found on my GitHub vickicarrica* -> repositories -> SkyStone (or current year's theme)


No objects- objects can be created in a different file and can be used for more complex applications.
See:
- Class Hierarchy
- teleOpDemo (Trobot* object)
    - TeamCode -> java -> org.firstinspires.ftc.teamcode -> ConceptTests -> teleOpDemo


POV mode vs Tank mode:

POV:
* more math
* can move:
* -quicker
* -straighter (easier)
Tank:
* less math
* can move:
* -slower
* -harder to go straight

See:
* BasicOpMode_Linear:
* FtcRobotController -> java -> org.firstinspires.ftc.robotcontroller -> external.samples -> BasicOpMode_Linear (2nd)

 */