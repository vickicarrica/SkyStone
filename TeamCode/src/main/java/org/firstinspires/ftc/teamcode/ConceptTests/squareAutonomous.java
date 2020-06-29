package org.firstinspires.ftc.teamcode.ConceptTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Basic autonomous that causes a robot to move in a square


@Autonomous (name= "Square Autonomous", group = "Autonomous")
public class squareAutonomous extends LinearOpMode {

    //Define DcMotor variables
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightBackMotor;

    @Override
    public void runOpMode(){
        //Pairs each DcMotor in the code with its corresponding motor port in the robot
        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightBackMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_back");
        rightBackMotor = hardwareMap.dcMotor.get("right_back");

        //"Most robots need the motor on one side to be reversed to drive forward" (FTC BasicOpMode_Linear)
        //sets both left motors (front&back) reversed
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        //Wait for start:
        waitForStart();

        //for loop to make 4 sides:
        for (int i = 0; i < 4; i ++)
        {
            //motors move at 25% for 3 seconds:
            leftFrontMotor.setPower(.25);
            rightFrontMotor.setPower(.25);
            leftBackMotor.setPower(.25);
            rightBackMotor.setPower(.25);
            sleep(3000);

            //stops motors:
            leftFrontMotor.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftBackMotor.setPower(0.0);
            rightBackMotor.setPower(0.0);

            //waits for 1 sec until next command:
            sleep(1000);

            if (i < 3) //will not turn after the final side
            {
                //turns (motors go in the opposite direction):
                leftFrontMotor.setPower(.25);
                rightFrontMotor.setPower(-.25);
                leftBackMotor.setPower(.25);
                rightBackMotor.setPower(-.25);

                sleep(2000); //guess, will need to be adjusted depending if it turned to little or too much

                //stop motors:
                leftFrontMotor.setPower(0.0);
                rightFrontMotor.setPower(0.0);
                leftBackMotor.setPower(0.0);
                rightBackMotor.setPower(0.0);

                //waits for 1 sec until next command:
                sleep(1000);
            }
        }
    }
}
