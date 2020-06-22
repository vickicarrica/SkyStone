package org.firstinspires.ftc.teamcode.ConceptTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

//Basic autonomous that causes a robot to move in a square


@Autonomous (name= "Square Autonomous", group = "Autonomous")
public class squareAutonomous extends LinearOpMode {

    //Define DcMotor variables
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode(){
        //Pairs each DcMotor in the code with its corresponding motor port in the robot
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        //"Most robots need the motor on one side to be reversed to drive forward" (FTC BasicOpMode_Linear)
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Wait for start:
        waitForStart();

        //for loop to make 4 sides:
        for (int i = 0; i < 4; i ++)
        {
            //motors move at 25% for 3 seconds:
            leftMotor.setPower(.25);
            rightMotor.setPower(.25);
            sleep(3000);

            //stops motors:
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

            //waits for 1 sec until next command:
            sleep(1000);

            if (i < 3) //will not turn after the final side
            {
                //turns (motors go in the opposite direction):
                leftMotor.setPower(.25);
                rightMotor.setPower(-.25);
                sleep(2000); //guess, will need to be adjusted depending if it turned to little or too much

                //stop motors:
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);

                //waits for 1 sec until next command:
                sleep(1000);
            }
        }
    }
}
