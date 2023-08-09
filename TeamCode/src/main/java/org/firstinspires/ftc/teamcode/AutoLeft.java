package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name="AutoLeft", group="Test")
public class AutoLeft extends IMU18380_Left{
    public void runOpMode() {


        List<Recognition> objectDetection;

        objectDetection = tfod.getRecognitions();

        encoderDrive(DRIVE_SPEED, 3, 0, 0, 5.0);// off wall
        encoderStrafe(DRIVE_SPEED, 24, 0, 0, 5.0); // 23 inches right
        encoderTurn(TURN_SPEED, -90, 10, 0, 5.0);
        encoderStrafe(DRIVE_SPEED, 35, 24, 0, 5.0); // 46 inches forward
        encoderDrive(DRIVE_SPEED, 4.375, 23, 0, 5.0);
        encoderLift(liftSpeed, 20, 1, 5.0); // up
        encoderDrive(DRIVE_SPEED, -4.375, 10, 1, 5.0);

        for (Recognition object : objectDetection) {
            if (object.getLabel().equals("Triangle")) { // Object 1 path
                encoderStrafe(DRIVE_SPEED, 12, 10, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                encoderDrive(DRIVE_SPEED, 43, 0, 1, 5.0); // move backwards 48 inches, set lift to 5 inches, keep claw open.
            } else if (object.getLabel().equals("Circle")) { // Object 2 path
                encoderStrafe(DRIVE_SPEED, 12, 10, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                encoderDrive(DRIVE_SPEED, 24, 0, 1, 5.0); // move backwards 24 inches, set lift to 5 inches, keep claw open.
            } else if (object.getLabel().equals("Square")) { // Object 3 path
                encoderStrafe(DRIVE_SPEED, 12, 0, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
            } else {
                encoderStrafe(DRIVE_SPEED, 12, 0, 1, 5.0); //strafe left 41 inches, set lift to 15 inches, keep claw open.
            }
        }
    }
}
