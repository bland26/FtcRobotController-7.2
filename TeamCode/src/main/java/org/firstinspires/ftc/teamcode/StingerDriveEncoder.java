/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//stinger robotics auto drive w/ encoder
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *This program will create an autonomous drive Op by using sequences of functions for
 * 1. driving forward and reverse - encoderDrive(leftInches, rightInches, timeout)
 * 2. strafing left and right - encoderStrafe(leftInches, rightInches, timeout)
 * 3. rotating clockwise and counter clockwise - encoderSpin(degrees,timeout)
 * using the encoder to translate distance entered into the proper motor revolutions
 */

@Autonomous(name="Auto Drive By Encoder", group="Linear Opmode")
public class StingerDriveEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LeftBack   = null;
    private DcMotor         RightBack  = null;
    private DcMotor         LeftFront   = null;
    private DcMotor         RightFront  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // encoder count to distance/degree conversions
    static final double     COUNTS_PER_MOTOR_REV    = 528.65 ;    // HD Hex Motor (Rev-41-1291) 20:1
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 75/25.4 ;     // 75mm Mechanum wheels
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_INCH_PER_REV     = 10.0;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.4;
    static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            STRAFE_INCH_PER_REV;
    static final double     DEGREE_PER_REV          = 45.0;
    static final double     COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            DEGREE_PER_REV;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftBack  = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);

        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                LeftBack.getCurrentPosition(),
                RightBack.getCurrentPosition(),
                LeftFront.getCurrentPosition(),
                RightFront.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // move forward 1 foot
        //encoderStrafe(DRIVE_SPEED,   12, 12, 5.0);  // strafe right 1 foot
        //encoderStrafe(DRIVE_SPEED, -12,-12,5.0); // strafe left 1 foot
        encoderSpin(TURN_SPEED,180,5.0); // spin 180 degrees
        encoderDrive(DRIVE_SPEED, 12, 12, 5.0);  // move backward 1 foot

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftBTarget;
        int newRightBTarget;
        int newLeftFTarget;
        int newRightFTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBTarget = LeftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBTarget = RightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFTarget = LeftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFTarget = RightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LeftBack.setTargetPosition(newLeftBTarget);
            RightBack.setTargetPosition(newRightBTarget);
            LeftFront.setTargetPosition(newLeftFTarget);
            RightFront.setTargetPosition(newRightFTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there  is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBTarget,  newRightBTarget,
                        newLeftFTarget, newRightFTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        LeftBack.getCurrentPosition(), RightBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(), RightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftBack.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            RightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    // Positive values for right strafe, negative for left strafe.
    public void encoderStrafe(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftBTarget;
        int newRightBTarget;
        int newLeftFTarget;
        int newRightFTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBTarget = LeftBack.getCurrentPosition() + (int)(-leftInches * STRAFE_COUNTS_PER_INCH);
            newRightBTarget = RightBack.getCurrentPosition() + (int)(rightInches * STRAFE_COUNTS_PER_INCH);
            newLeftFTarget = LeftFront.getCurrentPosition() + (int)(leftInches * STRAFE_COUNTS_PER_INCH);
            newRightFTarget = RightFront.getCurrentPosition() + (int)(-rightInches * STRAFE_COUNTS_PER_INCH);
            LeftBack.setTargetPosition(newLeftBTarget);
            RightBack.setTargetPosition(newRightBTarget);
            LeftFront.setTargetPosition(newLeftFTarget);
            RightFront.setTargetPosition(newRightFTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there  is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBTarget,  newRightBTarget,
                        newLeftFTarget, newRightFTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        LeftBack.getCurrentPosition(), RightBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(), RightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftBack.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            RightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    // Positive values for clockwise, negative for counter clockwise.
    public void encoderSpin(double speed,
                            double degrees,
                            double timeoutS) {
        int newLeftBTarget;
        int newRightBTarget;
        int newLeftFTarget;
        int newRightFTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBTarget = LeftBack.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newRightBTarget = RightBack.getCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);
            newLeftFTarget = LeftFront.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
            newRightFTarget = RightFront.getCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);
            LeftBack.setTargetPosition(newLeftBTarget);
            RightBack.setTargetPosition(newRightBTarget);
            LeftFront.setTargetPosition(newLeftFTarget);
            RightFront.setTargetPosition(newRightFTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there  is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBTarget,  newRightBTarget,
                        newLeftFTarget, newRightFTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        LeftBack.getCurrentPosition(), RightBack.getCurrentPosition(),
                        LeftFront.getCurrentPosition(), RightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftBack.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            RightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    /* placeholder for crane function
     * public void encoderSpin(double speed,
                            double degrees,
                            double timeoutS) {
        }
    */

}
