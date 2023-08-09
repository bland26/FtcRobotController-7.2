package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class Hive extends Stinger {

        final double     STRAFE_INCH_PER_REV;
        final double     DEGREE_PER_REV;
        final double     LIFT_INCH_PER_REV;


        // Claw variables

        final double ClawMin;
        final double ClawMax;
        final double ClawHome;


        // IMU correction constants
        final double     P_TURN_COEFF;     // Larger is more responsive, but also less stable
        final double     P_DRIVE_COEFF;     // Larger is more responsive, but also less stable

        // Speed constants
        final double     DRIVE_SPEED;
        final double     TURN_SPEED;
        final double     liftSpeed;

        // Object detection variables
        final String TFOD_MODEL_FILE  = "model_20221206_120301.tflite";
        final String[] LABELS = {
                "Circle",
                "Square",
                "Triangle"
        };

        public Hive(){
                STRAFE_INCH_PER_REV     = 8.75;
                DEGREE_PER_REV          = 44.0;
                LIFT_INCH_PER_REV       = 6.375;

                // Claw variables

                ClawMin          = 0.0;
                ClawMax          = 0.14;
                ClawHome         = 0.3;


                // IMU correction constants
                P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
                P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

                // Speed constants
                DRIVE_SPEED             = 0.6;
                TURN_SPEED              = 0.5;
                liftSpeed               = 0.8;
        }

        public void initTfod() {
                int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfodParameters.minResultConfidence = 0.8f;
                tfodParameters.isModelTensorFlow2 = true;
                tfodParameters.inputSize = 320;
                tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
                tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }

        public void clawState(int state){
                if (state == 1){
                        Grippy.setPosition(ClawMax);

                }
                else{
                        Grippy.setPosition(ClawMin);
                }
        }
        public void encoderDrive(double speed, double inches, double liftHeight,
                                 int clawOpen, double timeoutS) {
                int newLeftBTarget;
                int newRightBTarget;
                int newLeftFTarget;
                int newRightFTarget;
                int newLiftTarget;
                double  max;
                double  steer;
                double  leftSpeed;
                double  rightSpeed;
                double targetHeading = robotHeading;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                        // Determine new target position, and pass to motor controller
                        newLeftBTarget = LeftBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                        newRightBTarget = RightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                        newLeftFTarget = LeftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                        newRightFTarget = RightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
                        newLiftTarget = (int)(liftHeight * LIFT_COUNTS_PER_INCH);
                        LeftBack.setTargetPosition(newLeftBTarget);
                        RightBack.setTargetPosition(newRightBTarget);
                        LeftFront.setTargetPosition(newLeftFTarget);
                        RightFront.setTargetPosition(newRightFTarget);
                        lift.setTargetPosition(newLiftTarget);

                        // Turn On RUN_TO_POSITION
                        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // reset the timeout time and start motion.
                        runtime.reset();
                        LeftBack.setPower(Math.abs(speed));
                        RightBack.setPower(Math.abs(speed));
                        LeftFront.setPower(Math.abs(speed));
                        RightFront.setPower(Math.abs(speed));
                        lift.setPower(Math.abs(liftSpeed));

                        // keep looping while we are still active, and there  is time left, and both motors are running.
                        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                        // its target position, the motion will stop.  This is "safer" in the event that the robot will
                        // always end the motion as soon as possible.
                        // However, if you require that BOTH motors have finished their moves before the robot continues
                        // onto the next step, use (isBusy() || isBusy()) in the loop test.
                        while (opModeIsActive() &&
                                (runtime.seconds() < timeoutS) &&
                                (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy()
                                )) {

                                // read the orientation of the robot
                                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                                // and save the heading
                                robotHeading = angles.firstAngle;

                                steer = targetHeading - robotHeading;

                                if (steer < 1.5 && steer > -1.5){
                                        steer = 0;
                                }

                                leftSpeed = speed - (steer * P_DRIVE_COEFF);
                                rightSpeed = speed + (steer * P_DRIVE_COEFF);

                                // Normalize speeds if either one exceeds +/- 1.0;
                                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                                if (max > 1.0)
                                {
                                        leftSpeed /= max;
                                        rightSpeed /= max;
                                }

                                LeftBack.setPower(leftSpeed);
                                RightBack.setPower(rightSpeed);
                                LeftFront.setPower(leftSpeed);
                                RightFront.setPower(rightSpeed);

                                if (!digitalUp.getState()){
                                        lift.setPower(0);
                                }
                                if (digitalDown.isPressed()){
                                        lift.setPower(0);
                                }

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
                        lift.setPower(0);

                        clawState(clawOpen);

                        // Turn off RUN_TO_POSITION
                        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        sleep(250);   // optional pause after each move.
                }
        }
        // Positive values for right strafe, negative for left strafe.
        public void encoderStrafe(double speed, double inches, double liftHeight,
                                  int clawOpen, double timeoutS) {
                int newLeftBTarget;
                int newRightBTarget;
                int newLeftFTarget;
                int newRightFTarget;
                int newLiftTarget;
                double  max;
                double  steer;
                double  frontSpeed;
                double  backSpeed;
                double targetHeading = robotHeading;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                        // Determine new target position, and pass to motor controller
                        newLeftBTarget = LeftBack.getCurrentPosition() + (int)(-inches * STRAFE_COUNTS_PER_INCH);
                        newRightBTarget = RightBack.getCurrentPosition() + (int)(inches * STRAFE_COUNTS_PER_INCH);
                        newLeftFTarget = LeftFront.getCurrentPosition() + (int)(inches * STRAFE_COUNTS_PER_INCH);
                        newRightFTarget = RightFront.getCurrentPosition() + (int)(-inches * STRAFE_COUNTS_PER_INCH);
                        newLiftTarget = (int)(liftHeight * LIFT_COUNTS_PER_INCH);
                        LeftBack.setTargetPosition(newLeftBTarget);
                        RightBack.setTargetPosition(newRightBTarget);
                        LeftFront.setTargetPosition(newLeftFTarget);
                        RightFront.setTargetPosition(newRightFTarget);
                        lift.setTargetPosition(newLiftTarget);

                        // Turn On RUN_TO_POSITION
                        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // reset the timeout time and start motion.
                        runtime.reset();
                        LeftBack.setPower(Math.abs(speed));
                        RightBack.setPower(Math.abs(speed));
                        LeftFront.setPower(Math.abs(speed));
                        RightFront.setPower(Math.abs(speed));
                        lift.setPower(Math.abs(liftSpeed));

                        // keep looping while we are still active, and there  is time left, and both motors are running.
                        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                        // its target position, the motion will stop.  This is "safer" in the event that the robot will
                        // always end the motion as soon as possible.
                        // However, if you require that BOTH motors have finished their moves before the robot continues
                        // onto the next step, use (isBusy() || isBusy()) in the loop test.
                        while (opModeIsActive() &&
                                (runtime.seconds() < timeoutS) &&
                                (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy()
                                )) {


                                // read the orientation of the robot
                                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                                // and save the heading
                                robotHeading = angles.firstAngle;

                                steer = targetHeading - robotHeading;

                                if (steer < 1.5 && steer > -1.5){
                                        steer = 0;
                                }

                                frontSpeed = speed - (steer * P_DRIVE_COEFF);
                                backSpeed = speed + (steer * P_DRIVE_COEFF);


                                // Normalize speeds if either one exceeds +/- 1.0;
                                max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
                                if (max > 1.0)
                                {
                                        frontSpeed /= max;
                                        backSpeed /= max;
                                }

                                LeftBack.setPower(backSpeed);
                                RightBack.setPower(backSpeed);
                                LeftFront.setPower(frontSpeed);
                                RightFront.setPower(frontSpeed);



                                if (!digitalUp.getState()){
                                        lift.setPower(0);
                                }
                                if (digitalDown.isPressed()){
                                        lift.setPower(0);
                                }

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
                        lift.setPower(0);

                        clawState(clawOpen);

                        // Turn off RUN_TO_POSITION
                        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        sleep(250);   // optional pause after each move.
                }
        }
        // Positive values for clockwise, negative for counter clockwise.
        public void encoderTurn(double speed, double degrees, double liftHeight, int clawOpen,
                                double timeoutS) {
                int newLeftBTarget;
                int newRightBTarget;
                int newLeftFTarget;
                int newRightFTarget;
                int newLiftTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                        // Determine new target position, and pass to motor controller
                        newLeftBTarget = LeftBack.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
                        newRightBTarget = RightBack.getCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);
                        newLeftFTarget = LeftFront.getCurrentPosition() + (int)(degrees * COUNTS_PER_DEGREE);
                        newRightFTarget = RightFront.getCurrentPosition() + (int)(-degrees * COUNTS_PER_DEGREE);
                        newLiftTarget = (int)(liftHeight * LIFT_COUNTS_PER_INCH);
                        LeftBack.setTargetPosition(newLeftBTarget);
                        RightBack.setTargetPosition(newRightBTarget);
                        LeftFront.setTargetPosition(newLeftFTarget);
                        RightFront.setTargetPosition(newRightFTarget);
                        lift.setTargetPosition(newLiftTarget);

                        // Turn On RUN_TO_POSITION
                        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // reset the timeout time and start motion.
                        runtime.reset();
                        LeftBack.setPower(Math.abs(speed));
                        RightBack.setPower(Math.abs(speed));
                        LeftFront.setPower(Math.abs(speed));
                        RightFront.setPower(Math.abs(speed));
                        lift.setPower(Math.abs(liftSpeed));

                        // keep looping while we are still active, and there  is time left, and both motors are running.
                        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                        // its target position, the motion will stop.  This is "safer" in the event that the robot will
                        // always end the motion as soon as possible.
                        // However, if you require that BOTH motors have finished their moves before the robot continues
                        // onto the next step, use (isBusy() || isBusy()) in the loop test.
                        while (opModeIsActive() &&
                                (runtime.seconds() < timeoutS) &&
                                (LeftBack.isBusy() && RightBack.isBusy() && LeftFront.isBusy() && RightFront.isBusy()
                                )) {

                                if (!digitalUp.getState()){
                                        lift.setPower(0);
                                }
                                if (digitalDown.isPressed()){
                                        lift.setPower(0);
                                }

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
                        lift.setPower(0);

                        clawState(clawOpen);

                        // Turn off RUN_TO_POSITION
                        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        sleep(250);   // optional pause after each move.
                }
        }

        public void encoderLift ( double speed, double liftHeight, int clawOpen, double timeoutS){

                int newLiftTarget;


                // Ensure that the opmode is still active
                if (opModeIsActive()) {
                        newLiftTarget = (int)(liftHeight * LIFT_COUNTS_PER_INCH);

                        lift.setTargetPosition(newLiftTarget);

                        // Turn On RUN_TO_POSITION
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        // reset the timeout time and start motion.
                        runtime.reset();
                        lift.setPower(Math.abs(speed));

                        while (opModeIsActive() &&
                                (runtime.seconds() < timeoutS) &&
                                (lift.isBusy())) {


                                if (!digitalUp.getState()){
                                        lift.setPower(0);
                                }
                                if (!digitalDown.isPressed()){
                                        lift.setPower(0);
                                }


                        }

                        // Stop all motion;
                        LeftBack.setPower(0);
                        RightBack.setPower(0);
                        LeftFront.setPower(0);
                        RightFront.setPower(0);
                        lift.setPower(0);

                        clawState(clawOpen);

                        // Turn off RUN_TO_POSITION
                        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                        sleep(250);   // optional pause after each move.
                }
        }


        public void imuSpin (double speed, double degrees, double liftHeight, int clawOpen, double timeoutS){

                double steer = 100;
                double max;
                double leftSpeed;
                double rightSpeed;
                int newLiftTarget;

                if(opModeIsActive()) {

                        newLiftTarget = (int) (liftHeight * LIFT_COUNTS_PER_INCH);
                        lift.setTargetPosition(newLiftTarget);

                        runtime.reset();
                        lift.setPower(Math.abs(liftSpeed));

                        while (opModeIsActive() &&
                                (runtime.seconds() < timeoutS) && (Math.abs(steer) > 0.5)) {

                                // read the orientation of the robot
                                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                                // and save the heading
                                robotHeading = angles.firstAngle;

                                steer = robotHeading - degrees;




                                rightSpeed = speed * (steer * P_TURN_COEFF);
                                leftSpeed = -rightSpeed;


                                // Normalize speeds if either one exceeds +/- 1.0;
                                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                                if (max > 1.0) {
                                        leftSpeed /= max;
                                        rightSpeed /= max;
                                }

                                LeftBack.setPower(leftSpeed);
                                RightBack.setPower(rightSpeed);
                                LeftFront.setPower(leftSpeed);
                                RightFront.setPower(rightSpeed);

                                if (!digitalUp.getState()) {
                                        lift.setPower(0);
                                }
                                if (!digitalDown.isPressed()) {
                                        lift.setPower(0);
                                }
                        }

                        // Stop all motion;
                        LeftBack.setPower(0);
                        RightBack.setPower(0);
                        LeftFront.setPower(0);
                        RightFront.setPower(0);
                        lift.setPower(0);

                        clawState(clawOpen);

                        sleep(250);   // optional pause after each move.
                }
        }


}
