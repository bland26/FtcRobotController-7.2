//stinger robotics auto drive w/ encoder
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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


/*
 *This program will create an autonomous drive Op by using sequences of functions for
 * 1. driving forward and reverse - encoderDrive(inches, inches, timeout)
 * 2. strafing left and right - encoderStrafe(inches, inches, timeout)
 * 3. rotating clockwise and counter clockwise - encoderSpin(degrees,timeout)
 * using the encoder to translate distance entered into the proper motor revolutions and
 * an internal IMU to correct deviation from desired direction.
 */

@Autonomous(name="Auto Drive IMU correction", group="Linear Opmode")
//@Disabled
public class StingerDriveEncoderIMU extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LeftBack    = null;
    private DcMotor         RightBack   = null;
    private DcMotor         LeftFront   = null;
    private DcMotor         RightFront  = null;
    private DcMotor         Lift        = null;
    private Servo           Claw        = null;
    private DigitalChannel  TopLimit;
    private TouchSensor  BotLimit;

    private ElapsedTime     runtime = new ElapsedTime();

    private double          robotHeading;

    // The IMU sensor object
    BNO055IMU imu;

    // IMU variables for telemetry
    Orientation angles;
    Acceleration gravity;

    // encoder constants for count to distance/degree conversions
    static final double     COUNTS_PER_MOTOR_REV    = 528.65 ;    // HD Hex Motor (Rev-41-1291) 20:1
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 75/25.4 ;     // 75mm Mechanum wheels
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     STRAFE_INCH_PER_REV     = 10.0;
    static final double     STRAFE_COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            STRAFE_INCH_PER_REV;
    static final double     DEGREE_PER_REV          = 45.0;
    static final double     COUNTS_PER_DEGREE       = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            DEGREE_PER_REV;
    static final double     LIFT_INCH_PER_REV       = 6.75;
    static final double     LIFT_COUNTS_PER_INCH    = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            LIFT_INCH_PER_REV;

    // Claw variables
    final static double ClawHome              = 0.5;
    final static double ClawMinRange          = 0.0;
    final static double ClawMaxRange          = 1.0;


    // IMU correction constants
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05;     // Larger is more responsive, but also less stable

    // Speed constants
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.4;
    static final double     LIFT_SPEED              = 0.5;

    // Object detection variables
    private static final String TFOD_MODEL_FILE  = "model_20221206_120301.tflite";
    private static final String[] LABELS = {
            "Circle",
            "Square",
            "Triangle"
    };





    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Claw = hardwareMap.get(Servo.class, "Claw");
        TopLimit = hardwareMap.get(DigitalChannel.class, "TopLimit");
        BotLimit = hardwareMap.get(TouchSensor.class, "BotLimit");

        // Set the direction of motor rotation dependant on the orientation of motors in the robot.
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        // Set motor Brake mode
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set claw position on init
        Claw.setPosition(ClawHome);
        // Set limit switch mode
        TopLimit.setMode(DigitalChannel.Mode.INPUT);

        //Reset and initialize the motor encoders
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Object detection  ///////////////////////////////////////////////////////////////////////////////


        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }


////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU parameters ///////////////////////////////////////////////////////////////////////////////////

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);





        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // read the orientation of the robot
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // and save the heading
        robotHeading = angles.firstAngle;

////////////////////////////////////////////////////////////////////////////////////////////////////
//AUTONOMOUS PATH BUILDS////////////////////////////////////////////////////////////////////////////

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        /* parameters :
        encoderDrive(DRIVE_SPEED, How many inches you want to move (+ forward - reverse),
        Where you want to set the lift height in inches,
        Whether you want the claw open (1) or closed (0) at the end of the step,
        maximum time allowed for the step before it automatically stops.)

        encoderStrafe(DRIVE_SPEED, How many inches you want to move (+ Right - Left),
        Where you want to set the lift height in inches,
        Whether you want the claw open (1) or closed (0) at the end of the step,
        maximum time allowed for the step before it automatically stops.)

        encoderSpin(DRIVE_SPEED, How many degrees you want to rotate (+ clockwise - counter clockwise),
        Where you want to set the lift height in inches,
        Whether you want the claw open (1) or closed (0) at the end of the step,
        maximum time allowed for the step before it automatically stops.)
         */
        List<Recognition> objectDetection;
        objectDetection = tfod.getRecognitions();

        for (Recognition object : objectDetection) {
            if (object.getLabel().equals("Triangle")) { // Object 1 path
                clawState(0); // claw closure.
                sleep(100); // ensure cone grabbed.
                encoderDrive(DRIVE_SPEED, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(TURN_SPEED, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, -41, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(DRIVE_SPEED, 4, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                liftOnly(LIFT_SPEED,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(DRIVE_SPEED, -4, 30, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(DRIVE_SPEED, 12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                encoderDrive(DRIVE_SPEED, -48, 5, 1, 5.0); // move backwards 48 inches, set lift to 5 inches, keep claw open.
            } else if (object.getLabel().equals("Square")) { // Object 2 path
                clawState(0); // claw closure.
                sleep(100); // ensure cone grabbed.
                encoderDrive(DRIVE_SPEED, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(TURN_SPEED, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, -41, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(DRIVE_SPEED, 4, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                liftOnly(LIFT_SPEED,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(DRIVE_SPEED, -4, 30, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(DRIVE_SPEED, 12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
                encoderDrive(DRIVE_SPEED, -24, 5, 1, 5.0); // move backwards 24 inches, set lift to 5 inches, keep claw open.
            } else if (object.getLabel().equals("Circle")) { // Object 3 path
                clawState(0); // claw closure.
                sleep(100); // ensure cone grabbed.
                encoderDrive(DRIVE_SPEED, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(TURN_SPEED, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, -41, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(DRIVE_SPEED, 4, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                liftOnly(LIFT_SPEED,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(DRIVE_SPEED, -4, 30, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(DRIVE_SPEED, 12, 15, 1, 5.0); //strafe right 12 inches, set lift to 15 inches, keep claw open.
            } else {
                clawState(0); // claw closure.
                sleep(100); // ensure cone grabbed.
                encoderDrive(DRIVE_SPEED, 3, 5, 0, 5.0);  // move forward 3 inches, set lift to 5 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, 24, 5, 0, 5.0); // strafe right 24 inches, keep lift at 5 inches, keep claw closed.
                encoderSpin(TURN_SPEED, 90, 16, 10, 5.0); //  rotate 90 degrees clockwise, set lift to 16 inches, keep claw closed.
                encoderStrafe(DRIVE_SPEED, -41, 36, 0, 5.0); //strafe left 41 inches, set lift to 36 inches, keep claw closed.
                encoderDrive(DRIVE_SPEED, 4, 36, 0, 5.0); // move forward 4 inches, keep lift at 36 inches, open claw.
                liftOnly(LIFT_SPEED,30,1, 5.0); // set lift to 30 inches, open claw.
                encoderDrive(DRIVE_SPEED, -4, 30, 1, 5.0); // move back 4 inches, keep lift at 36 inches, keep claw open.
                encoderStrafe(DRIVE_SPEED, 41, 15, 1, 5.0); //strafe right 41 inches, set lift to 15 inches, keep claw open.
            }
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private static final String VUFORIA_KEY =
            "AZJC/3T/////AAABmRPn6kJC6k6BmqQQ09BqPMdxzm82RZmhCzQAUffgUDxWqKsnQDlYnQFZtG4Flyw/K/G5bXw" +
                    "Wa8z4LXQTxjqjga40ZY3pp73399DYqjOK6jl2BJl5uBss7OHkvUEDlw5kyWoU6xoSfPfNkMJt3Vg2JBl" +
                    "8CDGzXJkuzlGdqo5Hzb48A+8tf3kQv5xvCm90OMSxy48dRFRSfTX+o0yh0NT8l2ihKJ52TJlwYBXnxUD" +
                    "Q7jGInINMZ+SelJe/RCssYGf/YhEKDeqhCTGiBG9Lv9p1K/GFqdnVcRUtkkEoISSO8NnICbVzTTvG+jn" +
                    "2mUUCACNTKwiN6l4lyduZaj9Y3IQ5ioExnW/rdUeat3FEaAVx8vKc";

    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /*
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
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
            Claw.setPosition(ClawMaxRange);

        }
        else{
            Claw.setPosition(ClawMinRange);
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
        double targetHeading = 0.0;

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
            Lift.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));
            Lift.setPower(Math.abs(LIFT_SPEED));

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

                if (steer < 2.5 && steer > -2.5){
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

                if (!TopLimit.getState()){
                    Lift.setPower(0);
                }
                if (BotLimit.isPressed()){
                    Lift.setPower(0);
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
            Lift.setPower(0);

            clawState(clawOpen);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        double targetHeading = 0.0;

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
            Lift.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));
            Lift.setPower(Math.abs(LIFT_SPEED));

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

                if (steer < 2.5 && steer > -2.5){
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



                if (!TopLimit.getState()){
                    Lift.setPower(0);
                }
                if (BotLimit.isPressed()){
                    Lift.setPower(0);
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
            Lift.setPower(0);

            clawState(clawOpen);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    // Positive values for clockwise, negative for counter clockwise.
    public void encoderSpin(double speed, double degrees, double liftHeight, int clawOpen,
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
            Lift.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LeftBack.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));
            Lift.setPower(Math.abs(LIFT_SPEED));

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

                if (!TopLimit.getState()){
                    Lift.setPower(0);
                }
                if (BotLimit.isPressed()){
                    Lift.setPower(0);
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
            Lift.setPower(0);

            clawState(clawOpen);

            // Turn off RUN_TO_POSITION
            LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void liftOnly ( double speed, double liftHeight, int clawOpen, double timeoutS){

        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newLiftTarget = (int)(liftHeight * LIFT_COUNTS_PER_INCH);

            Lift.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            Lift.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (Lift.isBusy())) {


                if (!TopLimit.getState()){
                    Lift.setPower(0);
                }
                if (!BotLimit.isPressed()){
                    Lift.setPower(0);
                }


            }

            // Stop all motion;
            LeftBack.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            RightFront.setPower(0);
            Lift.setPower(0);

            clawState(clawOpen);

            // Turn off RUN_TO_POSITION
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }


    public void imuSpin (double speed, double degrees, double liftHeight, int clawOpen, double timeoutS){

        double steer = 1;
        double max;
        double leftSpeed;
        double rightSpeed;
        int newLiftTarget;

        if(opModeIsActive()) {

            newLiftTarget = (int) (liftHeight * LIFT_COUNTS_PER_INCH);
            Lift.setTargetPosition(newLiftTarget);

            runtime.reset();
            Lift.setPower(Math.abs(LIFT_SPEED));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && steer != 0) {

                // read the orientation of the robot
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                // and save the heading
                robotHeading = angles.firstAngle;

                steer = degrees - robotHeading;

                if (steer < 0.1 && steer > -0.1) {
                    steer = 0;
                }


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

                if (!TopLimit.getState()) {
                    Lift.setPower(0);
                }
                if (!BotLimit.isPressed()) {
                    Lift.setPower(0);
                }
            }

            // Stop all motion;
            LeftBack.setPower(0);
            RightBack.setPower(0);
            LeftFront.setPower(0);
            RightFront.setPower(0);
            Lift.setPower(0);

            clawState(clawOpen);

            sleep(250);   // optional pause after each move.
        }
    }

}
