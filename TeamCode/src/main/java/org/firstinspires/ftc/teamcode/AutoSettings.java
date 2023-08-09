package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class AutoSettings {

    /* Declare OpMode members. */
    HardwareSR24         robot;   // Use a Pushbot's hardware
    HardwareMap          hardwareMap;
    LinearOpMode         opMode;

    private ElapsedTime     runtime = new ElapsedTime();

    public ElapsedTime      autoTime = new ElapsedTime();

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    /*
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    /*
     * Initialize the Vuforia localization engine.
     */


    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 1.0;   // Nominal speed for auto moves.
    static final double     DRIVE_SPEED_SLOW        = 0.8;  // Slower speed where required
    static final double     TURN_SPEED              = 0.8;   // Turn speed

    static final double     HEADING_THRESHOLD       = 1.0 ;    // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.011; // Larger is more responsive, but also less accurate
    static final double     P_TURN_COEFF_180        = 0.009; // For turns closer to 180 degrees. Less responsive, but more accurate to account for momentum coming out of long turns.
    static final double     P_TURN_COEFF_STRONG     = 0.150; // For small 1 degree adjustment turns
    static final double     P_DRIVE_COEFF_1         = 0.01;  // Larger is more responsive, but also less accurate
    static final double     P_DRIVE_COEFF_2         = 0.25;  // Intenionally large so robot "wiggles" around the target setpoint while driving

    final String TFOD_MODEL_FILE  = "model_20221206_120301.tflite";
    final String[] LABELS = {
            "Circle",
            "Square",
            "Triangle"
    };

    public AutoSettings(){

    }

    public void init(HardwareSR24 r, HardwareMap hw, LinearOpMode op){
        // Remember the robot and hardareMap references
        robot = r;
        hardwareMap = hw;
        opMode = op;

        // Force reset the drive train encoders.  Do it twice as sometimes this gets missed due to USB congestion
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RobotLog.i("DM10337 -- Drive train encoders reset");

        RobotLog.i("DM10337- Finished Init");





    }

    public  final String VUFORIA_KEY =
            "AZJC/3T/////AAABmRPn6kJC6k6BmqQQ09BqPMdxzm82RZmhCzQAUffgUDxWqKsnQDlYnQFZtG4Flyw/K/G5bXw" +
                    "Wa8z4LXQTxjqjga40ZY3pp73399DYqjOK6jl2BJl5uBss7OHkvUEDlw5kyWoU6xoSfPfNkMJt3Vg2JBl" +
                    "8CDGzXJkuzlGdqo5Hzb48A+8tf3kQv5xvCm90OMSxy48dRFRSfTX+o0yh0NT8l2ihKJ52TJlwYBXnxUD" +
                    "Q7jGInINMZ+SelJe/RCssYGf/YhEKDeqhCTGiBG9Lv9p1K/GFqdnVcRUtkkEoISSO8NnICbVzTTvG+jn" +
                    "2mUUCACNTKwiN6l4lyduZaj9Y3IQ5ioExnW/rdUeat3FEaAVx8vKc";

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

    public void encoderDrive(double speed, double inches, double liftHeight,
                             int clawOpen, double timeoutS) {
        encoderDrive( speed, inches, liftHeight, clawOpen, timeoutS);
    }


    // Positive values for right strafe, negative for left strafe.
    public void encoderStrafe(double speed, double inches, double liftHeight,
                              int clawOpen, double timeoutS) {
        encoderStrafe(speed, inches, liftHeight, clawOpen, timeoutS);
        }

    // Positive values for clockwise, negative for counter clockwise.
    public void encoderTurn(double speed, double degrees, double liftHeight, int clawOpen,
                            double timeoutS) {
        encoderTurn(speed, degrees, liftHeight, clawOpen, timeoutS);
    }

    public void encoderLift ( double speed, double liftHeight, int clawOpen, double timeoutS){

        encoderLift ( speed, liftHeight, clawOpen, timeoutS);
    }


    public void imuSpin (double speed, double degrees, double liftHeight, int clawOpen, double timeoutS){

        imuSpin (speed,degrees, liftHeight, clawOpen, timeoutS);
    }

}
