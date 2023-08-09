package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

import java.util.List;

public class HardwareSwerve {

    /* Declare OpMode members. */
    private DcMotorEx topLeft  = null;
    private DcMotorEx botLeft  = null;
    private DcMotorEx topRight = null;
    private DcMotorEx botRight = null;

    public DcMotor         lift        = null;

    private Servo           intake        = null;

    private DigitalChannel  digitalUp;
    private TouchSensor     digitalDown;

    private ElapsedTime     runtime = new ElapsedTime();

    private double          robotHeading = 0.0;

    final static double countPerRev   = 528.65;
    final static double radPerRev     = 2*Math.PI;
    final static double countPerRad   = countPerRev/radPerRev;
    final double joystickDeadzone     = 0.1;
    final double posDeadzone          = 20;
    final static double driveSpeed    = 0.2;
    final static double deltaVSpeed   = 0.1;
    double direction                  = 0;
    double currentRad                 = 0;
    double optimumRad                 = 0;
    double targetVelocity             = 0;
    double targetAngle                = 0;
    double currentCount               = 0;

    // The IMU sensor object
    BNO055IMU imu;

    // IMU variables for telemetry
    Orientation angles;
    Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    public ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSwerve(){

    }

    public void botInit(HardwareMap map){
        // Save reference to Hardware map
        hwMap = map;

        //Define and Initialize drive-train Motors
        topLeft = hwMap.get(DcMotorEx.class, "topLeft");
        botLeft = hwMap.get(DcMotorEx.class, "botLeft");
        topRight = hwMap.get(DcMotorEx.class, "topRight");
        botRight = hwMap.get(DcMotorEx.class, "botRight");

        // Set motor Direction
        topLeft.setDirection(DcMotor.Direction.FORWARD);
        botLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        botRight.setDirection(DcMotor.Direction.REVERSE);

        // Set motor Brake mode
        setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset and initialize the motor encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void setDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        topLeft.setZeroPowerBehavior(behavior);
        botLeft.setZeroPowerBehavior(behavior);
        topRight.setZeroPowerBehavior(behavior);
        botRight.setZeroPowerBehavior(behavior);

    }

    public void setDriveMode(DcMotor.RunMode mode) {
        topLeft.setMode(mode);
        botLeft.setMode(mode);
        topRight.setMode(mode);
        botRight.setMode(mode);
    }

    public void setRadAndVel (double yInput, double xInput){
        double rad = Math.atan2(Math.abs(yInput), xInput);
        if (yInput < 0){
            rad = (2*Math.PI) - rad;
        }
        targetAngle = rad * (180/Math.PI);
        setOptimumRad(rad, currentRad);
        double velocity = Math.sqrt(Math.abs(yInput*yInput)+Math.abs(xInput*xInput));
        targetVelocity = velocity * direction;
    }

    public void setOptimumRad (double target, double current){
        double optimum =  target - current;
        if (optimum >  Math.PI/2 | optimum < -1*(Math.PI/2)){
            optimum = target + current;
            direction = -1;
        }else {
            direction = 1;
        }
        optimumRad = optimum;
    }

    public double leftAngleCorrection (double angle) {
        double tlPow = Math.abs(topLeft.getCurrentPosition() % countPerRev);
        double blPow = Math.abs(botLeft.getCurrentPosition() % countPerRev);
        double tarCount = angle * countPerRad;
        currentCount = tlPow - blPow;
        currentRad = currentCount/countPerRad;
        return deltaVSpeed * dzPos(tarCount - currentCount);
    }
    /*
        public double rightAngleCorrection (double angle) {
            double trPow = Math.abs(topRight.getCurrentPosition() % countPerRev);
            double brPow = Math.abs(botRight.getCurrentPosition() % countPerRev);
            double radCount = angle * countPerRad;
            currentRad = trPow - brPow;
            return deltaVSpeed * dzPos(radCount - currentRad);
        }
    */
    public double dzJoy(double input){
        double output;
        if(Math.abs(input) < joystickDeadzone) {
            output = 0.0;
        }else {
            output = input;
        }
        return output;
    }

    public double dzPos(double input){
        double output;
        if(Math.abs(input) < posDeadzone) {
            output = 0.0;
        }else {
            output = input;
        }
        return output;
    }




}