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

import java.util.List;

public class HardwareSR24 {

    /* Declare OpMode members. */
    public DcMotor         LeftBack    = null;
    public DcMotor         RightBack   = null;
    public DcMotor         LeftFront   = null;
    public DcMotor         RightFront  = null;

    public DcMotor         lift        = null;

    private Servo           intake        = null;

    private DigitalChannel  digitalUp;
    private TouchSensor     digitalDown;

    private ElapsedTime     runtime = new ElapsedTime();

    private double          robotHeading = 0.0;

    // The IMU sensor object
    BNO055IMU imu;

    // IMU variables for telemetry
    Orientation angles;
    Acceleration gravity;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    public ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSR24(){

    }

    public void botInit(HardwareMap map){
        // Save reference to Hardware map
        hwMap = map;

        //Define and Initialize drive-train Motors
        LeftBack = hwMap.get(DcMotor.class, "LeftBack");
        RightBack = hwMap.get(DcMotor.class, "RightBack");
        LeftFront = hwMap.get(DcMotor.class, "LeftFront");
        RightFront = hwMap.get(DcMotor.class, "RightFront");

        // Set motor Direction
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.REVERSE);

        // Set motor Brake mode
        setDriveZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset and initialize the motor encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void setDriveZeroPower(DcMotor.ZeroPowerBehavior behavior) {
        LeftBack.setZeroPowerBehavior(behavior);
        RightBack.setZeroPowerBehavior(behavior);
        LeftFront.setZeroPowerBehavior(behavior);
        RightFront.setZeroPowerBehavior(behavior);

    }

    public void setDriveMode(DcMotor.RunMode mode) {
        LeftBack.setMode(mode);
        RightBack.setMode(mode);
        LeftFront.setMode(mode);
        RightFront.setMode(mode);
    }




}
