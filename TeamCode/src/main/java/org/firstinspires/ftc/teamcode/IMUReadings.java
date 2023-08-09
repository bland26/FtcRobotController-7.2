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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="IMUReadings", group="Test")
//@Disabled
public class IMUReadings extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftBack = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor RightFront = null;
    private DcMotor lift = null;
    private Servo Grippy = null;
    private DigitalChannel digitalUp = null;
    private DigitalChannel digitalDown =null;

    static double driveSpeed    = 0.8;
    static double ClawStart = 0.1;
    static double ClawMin = 0.0;
    static double ClawMax =0.14;
    static double ClawSpeed = 0.001;
    double ClawPosition = 0.1;
    static double liftSpeed = 0.6;

    // The IMU sensor object
    BNO055IMU imu;

    // IMU variables for telemetry
    Orientation angles;
    Acceleration gravity;


    /*
     * Code to run ONCE when the driver hits INIT
     */


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftBack  = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        lift = hardwareMap.get(DcMotor.class, "lift");
        digitalUp = hardwareMap.get(DigitalChannel.class, "digitalUp");
        digitalDown = hardwareMap.get(DigitalChannel.class, "digitalDown");


        //change claw to grippy
        Grippy = hardwareMap.get(Servo.class,"Claw");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        digitalUp.setMode(DigitalChannel.Mode.INPUT);
        digitalDown.setMode(DigitalChannel.Mode.INPUT);

        Grippy.setPosition(ClawStart);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //IMU parameters ///////////////////////////////////////////////////////////////////////////////////

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);




        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double LeftBackPower;
        double RightBackPower;
        double LeftFrontPower;
        double RightFrontPower;
        double liftPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        final double DEADZONE = 0.1;
        double drive = -gamepad1.left_stick_y;
        if(Math.abs(drive) < DEADZONE) {
            drive = 0;
        }
        double driveCubed = drive * drive * drive;
        double strafe = gamepad1.left_stick_x;
        if(Math.abs(strafe) < DEADZONE) {
            strafe = 0;
        }
        double strafeCubed = strafe * strafe * strafe;
        double turn  =  gamepad1.right_stick_x;
        if(Math.abs(turn) < DEADZONE) {
            turn = 0;
        }

        double turnCubed = turn * turn * turn;

        double liftInput = gamepad2.left_stick_y;
        if(Math.abs(liftInput) < DEADZONE) {
            liftInput = 0;
        }
        double liftCubed = liftInput * liftInput * liftInput;



        while (gamepad1.right_bumper && ClawPosition < ClawMax)
            ClawPosition += ClawSpeed;
        while (gamepad1.left_bumper && ClawPosition >= ClawMin)
            ClawPosition -= ClawSpeed;

        if (gamepad1.right_trigger > 0 && ClawPosition < ClawMax)
            ClawPosition += ClawSpeed;
        if (gamepad1.left_trigger > 0 && ClawPosition >= ClawMin)
            ClawPosition -= ClawSpeed;







        LeftBackPower    = Range.clip(driveCubed - strafeCubed + turnCubed, -1.0, 1.0) ;
        RightBackPower   = Range.clip(driveCubed + strafeCubed - turnCubed, -1.0, 1.0) ;
        LeftFrontPower    = Range.clip(driveCubed + strafeCubed + turnCubed, -1.0, 1.0) ;
        RightFrontPower   = Range.clip(driveCubed - strafeCubed - turnCubed, -1.0, 1.0) ;
        liftPower = Range.clip(liftCubed,-1.0, 1.0);

        // Send calculated power to wheels
        LeftBack.setPower(LeftBackPower*driveSpeed);
        RightBack.setPower(RightBackPower*driveSpeed);
        LeftFront.setPower(LeftFrontPower*driveSpeed);
        RightFront.setPower(RightFrontPower*driveSpeed);
        if (liftPower > 0 &&digitalUp.getState() == true) {
            lift.setPower(liftPower*liftSpeed);
        } else if (liftPower < 0 &&digitalDown.getState() == true) {
            lift.setPower(liftPower*liftSpeed);
        } else {
            lift.setPower(0);
        }

        //set servo speed
        ClawPosition = Range.clip(ClawPosition,ClawMin,ClawMax);
        Grippy.setPosition(ClawPosition);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Show the elapsed game time and wheel power.
        telemetry.addData( "Z Yaw", angles.firstAngle);
        telemetry.addData("X Pitch", angles.thirdAngle);
        telemetry.addData("Y Roll", angles.secondAngle);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

