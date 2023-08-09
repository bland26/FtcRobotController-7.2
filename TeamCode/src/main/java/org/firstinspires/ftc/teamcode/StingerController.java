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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

@TeleOp(name="Driver Controlled", group="Iterative Opmode")
public class StingerController extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor         LeftBack    = null;
    private DcMotor         RightBack   = null;
    private DcMotor         LeftFront   = null;
    private DcMotor         RightFront  = null;
    private DcMotor         Lift        = null;
    private Servo           Claw        = null;
    private DigitalChannel  TopLimit;
    private DigitalChannel  BotLimit;
    private TouchSensor     BotButton;
    static final double     COUNTS_PER_MOTOR_REV    = 528.65 ;    // HD Hex Motor (Rev-41-1291) 20:1
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    final static double ClawHome              = 0.5;
    final static double ClawMinRange          = 0.0;
    final static double ClawMaxRange          = 1.0;
    final static double ClawSpeed             = 0.001;
    final static double driveSpeed            = 0.6;
    final static double liftSpeed             = 0.6;
    double ClawPosition                       = 0.5;




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
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        Claw = hardwareMap.get(Servo.class, "Claw");
        TopLimit = hardwareMap.get(DigitalChannel.class, "TopLimit");
        BotLimit = hardwareMap.get(DigitalChannel.class, "BotLimit");
        BotButton = hardwareMap.get(TouchSensor.class, "BotButton");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        Lift.setDirection(DcMotor.Direction.FORWARD);
        Claw.setPosition(ClawHome);
        TopLimit.setMode(DigitalChannel.Mode.INPUT);
        BotLimit.setMode(DigitalChannel.Mode.INPUT);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        final double Deadzone = 0.1;
        double driveInput =  -gamepad1.left_stick_y;
        if(Math.abs(driveInput) < Deadzone) {
            driveInput = 0.0;
        }
        double drive = driveInput * driveInput * driveInput;

        double strafeInput =  gamepad1.left_stick_x;
        if(Math.abs(strafeInput) < Deadzone) {
            strafeInput = 0.0;
        }
        double strafe = strafeInput * strafeInput * strafeInput;

        double spinInput  =  gamepad1.right_stick_x;
        if(Math.abs(spinInput) < Deadzone) {
            spinInput = 0.0;
        }
        double spin = spinInput * spinInput * spinInput;

        double liftInput = -gamepad2.left_stick_y;
        if(Math.abs(liftInput) < Deadzone) {
            liftInput = 0.0;
        }
        double lift = liftInput * liftInput * liftInput;

        while (gamepad1.right_bumper && ClawPosition < ClawMaxRange)
            ClawPosition += ClawSpeed;
        while (gamepad1.left_bumper && ClawPosition >= ClawMinRange)
            ClawPosition -= ClawSpeed;

        if (gamepad1.right_trigger > 0 && ClawPosition < ClawMaxRange)
            ClawPosition += ClawSpeed;
        if (gamepad1.left_trigger > 0 && ClawPosition >= ClawMinRange)
            ClawPosition -= ClawSpeed;



        LeftBackPower    = Range.clip(drive + (-strafe) +   spin, -1.0, 1.0) ;
        RightBackPower   = Range.clip(drive +   strafe  + (-spin), -1.0, 1.0) ;
        LeftFrontPower   = Range.clip(drive +   strafe  +   spin, -1.0, 1.0) ;
        RightFrontPower  = Range.clip(drive + (-strafe) + (-spin), -1.0, 1.0) ;
        liftPower        = Range.clip(lift, -1.0, 1.0);


        // Send calculated power to wheels
        LeftBack.setPower(LeftBackPower * driveSpeed);
        RightBack.setPower(RightBackPower * driveSpeed);
        LeftFront.setPower(LeftFrontPower * driveSpeed);
        RightFront.setPower(RightFrontPower * driveSpeed);

        // Limit switch code for switches

        if (liftPower > 0 && TopLimit.getState()) {
            Lift.setPower(liftPower * liftSpeed);
        } else if (liftPower < 0 && BotLimit.getState()){
            Lift.setPower(liftPower * liftSpeed);
        } else {
            Lift.setPower(0);
        }

        // Limit swtich code for button

        if (liftPower > 0 && !TopLimit.getState()) {
            Lift.setPower(0);
        } else if (liftPower < 0 && BotButton.isPressed() ){
            Lift.setPower(0);
        } else {
            Lift.setPower(liftPower * liftSpeed);
        }




        // Set servo Position
        ClawPosition = Range.clip(ClawPosition, ClawMinRange, ClawMaxRange) ;
        Claw.setPosition(ClawPosition);


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", LeftBackPower, RightBackPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
