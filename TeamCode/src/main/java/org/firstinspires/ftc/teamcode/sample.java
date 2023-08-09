package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "encoderTest (Blocks to Java)")
public class sample extends LinearOpMode {

    private DcMotor test;
    private DcMotor leftb;

    int testPos;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        test = hardwareMap.get(DcMotor.class, "test");
        leftb = hardwareMap.get(DcMotor.class, "leftb");

        // Put initialization blocks here.
        test.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testPos = 0;
        waitForStart();
        drive(1000, 0.25);
        drive(300, 0.25);
    }

    /**
     * Describe this function...
     */
    private void drive(int testTarget, double speed) {
        testPos += testTarget;
        test.setTargetPosition(testPos);
        test.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        test.setPower(speed);
        while (opModeIsActive() && test.isBusy()) {
            idle();
        }
        //test.setPower(0);
        //test.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
