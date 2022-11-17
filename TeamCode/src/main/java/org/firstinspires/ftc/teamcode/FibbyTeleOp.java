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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


// TODO: rename limit switch var and hardware mapping
// TODO: rename lift hardware mapping

// TODO: add gyro drive to TeleOp
// TODO: control scheme for drivers
// TODO: fix controls to make more sense or something new and better *cough* refer to game manual 0

//@Disabled
@TeleOp(name="FibbyTeleOp", group="TeleOp")
public class FibbyTeleOp extends OpMode
{
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;

    private DcMotor parallelEncoder;
    private DcMotor perpendicularEncoder;

    public Servo grabber = null;

    public DcMotor lift = null;
    public DcMotor lift2 = null;
    DigitalChannel L_limit;
    DigitalChannel U_limit;

    double poleShort = 567;
    double poleMid = 927;
    double poleHigh = 1290;
    double zero = 0;
    double tolerance = 5;
    char liftButton = 'n';


    //----------------------------------------------------------------------------------------------------
    // Initialization
    //----------------------------------------------------------------------------------------------------
    @Override
    public void init()
    {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");

        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift = hardwareMap.get(DcMotor.class,"Lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift2 = hardwareMap.get(DcMotor.class,"Lift2");
        lift2.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = hardwareMap.get(Servo.class,"Grabber");
        grabber.setPosition(0);

        L_limit = hardwareMap.get(DigitalChannel.class, "L_limit");
        U_limit = hardwareMap.get(DigitalChannel.class, "U_limit");

        L_limit.setMode(DigitalChannel.Mode.INPUT);
        U_limit.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData(">", "Robot Ready.  Press Play.");
    }

    //----------------------------------------------------------------------------------------------------
    // Initialization Loop
    //----------------------------------------------------------------------------------------------------
    @Override
    public void init_loop()
    {

    }

    //----------------------------------------------------------------------------------------------------
    // Start
    //----------------------------------------------------------------------------------------------------
    @Override
    public void start()
    {

    }

    //----------------------------------------------------------------------------------------------------
    // Loop
    //----------------------------------------------------------------------------------------------------
    @Override
    public void loop()
    {
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        double liftPower =0;


        telemetry.addData("LF, RF, Parallel, Perpendicular, Lift", "Starting at %7d :%7d :%7d :%7d :%7d ",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                parallelEncoder.getCurrentPosition(),
                perpendicularEncoder.getCurrentPosition(),
                lift.getCurrentPosition());
        telemetry.update();

        if (gamepad1.right_stick_y != 0) {
            leftFrontPower = -gamepad1.right_stick_y;
            rightFrontPower = -gamepad1.right_stick_y;
            leftRearPower = -gamepad1.right_stick_y;
            rightRearPower = -gamepad1.right_stick_y;
        } else if (gamepad1.left_stick_x != 0) {
            leftFrontPower = gamepad1.left_stick_x;
            rightFrontPower = -gamepad1.left_stick_x;
            leftRearPower = gamepad1.left_stick_x;
            rightRearPower = -gamepad1.left_stick_x;
        } else if (gamepad1.left_trigger != 0) { //if left trigger is being pressed it spinning left
            leftFrontPower = -gamepad1.left_trigger;
            rightFrontPower = gamepad1.left_trigger;
            leftRearPower = gamepad1.left_trigger;
            rightRearPower = -gamepad1.left_trigger;
        } else if (gamepad1.right_trigger != 0) {
            leftFrontPower = gamepad1.right_trigger;
            rightFrontPower = -gamepad1.right_trigger;
            leftRearPower = -gamepad1.right_trigger;
            rightRearPower = gamepad1.right_trigger;
        } else {
            leftFrontPower = 0;
            rightFrontPower = 0;
            leftRearPower = 0;
            rightRearPower = 0;
        }

        //Limit switch - false == pressed, true == not pressed
        //This if statement is broken into two chunks, if the driver is trying to lower the lift AND the lower limit is NOT pressed,
        // OOOOORRRRRR if the driver is trying to raise the lift and the upper limit switch is not pressed then run the lift motor, otherwise don't.
        if (gamepad2.a)
            liftButton = 'a';
        else if (gamepad2.b)
            liftButton = 'b';
        else if (gamepad2.x)
            liftButton = 'x';
        else if (gamepad2.y)
            liftButton = 'y';


        if ((gamepad2.right_stick_y > 0 && L_limit.getState() == true) || (gamepad2.right_stick_y < 0 && lift.getCurrentPosition() <= poleHigh+50))
        {

            if(gamepad2.right_stick_y < 0) {
                liftPower = gamepad2.right_stick_y;
            }
            else {
                liftPower = gamepad2.right_stick_y * .75;
            }
        }
       /* else {
            liftButton = 'n';
        }
*/

        if (gamepad2.right_stick_y != 0)
            liftButton = 'n';
        else
            if (liftButton == 'a') {
                if ((lift.getCurrentPosition() > zero + tolerance) && (L_limit.getState() == true))
                    liftPower = 0.5;
                else
                    liftButton = 'n';
            } else if (liftButton == 'b') {
                if ((lift.getCurrentPosition() > poleShort + tolerance) && (L_limit.getState() == true))
                    liftPower = 0.5;
                else if (lift.getCurrentPosition() < poleShort - tolerance)
                    liftPower = -0.75;
                else
                    liftButton = 'n';
            } else if (liftButton == 'x') {
                if ((lift.getCurrentPosition() > poleMid + tolerance) && (L_limit.getState() == true))
                    liftPower = 0.5;
                else if (lift.getCurrentPosition() < poleMid - tolerance)
                    liftPower = -0.75;
                else
                    liftButton = 'n';
            } else if (liftButton == 'y') {
                if ((lift.getCurrentPosition() > poleHigh + tolerance) && (L_limit.getState() == true))
                    liftPower = 0.5;
                else if (lift.getCurrentPosition() < poleHigh - tolerance)
                    liftPower = -0.75;
                else
                    liftButton = 'n';
            } else
                liftPower = 0;
















        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);

        leftRear.setPower(-leftRearPower);
        rightRear.setPower(-rightRearPower);

        lift.setPower(-liftPower);
        lift2.setPower(liftPower);

        //telemetry.addData("Servo Pos", grabber.getPosition());
        if (gamepad2.left_bumper){
            grabber.setPosition(0);
        }
        else if (gamepad2.right_bumper){
            grabber.setPosition(0.0765);
        }
    }

    @Override
    public void stop()
    {

    }
}
