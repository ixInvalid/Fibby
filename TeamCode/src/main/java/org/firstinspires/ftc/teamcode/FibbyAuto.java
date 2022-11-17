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

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

//@Disable
@Autonomous(name="FibbyAuto", group="Auto", preselectTeleOp = "FibbyTeleOp")
public class FibbyAuto extends LinearOpMode
{
    public DcMotor  leftFront   = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightRear   = null;

    public DcMotor parallelEncoder;
    public DcMotor perpendicularEncoder;

    public DcMotor lift  = null;
    public DcMotor lift2 = null;

    public Servo grabber = null;

    DigitalChannel L_limit;
    DigitalChannel U_limit;

    private double leftFrontPower;
    private double leftRearPower;
    private double rightFrontPower;
    private double rightRearPower;

    private boolean rightSide;

    private       double corrHeading;
    private final double corrFactor = 0.03;
    private       double diffCorrection;

    double heading = 0;
    int tickToINCH = 1068; //reading of the encoder per inch of motion

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    int coneImage = 0;

    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

    private static final String[] LABELS =
            {
                    "1 Bolt",
                    "2 Bulb",
                    "3 Panel"
            };

    private static final String VUFORIA_KEY = "Ae6MFyH/////AAABmf06rZVcN0VqtDgoLd1KtAcowILYnLgT+SXkGwocAFEpSEmZiGI51vbPAL/QfAgSIhIpPrAK3Fl+vReEgcd1kU5az1pYTI01VqIV1+3ELPbCEcnNMdw3Rs7L8tMMsyfY2nebMlSFfgn6rkfJnnoQEnr4gCGHB1K/7VVZsFg7AlG4SPJ1bORRlhkFf0xIP28Tvr80YnC06hnsL2AQgOJtGrPv7HUO04hWxe9jLMdwHhmnBu/FZfovI9A6EjrzB72joGNTzuLA5R2bBGKW6AsrkKcgw1y50EFOqanZ19gWBX7bc1OExeaNeNIMaGzbMV8jwVahVndqS4EiLc9FuudY21tw4b4jupvhSYUiSGBMtLmh\";\n";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //----------------------------------------------------------------------------------------------------
    // Start of the Auto
    //----------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()
    {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        parallelEncoder      = hardwareMap.get(DcMotor.class, "parallelEncoder");
        perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpendicularEncoder");

        parallelEncoder.setDirection(DcMotor.Direction.REVERSE);
        perpendicularEncoder.setDirection(DcMotor.Direction.REVERSE);

        lift  = hardwareMap.get(DcMotor.class,"Lift");
        lift2 = hardwareMap.get(DcMotor.class,"Lift2");

        lift.setDirection(DcMotor.Direction.REVERSE);
        lift2.setDirection(DcMotor.Direction.REVERSE);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabber = hardwareMap.get(Servo .class,"Grabber");
        grabber.setPosition(0);

        L_limit = hardwareMap.get(DigitalChannel.class, "L_limit");
        U_limit = hardwareMap.get(DigitalChannel.class, "U_limit");

        L_limit.setMode(DigitalChannel.Mode.INPUT);
        U_limit.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters gyro_parameters = new BNO055IMU.Parameters();
        gyro_parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro_parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyro_parameters.loggingEnabled = true;
        gyro_parameters.loggingTag = "IMU";
        gyro_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyro_parameters);

        initVuforia();
        initTfod();

        if (tfod != null)
        {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }

        Boolean questionAnswered = false;

        telemetry.addLine("Left Side - □ | Right Side - O");
        telemetry.update();

        while (questionAnswered == false)
        {
            if (gamepad1.b || gamepad2.b) {
                rightSide = true;
                questionAnswered = true;
            }
            else if (gamepad1.x || gamepad2.x)
            {
                rightSide = false;
                questionAnswered = true;
            }
            else
            {
                questionAnswered = false;
            }
            if (questionAnswered == true)
            {
                telemetry.clear();
                telemetry.update();
            }
        }

        while (!isStarted()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                        if (recognition.getLabel() == "1 Bolt") {
                            coneImage = 1;
                        } else if (recognition.getLabel() == "2 Bulb") {
                            coneImage = 2;
                        } else if (recognition.getLabel() == "3 Panel") {
                            coneImage = 3;
                        } else {
                            coneImage = 0;
                        }
                    }

                    telemetry.update();
                }
            }
        }

        // turn off the camera
        if (tfod != null) {
            tfod.shutdown();
        }

        ConeDrop();

    }

    //----------------------------------------------------------------------------------------------------
    // Camera & Tensor Flow Initialization
    //----------------------------------------------------------------------------------------------------
    private void initVuforia()
    {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    //----------------------------------------------------------------------------------------------------
    // Get Gyro Angle
    //----------------------------------------------------------------------------------------------------
    private void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        heading = -angles.firstAngle;

        telemetry.addData("heading: ", "%s", heading);
        telemetry.update();
    }

    //----------------------------------------------------------------------------------------------------
    // "My Blocks"
    //----------------------------------------------------------------------------------------------------
    public void GyroDriveENC(double distance, double power, int course)
    {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while ((parallelEncoder.getCurrentPosition()/tickToINCH <= distance && distance > 0) || (distance <0 && parallelEncoder.getCurrentPosition()/tickToINCH >= distance) && (opModeIsActive()))
        {
            checkOrientation();

            // it will make input power always positive
            //power = Math.abs(power);

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * corrFactor);

            if (corrHeading < 0)
            {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power);
                rightRearPower = (power);
            }
            else
            {
                leftFrontPower = (power);
                leftRearPower = (power);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power - diffCorrection);
            }

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void GyroStrafeENC(double distance, double power, String direction, int course)
    {
        distance = Math.abs(distance);
        //direction = toString().toLowerCase(direction);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        while (Math.abs(perpendicularEncoder.getCurrentPosition()/tickToINCH) <= distance && (opModeIsActive()))
        {
            checkOrientation();

            // it will make input power always positive
            //power = Math.abs(power);

            corrHeading = course - heading;
            diffCorrection = Math.abs(corrHeading * corrFactor);

            if ((corrHeading < 0 && direction == "left") || (corrHeading > 0 && direction == "right"))
            {
                leftFrontPower = (power);
                leftRearPower = (power - diffCorrection);
                rightFrontPower = (power);
                rightRearPower = (power - diffCorrection);
            }
            else
            {
                leftFrontPower = (power - diffCorrection);
                leftRearPower = (power);
                rightFrontPower = (power - diffCorrection);
                rightRearPower = (power);
            }

            if(direction == "left")
            {
                leftFront.setPower(-leftFrontPower);
                leftRear.setPower(leftRearPower);
                rightFront.setPower(rightFrontPower);
                rightRear.setPower(-rightRearPower);
            }
            else if(direction == "right")
            {
                leftFront.setPower(leftFrontPower);
                leftRear.setPower(-leftRearPower);
                rightFront.setPower(-rightFrontPower);
                rightRear.setPower(rightRearPower);
            }
            else
            {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
            }
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    public void liftENC(double distance, double power)
    {
        if (distance > 1300)
            distance = 1300;
        while ((power > 0 && lift.getCurrentPosition() <= distance) || ((power < 0 && lift.getCurrentPosition() >= distance + 15 && L_limit.getState() == true)) && (opModeIsActive()))
        {
            lift.setPower(power);
            lift2.setPower(-power);
        }

        lift.setPower(0);
        lift2.setPower(0);
    }
    //----------------------------------------------------------------------------------------------------
    // Auto Runs
    //----------------------------------------------------------------------------------------------------
    public void Parking()
    {
        if (coneImage == 1)
        {
            telemetry.addData(">", "coneimage1");
            telemetry.update();
            //move left

            //move forward

        }
        else if (coneImage == 2)
        {
            telemetry.addData(">", "coneimage2");
            telemetry.update();
            //move forward


        }
        else
        {
            telemetry.addData(">", "coneimage3");
            telemetry.update();
            //move right

            //move forward

        }
    }

    public void ConeDrop()
    {
        String lSide = "left";
        String rSide = "right";

        if(!rightSide)
        {
            lSide = "right";
            rSide = "left";

        }
        else
        {

        }

        // close grabber
        grabber.setPosition(0.0765);
        sleep(1000);

        // lift the arm and move forward to push cone
        liftENC(75, 0.85);
        GyroDriveENC(45, 0.3, 0);

        sleep(1000);

        // lift up and strafe left to pole
        liftENC(1750, 0.85);
        GyroStrafeENC(10, 0.5, lSide,0);

        // move to pole forward
        GyroDriveENC(1, 0.5, 0);

        sleep(1000);

        // drop cone
        grabber.setPosition(0);

        sleep(1000);

        // back up and lower lift
        GyroDriveENC(-1, -0.25, 0);
        liftENC(0, -0.5);


        if (coneImage == 1)
        {
            telemetry.addData(">", "coneimage1");
            telemetry.update();
            sleep(1000);

            //strafe to zone 1
            if (rightSide)
            {
                GyroStrafeENC(11, 0.5, "left", 0);
            }
            else
            {
                GyroStrafeENC(36, 0.5, "left", 0);
            }
            GyroDriveENC(-10, -0.4, 0);
        }
        else if (coneImage == 2)
        {
            //strafe to zone
            GyroStrafeENC(11, 0.5, rSide, 0);
            GyroDriveENC(-10, -0.4, 0);
        }
        else
        {
            if (rightSide)
            {
                GyroStrafeENC(36, 0.5, "right", 0);
            }
            else
            {
                GyroStrafeENC(11, 0.5, "right", 0);
            }
            GyroDriveENC(-10, -0.4, 0);
        }
    }

}
