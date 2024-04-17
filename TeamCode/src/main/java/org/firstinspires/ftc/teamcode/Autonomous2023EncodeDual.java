package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "Autonomous2023EncoderDual2 (Blocks to Java)", preselectTeleOp = "Gobilda 2")
public class Autonomous2023EncodeDual extends LinearOpMode {

    private Servo tilt;
    private DcMotor arm;
    private DcMotor backleft;
    private DcMotor backright;
    private DcMotor frontleft;
    private DcMotor frontright;
    private Servo wrist;
    private Servo claw1;

    float bestObjectRecognition;
    boolean blue;
    VisionPortal myVisionPortal;
    int time2;
    TfodProcessor myTfodProcessor;
    long startTime;
    AprilTagProcessor myAprilTagProcessor;
    int spikeSelection;
    int april_counter;
    double aprilDistanceX;
    double aprilDistance;
    int AprilTag1;
    int AprilTag2;
    double AprilAngle;

    /**
     * Describe this function...
     */
    private void backward(int distance, double speed) {
        Run_Wheels(distance * -17.8, distance * -17.8, distance * -17.8, distance * -17.8);
    }

    /**
     * Describe this function...
     */
    private void forward(int distance, double speed) {
        Run_Wheels(distance * 17.8, distance * 17.8, distance * 17.8, distance * 17.8);
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initDoubleVision() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.
        telemetry.addLine("init  double vision start");
        telemetry.update();
        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Set the detector decimation.
        myAprilTagProcessor.setDecimation(1);
        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("model_20231207_redblue.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("blue_bronco", "red_bronco"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(4 / 3);
        // Next, create a TfodProcessor.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(ClassFactory.createSwitchableCameraNameForAllWebcams(hardwareMap));
        // Set the camera resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Enable the live camera preview.
        myVisionPortalBuilder.enableLiveView(false);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
        // at first, it will enable the Tensorflow processor and disable the April Tag processor until later
        myVisionPortal.setProcessorEnabled(myTfodProcessor, true);
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, false);
        telemetry.addLine("init  double vision complete");
        telemetry.update();
    }

    /**
     * Display info (using telemetry) for a detected object
     */
    private void telemetryTfod2() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        bestObjectRecognition = 0;
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
            // Switch Cameras and use April Tags Once the Object Confidence is great enough
            if (myTfodRecognition.getConfidence() > bestObjectRecognition && spikeSelection == 0) {
                bestObjectRecognition = myTfodRecognition.getConfidence();
                if (myTfodRecognition.getLabel().equals("blue_bronco")) {
                    blue = true;
                } else {
                    blue = false;
                }
            }
        }
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(DcMotor.class, "arm");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw1 = hardwareMap.get(Servo.class, "claw1");

        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection, using
        // two webcams.
        initDoubleVision();
        moveWrist(true);
        moveClaw(false);
        tilt.setPosition(0.2);
        april_counter = 0;
        blue = false;
        aprilDistance = 54;
        aprilDistanceX = 0;
        // Wait for the match to begin.
        // Position of the Spike mark with the randomized game element
        spikeSelection = 0;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(1);
        arm.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        backleft.setTargetPosition(0);
        backright.setTargetPosition(0);
        frontleft.setTargetPosition(0);
        frontright.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.FORWARD);
        telemetry.speak("Ready", "en", "US");
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        // Start by moving the robot up near the center spike mark and look down for a pixel
        forward(36, 0.4);
        moveTilt(false);
        scanObjects(2);
        // Turn left and look for a pixel
        if (spikeSelection == 0) {
            turn(-25, 0.5);
            sleep(20);
            moveTilt(true);
            sleep(20);
            scanObjects(1);
        }
        if (spikeSelection == 0) {
            turn(60, 0.5);
            sleep(100);
            moveTilt(true);
            sleep(100);
            scanObjects(3);
        }
        // If it wasnt in center or left, then it must be on the right spike
        if (spikeSelection == 0) {
            requestOpModeStop();
        }
        // Switch to the lower camera. Done scanning for pixels, now scan for April Tags
        if (spikeSelection == 1) {
            moveTilt(false);
            forward(5, 0.4);
            turn(-22, 0.4);
            moveWrist(false);
            moveClaw(true);
            sleep(300);
            wrist.setPosition(0.6);
            sleep(500);
            moveClaw(false);
            sleep(100);
            moveWrist(true);
            strafe(10, 1);
            if (blue) {
                turn(-40, 1);
                strafe(-15, 1);
            } else {
                turn(140, 0.4);
                strafe(-30, 1);
            }
            sleep(100);
            AprilTag1 = 1;
            AprilTag2 = 4;
        } else if (spikeSelection == 2) {
            turn(17, 0.4);
            sleep(200);
            forward(20, 0.4);
            moveTilt(false);
            moveWrist(false);
            sleep(200);
            moveClaw(true);
            sleep(300);
            wrist.setPosition(0.6);
            moveClaw(false);
            sleep(100);
            moveWrist(true);
            if (blue) {
                turn(-105, 0.4);
            } else {
                turn(74, 0.4);
            }
            AprilTag1 = 2;
            AprilTag2 = 5;
        } else if (spikeSelection == 3) {
            if (blue) {
                strafe(-10, 1);
            }
            if (blue) {
                turn(25, 0.4);
                forward(10, 0.4);
            } else {
                turn(20, 0.4);
            }
            sleep(1);
            moveTilt(false);
            sleep(400);
            moveWrist(false);
            sleep(100);
            moveClaw(true);
            wrist.setPosition(0.6);
            moveClaw(false);
            sleep(400);
            moveWrist(true);
            if (blue) {
                strafe(-10, 1);
            }
            if (blue) {
                turn(-150, 0.4);
                strafe(30, 1);
            } else {
                turn(35, 0.4);
                strafe(10, 1);
            }
            AprilTag1 = 3;
            AprilTag2 = 6;
        }
        sleep(100);
        // Switch to the lower camera. Done scanning for pixels, now scan for April Tags
        tilt.setPosition(0.35);
        switchCameras();
        sleep(5);
        scanApril();
        wrist.setPosition(0.5);
        wrist.setPosition(0.22);
        forward((int) (aprilDistance * 1.75), 0.2);
        strafe((int) (aprilDistanceX * 2.8 + 18), 0.8);
        arm.setTargetPosition(550);
        moveTilt(true);
        wrist.setPosition(0.4);
        sleep(500);
        moveClaw(true);
        moveWrist(true);
        sleep(200);
        arm.setTargetPosition(0);
        moveTilt(false);
        sleep(200);
        if (blue) {
            strafe(spikeSelection * -15 + -45, 0.8);
        } else {
            strafe((4 - spikeSelection) * 15 + 35, 0.8);
        }
    }

    /**
     * Describe this function...
     */
    private void turn(int angle, double speed) {
        Run_Wheels(12 * angle, -12 * angle, 12 * angle, -12 * angle);
    }

    /**
     * Describe this function...
     */
    private void scanObjects(int forSpike) {
        time2 = 1000;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        startTime = System.currentTimeMillis();
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        while (opModeIsActive() && System.currentTimeMillis() < startTime + time2) {
            // Repeat Camera scans for the amount of time requested
            telemetryCameraSwitching();
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
        if (bestObjectRecognition > 0.7) {
            spikeSelection = forSpike;
        }
    }

    /**
     * Describe this function...
     */
    private void test_wheels() {
        frontleft.setPower(0.4);
        sleep(1000);
        frontleft.setPower(0);
        frontright.setPower(0.4);
        sleep(1000);
        frontright.setPower(0);
        backright.setPower(0.4);
        sleep(1000);
        frontright.setPower(0);
        backleft.setPower(0.4);
        sleep(1000);
        backleft.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void switchCameras() {
        // After Object Recognition succeeds, Switches to Camera 2 and starts April Tag detection
        myVisionPortal.setActiveCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);
        // Enable or disable the TensorFlow Object Detection processor.
        myVisionPortal.setProcessorEnabled(myTfodProcessor, false);
    }

    /**
     * Describe this function...
     */
    private void scanApril() {
        time2 = 1000;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        startTime = System.currentTimeMillis();
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        while (opModeIsActive() && System.currentTimeMillis() < startTime + time2) {
            // Repeat Camera scans for the amount of time requested
            telemetryCameraSwitching();
            // Push telemetry to the Driver Station.
            telemetry.update();
            // Share the CPU.
            sleep(20);
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        telemetry.addData("Spike Detected", spikeSelection);
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                if (myAprilTagDetection.id == AprilTag1 || myAprilTagDetection.id == AprilTag2) {
                    aprilDistance = myAprilTagDetection.ftcPose.y;
                    telemetry.addData("aprilDistance", aprilDistance);
                    aprilDistanceX = myAprilTagDetection.ftcPose.x;
                    telemetry.addData("aprilDistanceX", aprilDistanceX);
                    AprilAngle = myAprilTagDetection.ftcPose.yaw;
                    telemetry.addData("aprilangle", AprilAngle);
                }
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
    }

    /**
     * Describe this function...
     */
    private void telemetryCameraSwitching() {
        // Get the active camera and compare it to the webcam named Webcam 1.
        if (myVisionPortal.getActiveCamera().equals(hardwareMap.get(WebcamName.class, "Webcam 1"))) {
            telemetry.addData("activeCamera", "Webcam 1");
            telemetry.addLine("Searching for Pixel on Spike Mark");
        } else {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.addLine("Searching for April Tags");
        }
        if (myVisionPortal.getProcessorEnabled(myAprilTagProcessor)) {
            telemetryAprilTag();
        } else {
            telemetry.addLine("Dpad Right to enable AprilTag");
        }
        if (myVisionPortal.getProcessorEnabled(myTfodProcessor)) {
            telemetry.addLine("Dpad Down to disable TFOD");
            telemetry.addLine("");
            telemetryTfod2();
        } else {
            telemetry.addLine("Dpad Up to enable TFOD");
        }
    }

    /**
     * Describe this function...
     */
    private void Run_Wheels(double FL, double FR, double BL, double BR) {
        frontleft.setTargetPosition((int) FL);
        frontright.setTargetPosition((int) FR);
        backleft.setTargetPosition((int) BL);
        backright.setTargetPosition((int) BR);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) frontleft).setVelocity(1750);
        ((DcMotorEx) frontright).setVelocity(1750);
        ((DcMotorEx) backleft).setVelocity(1750);
        ((DcMotorEx) backright).setVelocity(1750);
        while (opModeIsActive() && frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            telemetry.addData("position", frontleft.getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
        sleep(200);
        ((DcMotorEx) frontleft).setVelocity(0);
        ((DcMotorEx) frontright).setVelocity(0);
        ((DcMotorEx) frontleft).setVelocity(0);
        ((DcMotorEx) frontright).setVelocity(0);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void strafe(int distance, double speed) {
        Run_Wheels(distance * 17.8, distance * -17.8, distance * -17.8, distance * 17.8);
    }

    /**
     * Describe this function...
     */
    private void moveTilt(boolean forward2) {
        if (forward2) {
            tilt.setPosition(0.8);
        } else {
            tilt.setPosition(0.4);
        }
        sleep(400);
    }

    /**
     * Describe this function...
     */
    private void moveClaw(boolean open) {
        if (open) {
            claw1.setPosition(0.2);
        } else {
            claw1.setPosition(0.42);
        }
        sleep(300);
    }

    /**
     * Describe this function...
     */
    private void drivetoapriltag() {
        ((DcMotorEx) frontleft).setVelocity(500);
        ((DcMotorEx) frontright).setVelocity(500);
        ((DcMotorEx) backleft).setVelocity(500);
        ((DcMotorEx) backright).setVelocity(500);
        while (aprilDistance > 25 && opModeIsActive() && april_counter < 8) {
            telemetryCameraSwitching();
            // Push telemetry to the Driver Station.
            telemetry.update();
            sleep(500);
            april_counter += 1;
        }
        ((DcMotorEx) frontleft).setVelocity(0);
        ((DcMotorEx) frontright).setVelocity(0);
        ((DcMotorEx) backleft).setVelocity(0);
        ((DcMotorEx) backright).setVelocity(0);
    }

    /**
     * Describe this function...
     */
    private void rotatetoapriltag() {
        frontleft.setPower(0.1);
        frontright.setPower(-0.1);
        backleft.setPower(0.1);
        backright.setPower(-0.1);
        while (aprilDistanceX < 10 && opModeIsActive()) {
            telemetryCameraSwitching();
            // Push telemetry to the Driver Station.
            telemetry.update();
            sleep(200);
        }
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void moveWrist(boolean up) {
        if (up) {
            wrist.setPosition(0.22);
        } else {
            wrist.setPosition(0.64);
        }
        sleep(300);
    }
}