package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public final class AutonomousTfodTest extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0
                , 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            // make a Tilt instance for the servo that tilts the arm
            Tilt tilt = new Tilt(hardwareMap);
            // Tilt the arm up for initialization. Tilt is an action, which will set the tilt servo
            Actions.runBlocking(tilt.tiltUp());

            // Last years code to initialize the 2 cameras
            initDoubleVision();
            // ObjectScanner for camera to scan for horse, based on our ScanObjects block
            ObjectScanner obScanner = new ObjectScanner(hardwareMap);

            // Give ... Option for Camera Stream in the menu
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();

            // As always, waitForStart()
            waitForStart();

            // Drive forward and around, then Tilt arm Forward, then scan for horse in position 2
            Actions.runBlocking(
                    // robot does list of actions sequentially (means one after the other)
                    new SequentialAction(
                            // beginPose = x:0,y:0,heading:0
                            // this means you placed the robot in the center facing north
                            // (north is considered toward the scoreboards)
                            drive.actionBuilder(beginPose)
                                    // travel in starting direction (north) until center of robot crosses X=11 inches and keep facing north
                                    .lineToXSplineHeading(22, Math.toRadians(0))
                                    // set travel direction north (might not need this)
                                    .setTangent(Math.toRadians(0))
                                    // turn 45 degrees to the left
                                    .turn(Math.toRadians(45))
                                    .waitSeconds(0.1)
                                    // travel on the 45 degree angle diagonally until center of robot crosses Y=36
                                    // SplineHeading 180 causes it to smoothly rotate to face 180 South while traveling diagonally north
                                    .lineToYSplineHeading(36, Math.toRadians(180))
                                    .setTangent(Math.toRadians(180))
                                    .lineToX (42)
                                    .build(),
                                // Tilt arm forward after driving is finished
                                tilt.tiltForward()
                                // Scan for horse object in position 2 (center spike mark)

                    )
            );

            Actions.runBlocking(obScanner.scanObject(2));

            // Last years code to scan for the horse if it is in position 2 (center position)
            //scanObjects(2);

            // Turn 45 right and scan for a horse in position 1
            if (obScanner.spikeSelection == 0) {
                Actions.runBlocking( new SequentialAction(
                        drive.actionBuilder(drive.pose).turn (Math.toRadians(-45)).build())
                );
                Actions.runBlocking(obScanner.scanObject(1));
                obScanner.spikeSelection = 1;
            }

            // Turn 90 degrees left and then scan for a horse in position 3
            if (obScanner.spikeSelection == 0) {
                Actions.runBlocking( new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .turn (Math.toRadians(90))
                                .build(),
                        obScanner.scanObject(3))
                );
                //scanObjects(3);
            }

            Actions.runBlocking(
                    new SequentialAction(
                            tilt.tiltUp(),
                            drive.actionBuilder(drive.pose)
                                    // face south
                                    .turnTo(Math.toRadians(180))
                                    // set travel direction north
                                    .setTangent(Math.toRadians(0))
                                    // travel north until X=50
                                    .lineToX(50)
                                    // set travel direction east
                                    .setTangent(Math.toRadians(-90))
                                    // travel east until y=0 while rotating to face 0 degrees north
                                    .lineToYSplineHeading(0, Math.toRadians(0))
                                    // set travel direction north
                                    .setTangent(Math.toRadians(0))
                                    // travel backwards to X = 0, to end up back at the start 0,0
                                    .lineToX(0)
                                    .build()

                    )
            );

        } else {
            throw new RuntimeException();
        }
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
    private void scanObjects(int forSpike) {
        time2 = 6000;
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
            if (bestObjectRecognition > 0.7) {
                spikeSelection = forSpike;
                break;
            }
        }

    }

    // Arm Tilt class
    public class Tilt {
        private Servo tilt;

        public Tilt(HardwareMap hardwareMap) {
            tilt = hardwareMap.get(Servo.class, "tilt");
        }

        // within the Claw class
        public class TiltUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                tilt.setPosition(0.35);
                return false;
            }
        }
        public Action tiltUp() {
            return new TiltUp();
        }

        public class TiltForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                tilt.setPosition(0.55);
                return false;
            }
        }
        public Action tiltForward() {
            return new TiltForward();
        }
    }

    // Arm Tilt class
    public class ObjectScanner {
        private int timeout = 2000;
        public int spikeSelection = 0;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        long startTime = System.currentTimeMillis();
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.

        public ObjectScanner(HardwareMap hardwareMap) {
            //tilt = hardwareMap.get(Servo.class, "tilt");
        }

        // within the Claw class
        public class ScanObject implements Action {
            int number;
            ScanObject (int num){
                number = num;
                startTime = System.currentTimeMillis();
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Repeat Camera scans for the amount of time requested
                telemetryCameraSwitching();
                // Push telemetry to the Driver Station.
                telemetry.update();

                if (bestObjectRecognition > 0.7) {
                    spikeSelection = number;
                    return false; // return false when finished
                }
                if (startTime < System.currentTimeMillis() - timeout)
                    return false; // return false to timeout and continue
                return true;  // return true to keep scanning
            }
        }
        public Action scanObject(int number) {
            return new ScanObject(number);
        }

    }
}


