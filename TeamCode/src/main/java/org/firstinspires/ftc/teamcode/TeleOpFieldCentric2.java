package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.actions.Claw;
import org.firstinspires.ftc.teamcode.actions.Harm;
import org.firstinspires.ftc.teamcode.actions.Wrist;

import java.util.ArrayList;
import java.util.List;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
public class TeleOpFieldCentric2 extends OpMode {

    // Declare a PIDF Controller to regulate heading
    private final PIDFController.PIDCoefficients HEADING_PID = new PIDFController.PIDCoefficients(0.25, 0.0, 0.0);
    private final PIDFController headingController = new PIDFController(HEADING_PID);
    double speed;
    LynxModule CONTROL_HUB;
    LynxModule EXPANSION_HUB;
    boolean fieldCentric = true;
    boolean blue = false;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private  MecanumDrive drive;
    private  Claw claw;

    private  Harm harm;

    private int OldposR;

    private boolean started = false;

    private Wrist wrist;

    @Override
    public void init() {

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        Pose2d beginPose = new Pose2d(0
                , 0, Math.toRadians(90));
        // Initialize SampleMecanumDrive
        //drive = new MecanumDrive(hardwareMap,beginPose );
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        DcMotor harmMotor = hardwareMap.get(DcMotor.class, "harm");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        Servo clawServo = hardwareMap.get(Servo.class, "claw");


        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        harm = new Harm(hardwareMap);
        // Tilt the arm up for initialization. Tilt is an action, which will set the tilt servo
        Actions.runBlocking(wrist.wristBack());
        Actions.runBlocking(claw.clawClose());
        Actions.runBlocking(harm.runToPos(0));


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CONTROL_HUB = allHubs.get(0);
        EXPANSION_HUB = allHubs.get(1);
        // RoadRunner Init

        headingController.setInputBounds(-Math.PI, Math.PI);

        // Telemetry Init
        telemetry.setMsTransmissionInterval(50);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motor Init

    }

    @Override
    public void loop() {
        double prevTime = System.nanoTime(); // set to current time
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        if (!started){
            Actions.runBlocking(wrist.wristUp());
            started = true;
        }

        // update running actions

        // Update the speed
        if (gamepad1.left_bumper) {
            speed = .35;
        } else if (gamepad1.right_bumper) {
            speed = 1;
        } else {
            speed = .8;
        }

        if (gamepad1.dpad_left) {
            drive.pose = new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(-90.0));
        }
        if (gamepad1.dpad_up) {
            fieldCentric = true;
        } else if (gamepad1.dpad_down) {
            fieldCentric = false;

        }

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed
        );
        Pose2d poseEstimate = drive.pose;
        if (fieldCentric) {
            if (blue){
                input = drive.pose.heading.plus(Math.PI).inverse().times(
                        new Vector2d(-input.x, input.y));
            } else {
                input = drive.pose.heading.inverse().times(
                        new Vector2d(-input.x, input.y));
            }
            //double rotationAmount = -poseEstimate.heading.log() + Math.toRadians(90.0); // Rotation2d.log() makes it into a double in radians. SUPER useful, BAD name.
            //input = new Vector2d(input.x * Math.cos(rotationAmount) - input.y * Math.sin(rotationAmount), input.x * Math.sin(rotationAmount) + input.y * Math.cos(rotationAmount));
        }

        Vector2d controllerHeading = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);

        if (controllerHeading.minus(new Vector2d(0.0,0.0)).norm() < 0.7) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    input.x,
                                    input.y
                            ),
                            (gamepad1.left_trigger - gamepad1.right_trigger)
                    )
            );
        } else {
            // Set the target heading for the heading controller to our desired angle


            headingController.targetPosition = controllerHeading.angleCast().log() + Math.toRadians(180);


            // Set desired angular velocity to the heading controller output + angular
            // velocity feedforward
            double headingInput = (headingController.update(poseEstimate.heading.log())
                    * MecanumDrive.PARAMS.kV
                    * MecanumDrive.PARAMS.trackWidthTicks * MecanumDrive.PARAMS.inPerTick);
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    input.x,
                                    input.y
                            ),
                            headingInput
                    )
            );

        }
        if (gamepad1.right_stick_button) {
            //headingController.targetPosition = drive.getExternalHeading() + poleDetectionPipeline.getMaxRect().x

            // Set desired angular velocity to the heading controller output + angular
            // velocity feedforward
            double headingInput = (headingController.update(poseEstimate.heading.log())
                    * MecanumDrive.PARAMS.kV)
                    * MecanumDrive.PARAMS.trackWidthTicks;
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    input.x,
                                    input.y
                            ),
                            headingInput
                    )
            );
        }

        telemetry.addData("x", poseEstimate.position.x);
        telemetry.addData("y", poseEstimate.position.y);
        telemetry.addData("heading", poseEstimate.heading);
        telemetry.update();

        // Update everything. Odometry. Etc.
        //drive.update();

        if (gamepad2.left_trigger == 1) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    new InstantAction(() -> claw.clawOpen())
            ));
        }

        if (gamepad2.left_trigger == 0) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    new InstantAction(() -> claw.clawClose())
            ));
        }

        if (gamepad2.right_stick_x >= 0 && gamepad2.right_stick_x != OldposR){
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> harm.runToPos((int) (gamepad2.right_stick_x*1000)))

            ));

            OldposR = (int) (gamepad2.right_stick_x*1000);
        }

        if (gamepad2.left_bumper) {
            packet.addLine("wrist up");
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    new InstantAction(() -> wrist.wristUp())
            ));
        }

        if (gamepad2.right_bumper) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    new InstantAction(() -> wrist.wristDown())
            ));
        }



        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);


        gamepad1.rumble(CONTROL_HUB.getCurrent(CurrentUnit.AMPS),EXPANSION_HUB.getCurrent(CurrentUnit.AMPS), Gamepad.RUMBLE_DURATION_CONTINUOUS);
        drive.updatePoseEstimate();

        // Timing
        // measure difference between current time and previous time
        double timeDifference = (System.nanoTime() - prevTime) / 1000000.0;

        //MecanumDrive.dr(packet.fieldOverlay(), drive.pose); //new Pose2d(new Vector2d(IN_PER_TICK * drive.pose.trans.x,IN_PER_TICK * drive.pose.trans.y), drive.pose.rot)

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.position.x);
        telemetry.addData("y", poseEstimate.position.y);
        telemetry.addData("heading", poseEstimate.heading.log());
        telemetry.addData("controllerHeading", controllerHeading.angleCast().log());
        telemetry.addData("loopTimeMs", timeDifference);
        telemetry.addData("loopTimeHz", 1000.0 / timeDifference);
        telemetry.update();
    }

}