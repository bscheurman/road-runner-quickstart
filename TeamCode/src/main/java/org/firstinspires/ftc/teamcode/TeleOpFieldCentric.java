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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.Claw;
import org.firstinspires.ftc.teamcode.actions.Harm;
import org.firstinspires.ftc.teamcode.actions.Slide;
import org.firstinspires.ftc.teamcode.actions.Wrist;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

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
public class TeleOpFieldCentric extends OpMode {

    private double speed = 1.0;


    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private  MecanumDrive drive;
    private  Claw claw;

    private  Harm harm;

    private Slide slide;

    private double OldposR;

    private boolean started = false;

    private Wrist wrist;

    boolean blue = false;
    boolean fieldCentric = true;

    @Override
    public void init() {

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        Pose2d beginPose = new Pose2d(0
                , -61, Math.toRadians(0));
        // Initialize SampleMecanumDrive
        drive = new MecanumDrive(hardwareMap,beginPose );
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "harm");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist");
        Servo clawServo = hardwareMap.get(Servo.class, "claw");


        wrist = new Wrist(hardwareMap);
        claw = new Claw(hardwareMap);
        harm = new Harm(hardwareMap);
        slide = new Slide(hardwareMap);
        // Tilt the arm up for initialization. Tilt is an action, which will set the tilt servo
        Actions.runBlocking(wrist.wristBack());
        Actions.runBlocking(claw.clawClose());
        Actions.runBlocking(slide.slideTo0());


        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // at the start, extend the wrist. (but it might be better to keep it back)
        if (!started){
            Actions.runBlocking(wrist.wristUp());
            started = true;
        }

        // allow to switch between field centric using the dpad
        if (gamepad1.dpad_up) {
            fieldCentric = true;
        } else if (gamepad1.dpad_down) {
            fieldCentric = false;
        }

        if (gamepad1.right_bumper) {
            speed = 1.0;
        }
        else {
            speed = 0.7;
        }

        if (gamepad1.left_bumper) {
            speed = 0.4;
        }
        else {
            speed = 0.7;
        }

        // Create a vector from the gamepad x/y inputs
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * speed,
                -gamepad1.left_stick_x * speed
        );

        // Then, rotate that vector by the inverse of that heading
        if (fieldCentric) {
            if (blue){
                input = drive.pose.heading.plus(Math.PI).inverse().times(
                        new Vector2d(input.x, input.y));
            } else {
                // you might need to inverse the heading or not
                input = drive.pose.heading.inverse().times(
                        new Vector2d(input.x, input.y));
            }
        }

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setDrivePowers(
                new PoseVelocity2d(
                        input,
                        -gamepad1.right_stick_x
                )
        );

        //telemetry.addData("x", poseEstimate.position.x);
        //telemetry.addData("y", poseEstimate.position.y);
        //telemetry.addData("heading", poseEstimate.heading);
        //telemetry.update();

        // get the current pose each time
        drive.updatePoseEstimate();

        // updated based on gamepads
        if (gamepad2.left_trigger > 0.8) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    claw.clawOpen()
            ));
        }

        if (gamepad2.left_trigger < 0.1) {
            runningActions.add(new SequentialAction(
                    new SleepAction(0.5),
                    claw.clawClose()
            ));
        }

        if (gamepad2.right_stick_x >= 0 && gamepad2.right_stick_x != OldposR){
            runningActions.add(new SequentialAction(
//                    harm.runToPos((int) (gamepad2.right_stick_x*1000))
                    slide.slideToPos((int) (gamepad2.right_stick_x*1000))

            ));

            OldposR = (gamepad2.right_stick_x);
        }

        if (gamepad2.left_bumper) {
            packet.addLine("wrist up");
            runningActions.add(new SequentialAction(

                    wrist.wristUp()
            ));
        }

        if (gamepad2.right_bumper) {
            runningActions.add(new SequentialAction(

                    wrist.wristDown()
            ));
        }


        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }

}