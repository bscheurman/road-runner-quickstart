package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.Claw;
import org.firstinspires.ftc.teamcode.actions.Harm;
import org.firstinspires.ftc.teamcode.actions.Slide;
import org.firstinspires.ftc.teamcode.actions.Wrist;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Config
@Autonomous(name = "Autonomous2024", group = "Autonomous", preselectTeleOp = "TeleOpFieldCentric")
public final class Autonomous2024 extends LinearOpMode {
    private DcMotor harmMotor;
    private Servo wristServo;
    private Servo clawServo;
    @Override
    public void runOpMode() throws InterruptedException {
        harmMotor = hardwareMap.get(DcMotor.class, "harm");
        wristServo = hardwareMap.get(Servo.class, "wrist");
        clawServo = hardwareMap.get(Servo.class, "claw");
        Pose2d beginPose = new Pose2d(0
                , -61,  Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Wrist wrist = new Wrist(hardwareMap);
            Claw claw = new Claw(hardwareMap);
            Slide harm = new Slide(hardwareMap,"harm");
            Slide slide1 = new Slide(hardwareMap,"Varm1");
            Slide slide2 = new Slide(hardwareMap,"Varm2");
            // Tilt the arm up for initialization. Tilt is an action, which will set the tilt servo
            Actions.runBlocking(wrist.wristBack());
            Actions.runBlocking(claw.clawClose());
//            Actions.runBlocking(
//                    new ParallelAction(
//                            harm.slideTo0(),
//                            slide1.slideTo0(),
//                            slide2.slideTo0()
//                    ));



            waitForStart();



            Actions.runBlocking(
                     new SequentialAction(
                                    drive.actionBuilder(beginPose)
                                            .setTangent(Math.toRadians(90))
                                            .lineToYSplineHeading(-44, Math.toRadians(90))

                                            .build(),
                                    wrist.wristUp(),
                                     new SleepAction(0.5)));

            Actions.runBlocking(
                    new ParallelAction(

                            slide1.slideToPos(500),
                            slide2.slideToPos(500),
                            harm.slideToPos(200),
                            wrist.wristUp()));
            Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(1.0),
                    drive.actionBuilder(beginPose)
                            .setTangent(Math.toRadians(90))
                            .lineToYSplineHeading(-42, Math.toRadians(90))

                            .build(),
                    wrist.wristMoveToPos(0.82),
                   claw.clawOpen()));

            Actions.runBlocking(
                    new SequentialAction(

                            new SleepAction(2.5),
                            drive.actionBuilder(beginPose)
                                    .setTangent(Math.toRadians(90))
                                    .lineToYSplineHeading(-42, Math.toRadians(90))
                                    .build(),
                            claw.clawClose(),
                            wrist.wristBack()));


                                    // face south
                //drive.actionBuilder(beginPose)
                //            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                //        .splineTo(new Vector2d(0, 60), Math.PI)
                //        .build());


        } else {
            throw new RuntimeException();
        }
    }
}
