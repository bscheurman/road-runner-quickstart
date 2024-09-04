package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

public final class AutonomousTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0
                , 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                //drive.actionBuilder(beginPose)
                //            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                //        .splineTo(new Vector2d(0, 60), Math.PI)
                //        .build());
            drive.actionBuilder(beginPose)
                    .lineToXSplineHeading(11, Math.toRadians(0))
                    .setTangent(Math.toRadians(0))
                    //.waitSeconds(2)
                    .turn(Math.toRadians(45))
                    .waitSeconds(0.1)

                    .lineToYSplineHeading(36, Math.toRadians(180))
                    .setTangent(Math.toRadians(180))
                    .lineToXSplineHeading(-24, Math.toRadians(-90))
                    .setTangent(Math.toRadians(-90))
                    .lineToYSplineHeading(0, Math.toRadians(0))
                    .setTangent(Math.toRadians(0))
                    .lineToX(0)
                    //.lineToY(48)
                    //.setTangent(Math.toRadians(0))
                    //.lineToX(32)
                    //.strafeTo(new Vector2d(44.5, 30))
                    //.turn(Math.toRadians(180))
                    //.lineToX(47.5)
                    //.waitSeconds(3)
                    .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
