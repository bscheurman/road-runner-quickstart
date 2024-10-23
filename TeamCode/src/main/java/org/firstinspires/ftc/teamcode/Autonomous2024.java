package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.Claw;
import org.firstinspires.ftc.teamcode.actions.Harm;
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
        clawServo = hardwareMap.get(Servo.class, "claw1");
        Pose2d beginPose = new Pose2d(0
                , -60, 90);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
            Wrist wrist = new Wrist(hardwareMap);
            Claw claw = new Claw(hardwareMap);
            Harm harm = new Harm(hardwareMap);
            // Tilt the arm up for initialization. Tilt is an action, which will set the tilt servo
            Actions.runBlocking(wrist.wristDown());
            Actions.runBlocking(claw.clawClose());
            Actions.runBlocking(harm.runToPos(0));



            waitForStart();

            Actions.runBlocking(
                //drive.actionBuilder(beginPose)
                //            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                //        .splineTo(new Vector2d(0, 60), Math.PI)
                //        .build());
            drive.actionBuilder(beginPose)
                    .lineToY(-40)
                    .turn(Math.toRadians(90))
                    .lineToX(-54)
                    .turn(Math.toRadians(90))

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
