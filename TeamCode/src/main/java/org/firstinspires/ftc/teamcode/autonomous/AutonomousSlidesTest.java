package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AutonomousSlidesTest ", preselectTeleOp = "TeleOpFieldCentric")
public class AutonomousSlidesTest extends LinearOpMode {

    private DcMotor arm1;
    private DcMotor arm2;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        arm1 = hardwareMap.get(DcMotor.class, "Varm1");
        arm2 = hardwareMap.get(DcMotor.class, "Varm2");

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setPower(0.4);
        arm1.setDirection(DcMotor.Direction.REVERSE);

        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setPower(0.4);
        arm2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("arm", "0");
        arm1.setTargetPosition(0);
        arm2.setTargetPosition(0);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        arm1.setTargetPosition(500);
        arm2.setTargetPosition(500);

        sleep(100);
        telemetry.addData("arm1", arm1.getCurrentPosition());
        telemetry.addData("arm2", arm1.getCurrentPosition());
        sleep(500);
        telemetry.addData("arm1", arm1.getCurrentPosition());
        telemetry.addData("arm2", arm1.getCurrentPosition());
        sleep(4000);
        telemetry.addData("arm1", arm1.getCurrentPosition());
        telemetry.addData("arm2", arm1.getCurrentPosition());

        arm1.setTargetPosition(0);
        arm2.setTargetPosition(0);
        sleep(1000);

    }

}