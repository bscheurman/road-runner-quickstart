package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Gobilda22 (Blocks to Java)", group = "sleep")
public class GobildaDriver extends LinearOpMode {

    private Servo airplane_servo;
    private DcMotor arm;
    private DcMotor winch;
    private Servo tilt;
    private Servo wrist;
    private Servo claw1;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor backright;

    float RightX;

    int ArmPowerCounter;
    int MaxSpeed;
    int armpos;
    int Winchpos;
    double wristpos;
    double Speed_X;
    double claw_grip;
    int WinchPowerCounter;

    /**
     * Describe this function...
     */
    private void airplane_servo2() {
        // On Gamepad X button press, function turns servo 90 degrees to release elastic for airplane launch.  Needs to run inside repeat forever loop
        if (gamepad1.x) {
            airplane_servo.setPosition(0.2);
            telemetry.addData("Driver X-btn", "pressed");
        } else {
            airplane_servo.setPosition(0.5);
            telemetry.addData("Driver X-btn", "---");
        }
    }

    /**
     * Describe this function...
     */
    private void armcontrol() {
        float LeftY2;

        arm.setTargetPosition(armpos);
        LeftY2 = gamepad2.left_stick_y;
        if (LeftY2 < -0.2 && armpos < 1300) {
            armpos += 50;
            Winchpos += 100;
        }
        if (LeftY2 > 0.2 && armpos > 0) {
            armpos += -50;
            Winchpos += -100;
        }
        if (LeftY2 != 0) {
            WinchPowerCounter = 100;
            ArmPowerCounter = 100;
            arm.setPower(1);
            winch.setPower(1);
            arm.setTargetPosition(armpos);
        }
        if (ArmPowerCounter > 0) {
            ArmPowerCounter += -1;
        } else if (ArmPowerCounter == 0 && 0 == armpos) {
            arm.setPower(0);
        }
        if (gamepad2.x) {
            tilt.setPosition(0.6);
        }
        if (gamepad2.y) {
            tilt.setPosition(0.1);
        }
        telemetry.addData("armpos", Double.parseDouble(JavaUtil.formatNumber(armpos, 2)));
        telemetry.addData("wristpos", Double.parseDouble(JavaUtil.formatNumber(wristpos, 2)));
        telemetry.addData("LeftY2", Double.parseDouble(JavaUtil.formatNumber(LeftY2, 2)));
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void claw_control() {
        float RightY2;

        RightY2 = gamepad2.right_stick_y;
        wrist.setPosition(wristpos);
        if (gamepad2.a) {
            claw1.setPosition(0);
        }
        if (gamepad2.b) {
            claw1.setPosition(claw_grip);
        }
        if (gamepad2.right_bumper && claw_grip < 1) {
            claw_grip += -0.04;
        }
        if (gamepad2.left_bumper && claw_grip > 0) {
            claw_grip += 0.04;
        }
        if (RightY2 < -0.2 && wristpos > 0.22) {
            wristpos += -0.04;
        }
        if (RightY2 > 0.2 && wristpos < 0.64) {
            wristpos += 0.04;
        }
    }

    /**
     * Describe this function...
     */
    private void WinchControl() {
        boolean up;
        boolean down;

        winch.setTargetPosition(Winchpos);
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        winch.setPower(1);
        if (up) {
            Winchpos += -80;
        }
        if (down) {
            Winchpos += 80;
        }
        winch.setTargetPosition(Winchpos);
        telemetry.addData("winchpos", Double.parseDouble(JavaUtil.formatNumber(Winchpos, 2)));
    }

    /**
     * Describe this function...
     */
    private void RunWheels(int FL, int FR, int BL, int BR) {
        frontleft.setPower(FL * MaxSpeed);
        frontright.setPower(FR * MaxSpeed);
        backleft.setPower(BL * MaxSpeed);
        backright.setPower(BR * MaxSpeed);
    }

    /**
     * Describe this function...
     */
    private void StickRotate() {
        RightX = gamepad1.right_stick_x;
        if (RightX < -0.2 || RightX > 0.2) {
            RunWheels((int) (RightX * -1 * Speed_X), (int) (RightX * 1 * Speed_X), (int) (RightX * -1 * Speed_X), (int) (RightX * 1 * Speed_X));
        } else {
            RunWheels(0, 0, 0, 0);
        }
    }

    /**
     * Describe this function...
     */
    private void WinchFunction() {
        if (gamepad1.dpad_up) {
            winch.setPower(1);
        }
        if (gamepad1.dpad_down) {
            winch.setPower(-1);
        } else {
            winch.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void ExtraControls() {
        if (gamepad2.dpad_left) {
            RunWheels((int) (1 * Speed_X), (int) (1 * Speed_X), (int) (1 * Speed_X), (int) (1 * Speed_X));
            sleep(300);
            RunWheels(0, 0, 0, 0);
            sleep(500);
            wrist.setPosition(0.22);
            armpos = 0;
            ArmPowerCounter = 100;
            tilt.setPosition(0.6);
            claw1.setPosition(0.05);
            arm.setPower(1);
            arm.setTargetPosition(armpos);
        }
    }

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        airplane_servo = hardwareMap.get(Servo.class, "airplane_servo");
        arm = hardwareMap.get(DcMotor.class, "arm");
        winch = hardwareMap.get(DcMotor.class, "winch");
        tilt = hardwareMap.get(Servo.class, "tilt");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        MaxSpeed = 1;
        ArmPowerCounter = 0;
        armpos = 0;
        Winchpos = 0;
        claw_grip = 0.5;
        wristpos = 0.2;
        tilt.setDirection(Servo.Direction.REVERSE);
        tilt.setPosition(0.8);
        wrist.setPosition(wristpos);
        arm.setTargetPosition(armpos);
        winch.setTargetPosition(Winchpos);
        claw1.setPosition(0.05);
        arm.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        tilt.setPosition(0.6);
        while (opModeIsActive()) {
            StickControl2();
            armcontrol();
            claw_control();
            airplane_servo2();
            ExtraControls();
            WinchControl();
        }
        wristpos = 0.22;
        sleep(1);
    }

    /**
     * Describe this function...
     */
    private void StickControl2() {
        float LeftY;
        float LeftX;
        float RightY;
        float L2;
        float R2;

        LeftY = gamepad1.left_stick_y;
        LeftX = gamepad1.left_stick_x;
        RightY = gamepad1.right_stick_y;
        RightX = gamepad1.right_stick_x;
        L2 = gamepad1.left_trigger;
        R2 = gamepad1.right_trigger;
        if (L2 > 0) {
            Speed_X = 0.3;
        } else if (R2 > 0) {
            Speed_X = 1;
        } else {
            Speed_X = 0.75;
        }
        if (LeftY > 0.2 || LeftY < -0.2) {
            RunWheels((int) (((LeftY - RightX) + RightY) * Speed_X), (int) (((LeftY + RightX) - RightY) * Speed_X), (int) (((LeftY - RightX) + RightY) * Speed_X), (int) (((LeftY + RightX) - RightY) * Speed_X));
        } else if (LeftX < -0.2 || LeftX > 0.2) {
            RunWheels((int) (LeftX * -1 * Speed_X), (int) (LeftX * Speed_X), (int) (LeftX * Speed_X), (int) (LeftX * -1 * Speed_X));
        } else if (RightY < -0.2 || RightY > 0.2) {
            RunWheels((int) (RightY * -1 * Speed_X), (int) (RightY * Speed_X), (int) (RightY * Speed_X), (int) (RightY * -1 * Speed_X));
        } else {
            StickRotate();
        }
        telemetry.addData("LeftX", Double.parseDouble(JavaUtil.formatNumber(LeftX, 2)));
        telemetry.addData("LeftY", Double.parseDouble(JavaUtil.formatNumber(LeftY, 2)));
        telemetry.addData("RightX", Double.parseDouble(JavaUtil.formatNumber(RightX, 2)));
        telemetry.addData("speed x", Double.parseDouble(JavaUtil.formatNumber(Speed_X, 2)));
    }
}