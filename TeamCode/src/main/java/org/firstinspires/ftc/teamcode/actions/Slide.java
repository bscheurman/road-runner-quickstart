package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


// lift class
public class Slide {
    private DcMotorEx slideMotor;
    int position;
    long startTime;

    String motorNameVar;

    // Horizontal Slide Arm
    public Slide (HardwareMap hardwareMap, String motorName) {
        motorNameVar = motorName;
        slideMotor = hardwareMap.get(DcMotorEx.class, motorNameVar);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public class SlideToPos implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;
        private int direction = 0;
        SlideToPos (int pos){
            position = pos;
            startTime = System.currentTimeMillis();
        }

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                // direction of motor depends if running to a higher or lower position
                if (slideMotor.getCurrentPosition() < position) {
                    direction = 1;
                    slideMotor.setPower(0.8);
                } else {
                    direction = -1;
                    slideMotor.setPower(-0.8);
                }
                initialized = true;
            }

            // checks lift's current position
            double pos = slideMotor.getCurrentPosition();
            packet.put("slidePos", pos);
            if (pos < position && direction == 1) {
                // true causes the action to rerun
                return true;
            } if (pos > position && direction == -1) {
                return true;
            } else {
                // false stops action rerun
                slideMotor.setPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 1000 encoder ticks, then powers it off
        }
    }
    public Action slideToPos(int pos) {

        return new SlideToPos(pos);
    }

    // within the Lift class
    public class SlideTo0 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                slideMotor.setPower(-0.8);
                initialized = true;
            }

            double pos = slideMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > 100.0) {
                return true;
            } else {
                slideMotor.setPower(0);
                return false;
            }
        }
    }

    public Action slideTo0() {
        return new SlideTo0();
    }
}

