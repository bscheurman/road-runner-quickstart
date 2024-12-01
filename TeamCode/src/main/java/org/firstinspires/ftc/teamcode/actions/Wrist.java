package org.firstinspires.ftc.teamcode.actions;

import static java.lang.Math.max;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    private Servo wrist;

    public double targetPos = 0.5;
    long startTime = 0;


    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    // within the Claw class
    public class wristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(.9);

            return false;
        }
    }
    public Action wristDown() {
        return new Wrist.wristDown();
    }

    public double getTargetPos(){
        return targetPos;
    }

    public double getCurrentPos(){
        return wrist.getPosition();
    }

    public class wristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.70);
            return false;
        }
    }
    public Action wristUp() {
        return new Wrist.wristUp();
    }

    public class wristBack implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.15);
            return false;
        }
    }
    public Action wristBack() {
        return new Wrist.wristBack();
    }

    public class WristMoveToPos implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;
        private int direction = 0;
        WristMoveToPos (double pos){
            targetPos = max(0,pos); // cant go below 0;
            startTime = System.currentTimeMillis();
        }


        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                // direction of motor depends if running to a higher or lower position
                if (wrist.getPosition() < targetPos) {
                    direction = 1;
                } else {
                    direction = -1;
                }
                wrist.setPosition(targetPos);
                initialized = true;
            }

            // checks lift's current position
            double pos = wrist.getPosition();

            packet.put("wristPos " , pos);
            packet.put("wristPosTarget " , targetPos);
            // timeout if it takes 0.5 seconds
            if (System.currentTimeMillis() > startTime + 200) {
                //slideMotor.setPower(0);
                return false;
            }
            if (pos < targetPos && direction == 1) {
                // true causes the action to rerun
                return false;
            } if (pos > targetPos && direction == -1) {
                return false;
            } else {
                // false stops action rerun
                return false;
            }
            //wristPos : 0.75
            //wristPosTarget : 0.7

        }

    }
    public Action wristMoveToPos(double pos) {

        return new Wrist.WristMoveToPos(pos);
    }
}
