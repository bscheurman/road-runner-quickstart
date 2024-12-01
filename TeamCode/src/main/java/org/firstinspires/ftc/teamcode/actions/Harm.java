package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous2024;

public class Harm {
    private DcMotor harm;

    public Harm(HardwareMap hardwareMap) {
        harm = hardwareMap.get(DcMotor.class, "harm");
        harm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        harm.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    int pos;
    long startTime;


    // within the Harm class
    public class runToPos implements Action {
        private boolean initialized = false;
        private int direction = 0;
        runToPos (int num){
            pos = num;
            startTime = System.currentTimeMillis();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                harm.setTargetPosition(pos);
                harm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // direction of motor depends if running to a higher or lower position
                if (harm.getCurrentPosition() < pos) {
                    direction = 1;
                    harm.setPower(0.8);
                } else {
                    direction = -1;
                    harm.setPower(-0.8);
                }
                initialized = true;
            }
            // checks current position
            double currpos = harm.getCurrentPosition();
            packet.put("horz slide pos", currpos);
            packet.put("horz slide target", pos);


//            if (currpos < pos && direction == 1) {
//                // true causes the action to rerun
//                return true;
//            } if (currpos > pos && direction == -1) {
//                return true;
//            } else {
//                // false stops action rerun
//                harm.setPower(0);
//                return false;
//            }
            return false;
        }
    }
    public Action runToPos(int pos) {
        return new Harm.runToPos(pos);
    }
}
