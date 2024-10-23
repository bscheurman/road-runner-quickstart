package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous2024;

public class Wrist {
    private Servo wrist;
    private Servo Wrist;

    public Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    // within the Claw class
    public class wristDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(1);
            return false;
        }
    }
    public Action wristDown() {
        return new Wrist.wristDown();
    }



    public class wristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(0.75);
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
}
