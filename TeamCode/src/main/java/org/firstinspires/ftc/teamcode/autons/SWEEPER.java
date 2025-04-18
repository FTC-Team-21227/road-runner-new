package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem_Constants;

public class SWEEPER {
    private Servo Sweeper;
    final double sweeperScale0 = Subsystem_Constants.sweeperScale0;
    final double sweeperScale1 = Subsystem_Constants.sweeperScale1;
    final double closeSweeper = Subsystem_Constants.closeSweeper;
    final double openSweeper = Subsystem_Constants.openSweeper;

    public SWEEPER(HardwareMap hardwareMap) {
        Sweeper = hardwareMap.get(Servo.class, "Sweeper");
        Sweeper.scaleRange(sweeperScale0, sweeperScale1);
    }

    public class MoveSweeper implements Action {
        ElapsedTime time = new ElapsedTime();
        boolean start;
        double runTime;
        double pos;

        public MoveSweeper(double pos) {
            start = false;
            runTime = 0;
            this.pos = pos;
        }

        public MoveSweeper(double pos, double runt) {
            start = false;
            runTime = runt;
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!start) {
                time.reset();
                start = true;
            }
            if (time.seconds() < runTime) {
                return true;
            } else {
                Sweeper.setPosition(pos); //0.28 //0.65
                return false;
            }
        }
    }

    public Action RotatePosition0() {
        return new MoveSweeper(closeSweeper);
    }

    public Action RotatePosition0(double runt) {
        return new MoveSweeper(closeSweeper, runt);
    }

    public Action RotatePosition1() {
        return new MoveSweeper(openSweeper);
    }

    public Action RotatePosition1(double runt) {
        return new MoveSweeper(openSweeper, runt);
    }
}