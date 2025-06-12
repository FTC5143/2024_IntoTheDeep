package org.firstinspires.ftc.teamcode.opmodes.debug.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.DebugRobot;

@Autonomous(name="Debug Move Test", group="autonomous")
public class DebugPosTest extends LinearOpMode {

    DebugRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new DebugRobot(this);
        robot.startup();

        robot.drive_train.odo_reset(0,0,Math.PI * -1/2);

        waitForStart();

        // robot.drive_train.odo_drive(0,0,Math.PI * 0, 0.5);

        // halt(5);

        robot.drive_train.odo_drive(0,0,Math.PI * 1/2, 0.5);

        halt(2);

        robot.drive_train.odo_drive(0,0,Math.PI, 0.5);

        halt(2);

        robot.drive_train.odo_drive(0,0,Math.PI * 3/2, 0.5);

        halt(2);

        robot.drive_train.odo_drive(0,0,0, 0.5);

        halt(2);

        robot.shutdown();
    }

    public void halt(double seconds) {
        double start = robot.opmode.getRuntime();
        while ((robot.opmode.getRuntime() - start) < seconds && robot.opmode.opModeIsActive());
    }
}
