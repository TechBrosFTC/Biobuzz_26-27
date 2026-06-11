package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Ateleop2")
public class Ateleop2 extends LinearOpMode {

    private static final double DRIVE_POWER = 0.8;
    private static final double MANUAL_TURN_POWER = 0.4;
    private static final double LEFT_STICK_DEADBAND = 0.15;
    private static final double RIGHT_STICK_DEADBAND = 0.10;

    private static final double KP = 2.2;
    private static final double KI = 0.0006;
    private static final double KD = 2.0;
    private static final double INTEGRAL_LIMIT = 100.0;
    private static final double PID_OUTPUT_LIMIT = 0.35;

    // O V1 usa oito direcoes fixas para facilitar movimentos previsiveis em competicao.
    // As diagonais usam metade em cada eixo para manter uma resposta de potencia uniforme.
    //                           D           FD           F           FE
    private static final double[][] DIRECTIONS = {
            { 1.0,  0.0}, { 0.5,  0.5}, { 0.0,  1.0}, {-0.5,  0.5},
            {-1.0,  0.0}, {-0.5, -0.5}, { 0.0, -1.0}, { 0.5, -0.5}
    };
    private static final String[] DIRECTION_NAMES = {
            "D", "FD", "F", "FE", "E", "TE", "T", "TD"
    };

    private enum DriveState {
        STOPPED,
        DRIVING,
        TURNING
    }

    private IMU imu;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private DriveState driveState = DriveState.STOPPED;
    private int directionSector = -1;

    private double targetHeading;
    private double headingError;
    private double integral;
    private double lastHeadingError;
    private double turnCorrection;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        targetHeading = getHeading();

        while (opModeIsActive()) {
            double stickX = gamepad1.left_stick_x;
            double stickY = -gamepad1.left_stick_y;
            double driveMagnitude = Math.min(Math.hypot(stickX, stickY), 1.0);
            double[] direction = getDiscreteDirection(stickX, stickY);

            double moveX = direction[0] * driveMagnitude * DRIVE_POWER;
            double moveY = direction[1] * driveMagnitude * DRIVE_POWER;
            double turnInput = applyDeadband(gamepad1.right_stick_x, RIGHT_STICK_DEADBAND);

            updateDriveState(moveX, moveY, turnInput);
            driveMecanum(moveX, moveY, turnCorrection);
            updateTelemetry();
        }
    }

    private void initializeHardware() {
        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(hubOrientation));

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void updateDriveState(double moveX, double moveY, double turnInput) {
        boolean operatorIsTurning = turnInput != 0.0;
        boolean robotIsDriving = moveX != 0.0 || moveY != 0.0;

        if (operatorIsTurning) {
            // Durante o giro manual, o operador assume o controle e define o novo heading alvo.
            turnCorrection = turnInput * MANUAL_TURN_POWER * DRIVE_POWER;
            targetHeading = getHeading();
            resetPid();
            driveState = DriveState.TURNING;
            return;
        }

        if (robotIsDriving) {
            if (driveState != DriveState.DRIVING) {
                targetHeading = getHeading();
                resetPid();
            }

            // Sem giro manual, o PID corrige a rotacao para manter o heading da trajetoria.
            turnCorrection = calculateHeadingCorrection();
            driveState = DriveState.DRIVING;
            return;
        }

        turnCorrection = 0.0;
        resetPid();
        driveState = DriveState.STOPPED;
    }

    private double[] getDiscreteDirection(double x, double y) {
        if (Math.hypot(x, y) < LEFT_STICK_DEADBAND) {
            directionSector = -1;
            return new double[]{0.0, 0.0};
        }

        double angle = Math.toDegrees(Math.atan2(y, x));
        if (angle < 0.0) {
            angle += 360.0;
        }

        directionSector = ((int) ((angle + 22.5) / 45.0)) % DIRECTIONS.length;
        return DIRECTIONS[directionSector];
    }

    private double calculateHeadingCorrection() {
        headingError = normalizeAngle(getHeading() - targetHeading);
        integral = clamp(integral + headingError, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = headingError - lastHeadingError;
        lastHeadingError = headingError;

        double pidOutput = (KP * headingError)
                + (KI * integral)
                + (KD * derivative);

        return clamp(pidOutput / 100.0, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    }

    private void resetPid() {
        headingError = 0.0;
        integral = 0.0;
        lastHeadingError = 0.0;
    }

    private void driveMecanum(double x, double y, double turnPower) {
        // O maior valor entre a soma e 1 normaliza as rodas sem alterar a direcao pedida.
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turnPower), 1.0);

        leftFrontPower = (y + x + turnPower) / denominator;
        leftBackPower = (y - x + turnPower) / denominator;
        rightFrontPower = (y - x - turnPower) / denominator;
        rightBackPower = (y + x - turnPower) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    private void updateTelemetry() {
        telemetry.addData("Heading atual", "%.1f", getHeading());
        telemetry.addData("Heading alvo", "%.1f", targetHeading);
        telemetry.addData("Erro heading", "%.1f", headingError);
        telemetry.addData("Correcao PID", "%.3f", turnCorrection);
        telemetry.addData("Estado", driveState);
        telemetry.addData(
                "Setor/direcao",
                directionSector < 0
                        ? "- / parado"
                        : directionSector + " / " + DIRECTION_NAMES[directionSector]);
        telemetry.addData(
                "Motores LF/LB/RF/RB",
                "%.2f / %.2f / %.2f / %.2f",
                leftFrontPower,
                leftBackPower,
                rightFrontPower,
                rightBackPower);
        telemetry.update();
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0.0 : value;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }
        while (angle <= -180.0) {
            angle += 360.0;
        }
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
