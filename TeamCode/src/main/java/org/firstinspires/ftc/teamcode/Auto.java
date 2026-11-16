package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * OpMode Autônomo com movimentação Mecanum controlada pelo GoBILDA Pinpoint
 * e rotina automatizada do Shooter.
 *
 * Convenção de movimento:
 * X positivo = direita
 * X negativo = esquerda
 * Y positivo = frente
 * Y negativo = trás
 * Heading positivo = sentido anti-horário, conforme orientação do Pinpoint.
 */
@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {

    // --- ODOMETRIA E CHASSI ---
    private GoBildaPinpointDriver odo;

    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    // --- MOTORES DOS MECANISMOS ---
    private DcMotorEx shooter;
    private DcMotorEx transfer;
    private DcMotorEx intake;
    private DcMotorEx passador;

    // --- SERVOS E TORRETA ---
    private Servo servorampa;
    private Servo hood;
    private CRServo torreta;
    private CRServo torreta1;
    private DcMotorEx encoder_torreta;
    private Turret turretObjeto;

    // --- LIMELIGHT ---
    private Limelight limelight;

    // --- CONFIGURAÇÕES PIDF DOS MOTORES ---
    public static double SHOOTER_P = 88;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 13.4291;

    public static double TRANSFER_P = 15;
    public static double TRANSFER_I = 0.0;
    public static double TRANSFER_D = 0.0;
    public static double TRANSFER_F = 12.41;

    public static double PASSADOR_P = 5.0;
    public static double PASSADOR_I = 0.0;
    public static double PASSADOR_D = 0.0;
    public static double PASSADOR_F = 12.0;

    // --- CONTROLE DE MOVIMENTAÇÃO / PID DE HEADING ---
    private double kp = 0;
    private double kd = 0;
    private double ki = 0;

    private double erro;
    private double proporcional;
    private double integral = 0.0;
    private double ultimoErro = 0.0;
    private double alvo = 0.0;

    // --- CONFIGURAÇÕES DO SHOOTER ---
    private double shooterTicksPerRev = 28.0;
    public static int defaultShooterRPM = 2760;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------------------------------------------
        // CONFIGURAÇÃO DOS SERVOS
        // -------------------------------------------------------------
        servorampa = hardwareMap.get(Servo.class, "servorampa");
        hood = hardwareMap.get(Servo.class, "hood");

        // -------------------------------------------------------------
        // CONFIGURAÇÃO DOS MOTORES DO CHASSI MECANUM
        // Mantidas as direções do código novo.
        // -------------------------------------------------------------
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        direitaFrente.setDirection(DcMotorSimple.Direction.FORWARD);
        direitaTras.setDirection(DcMotorSimple.Direction.FORWARD);
        esquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.FORWARD);

        direitaFrente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        direitaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdaFrente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -------------------------------------------------------------
        // CONFIGURAÇÃO DO SHOOTER
        // -------------------------------------------------------------
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F
        );
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // -------------------------------------------------------------
        // CONFIGURAÇÃO DO PASSADOR
        // -------------------------------------------------------------
        passador = hardwareMap.get(DcMotorEx.class, "passador");
        passador.setDirection(DcMotorSimple.Direction.FORWARD);
        passador.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        passador.setVelocityPIDFCoefficients(
                PASSADOR_P, PASSADOR_I, PASSADOR_D, PASSADOR_F
        );

        // -------------------------------------------------------------
        // CONFIGURAÇÃO DO TRANSFER E INTAKE
        // -------------------------------------------------------------
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transfer.setVelocityPIDFCoefficients(
                TRANSFER_P, TRANSFER_I, TRANSFER_D, TRANSFER_F
        );

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // -------------------------------------------------------------
        // CONFIGURAÇÃO DA TORRETA E LIMELIGHT
        // -------------------------------------------------------------
        torreta = hardwareMap.get(CRServo.class, "torreta");
        torreta1 = hardwareMap.get(CRServo.class, "torreta1");

        torreta1.setDirection(DcMotorSimple.Direction.REVERSE);
        torreta.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder_torreta = hardwareMap.get(DcMotorEx.class, "leftFront");

        double ticksPerDegreeTurret = 8192.0 / 36.0;
        turretObjeto = new Turret(
                torreta,
                torreta1,
                encoder_torreta,
                ticksPerDegreeTurret,
                DcMotorSimple.Direction.REVERSE
        );


        // -------------------------------------------------------------
        // CONFIGURAÇÃO DA ODOMETRIA GoBILDA PINPOINT
        // -------------------------------------------------------------
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setEncoderResolution(76.964, DistanceUnit.MM);
        odo.setOffsets(-130, -55, DistanceUnit.MM);

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        odo.initialize();

        // Posição inicial definida no referencial do campo.
        odo.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        10.0,
                        10.0,
                        AngleUnit.DEGREES,
                        0.0
                )
        );

        servorampa.setPosition(1.0);
        hood.setPosition(0.5);

        while (opModeInInit()){
            telemetry.addData("Status", "Autônomo Inicializado com Sucesso!");
            telemetry.addData("Pinpoint X", "%.2f",
            odo.getPosition().getX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y", "%.2f",
            odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        // -------------------------------------------------------------
        // ROTINA AUTÔNOMA
        // -------------------------------------------------------------
        if (opModeIsActive()) {

            // 1. Disparo inicial
            telemetry.addData("Autônomo", "Iniciando disparo do Shooter...");
            telemetry.update();
            shootRoutine(defaultShooterRPM, 3.5);

            // 2. Avança 5 polegadas, mantendo heading em 0 grau.
            telemetry.addData("Autônomo", "Avançando no eixo Y...");
            telemetry.update();
            MRUY(0.35, 5, 0.3);

            // 3. Estrifa 20 polegadas para a direita.
            telemetry.addData("Autônomo", "Estrifando no eixo X...");
            telemetry.update();

            sleep(500);

            // 4. Gira a base para 90 graus.
            telemetry.addData("Autônomo", "Girando para 90 graus...");
            telemetry.update();


            // 5. Finalização
            stopAll();

            telemetry.addData("Autônomo", "Rotina Autônoma Concluída!");
            telemetry.update();
        }
    }

    // =================================================================
    // SISTEMA DE MOVIMENTAÇÃO - PINPOINT + MECANUM
    // =================================================================

    /**
     * Normaliza um ângulo para o intervalo -180 a +180 graus.
     */
    public double normalizarAngulo(double angulo) {
        while (angulo > 180) {
            angulo -= 360;
        }
        while (angulo < -180) {
            angulo += 360;
        }
        return angulo;
    }

    /**
     * Retorna o heading atual medido pelo Pinpoint.
     */
    public double gyro() {
        odo.update();
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /**
     * Zera o acumulador e o erro anterior do PID de heading.
     */
    public void resetPID() {
        integral = 0.0;
        ultimoErro = 0.0;
    }

    /**
     * Calcula a correção PID para manter o heading desejado.
     */
    public double headingPID() {
        erro = normalizarAngulo(gyro() - alvo);

        proporcional = kp * erro;

        integral += erro;
        integral = Range.clip(integral, -100.0, 100.0);

        double derivativa = (erro - ultimoErro) * kd;

        double correcao = (
                proporcional
                        + integral * ki
                        + derivativa
        ) / 100.0;

        ultimoErro = erro;

        return Range.clip(correcao, -0.5, 0.5);
    }

    /**
     * Anda no eixo Y usando o deslocamento medido pelo Pinpoint.
     *
     * distanceInches positivo = frente.
     * distanceInches negativo = trás.
     */
    public void driveY(
            double distanceInches,
            double power,
            double targetHeading,
            double timeoutSeconds
    ) {
        driveDistance(false, distanceInches, power,
                targetHeading, timeoutSeconds);
    }

    /**
     * Anda no eixo X usando o deslocamento medido pelo Pinpoint.
     *
     * distanceInches positivo = direita.
     * distanceInches negativo = esquerda.
     */
    public void driveX(
            double distanceInches,
            double power,
            double targetHeading,
            double timeoutSeconds
    ) {
        driveDistance(true, distanceInches, power,
                targetHeading, timeoutSeconds);
    }

    /**
     * Rotina compartilhada de deslocamento nos eixos X e Y.
     * A distância é projetada no referencial do robô usando o heading
     * registrado no início do movimento.
     */
    private void driveDistance(
            boolean eixoX,
            double distanceInches,
            double power,
            double targetHeading,
            double timeoutSeconds
    ) {
        if (!opModeIsActive() || Math.abs(distanceInches) < 0.01) {
            parar();
            return;
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        resetPID();
        alvo = targetHeading;

        odo.update();
        Pose2D initialPose = odo.getPosition();

        double startX = initialPose.getX(DistanceUnit.INCH);
        double startY = initialPose.getY(DistanceUnit.INCH);
        double initialHeading = Math.toRadians(
                initialPose.getHeading(AngleUnit.DEGREES)
        );

        double targetDistance = Math.abs(distanceInches);
        double direction = Math.signum(distanceInches);

        double maxPower = Range.clip(Math.abs(power), 0.0, 1.0);
        double minPower = Math.min(maxPower,
                Math.max(0.12, maxPower * 0.25));

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            odo.update();
            Pose2D currentPose = odo.getPosition();

            double currentX = currentPose.getX(DistanceUnit.INCH);
            double currentY = currentPose.getY(DistanceUnit.INCH);

            double deltaX = currentX - startX;
            double deltaY = currentY - startY;

            // Projeta o deslocamento global no eixo do robô.
            double traveledOnX =
                    deltaX * Math.cos(initialHeading)
                            + deltaY * Math.sin(initialHeading);

            double traveledOnY =
                    -deltaX * Math.sin(initialHeading)
                            + deltaY * Math.cos(initialHeading);

            double signedTravel = eixoX ? traveledOnX : traveledOnY;
            double traveledDistance = signedTravel * direction;

            if (traveledDistance >= targetDistance) {
                break;
            }

            // Desaceleração progressiva na aproximação do alvo.
            double progress = Range.clip(
                    traveledDistance / targetDistance,
                    0.0,
                    1.0
            );

            double currentPower = maxPower
                    - (maxPower - minPower) * progress;

            currentPower = Range.clip(
                    currentPower,
                    minPower,
                    maxPower
            );

            double moveX = eixoX ? currentPower * direction : 0.0;
            double moveY = eixoX ? 0.0 : currentPower * direction;

            double rotation = headingPID();

            mecanum(moveX, moveY, rotation);

            telemetry.addData("Movimento", eixoX ? "Drive X" : "Drive Y");
            telemetry.addData("Distância percorrida", "%.2f in",
                    Math.max(0.0, traveledDistance));
            telemetry.addData("Distância alvo", "%.2f in",
                    targetDistance);
            telemetry.addData("Heading atual", "%.2f", gyro());
            telemetry.addData("Heading alvo", "%.2f", targetHeading);
            telemetry.addData("Tempo", "%.2f / %.2f",
                    timer.seconds(), timeoutSeconds);
            telemetry.update();

            sleep(10);
        }

        parar();
        resetPID();
    }

    /**
     * Gira o robô até atingir o heading desejado, usando o Pinpoint.
     */
    public void turn(double targetHeadingDegrees, double timeoutSeconds) {

        if (!opModeIsActive()) {
            parar();
            return;
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        resetPID();

        final double kPturn = 0.02;
        final double minTurnPower = 0.10;
        final double maxTurnPower = 0.65;
        final double headingTolerance = 1.5;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            odo.update();

            double currentHeading = odo.getPosition()
                    .getHeading(AngleUnit.DEGREES);

            double errorHeading = normalizarAngulo(
                    targetHeadingDegrees - currentHeading
            );

            if (Math.abs(errorHeading) <= headingTolerance) {
                break;
            }

            double turnPower = Range.clip(
                    errorHeading * kPturn,
                    -maxTurnPower,
                    maxTurnPower
            );

            // Evita potência muito baixa para vencer o atrito estático.
            if (Math.abs(turnPower) < minTurnPower) {
                turnPower = Math.copySign(minTurnPower, turnPower);
            }

            mecanum(0.0, 0.0, turnPower);

            telemetry.addData("Movimento", "Turn");
            telemetry.addData("Heading atual", "%.2f", currentHeading);
            telemetry.addData("Heading alvo", "%.2f",
                    targetHeadingDegrees);
            telemetry.addData("Erro", "%.2f", errorHeading);
            telemetry.update();

            sleep(10);
        }

        parar();
        resetPID();
    }

    /**
     * Move o robô até uma coordenada global X, Y e heading.
     *
     * X positivo = direita no campo.
     * Y positivo = frente no campo.
     */
    public void driveToPosition(
            double targetX,
            double targetY,
            double targetHeading,
            double timeoutSeconds
    ) {
        if (!opModeIsActive()) {
            parar();
            return;
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        resetPID();

        final double kpPosition = 0.08;
        final double kpHeading = 0.04;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {

            odo.update();
            Pose2D pose = odo.getPosition();

            double currentX = pose.getX(DistanceUnit.INCH);
            double currentY = pose.getY(DistanceUnit.INCH);
            double currentHeading = pose.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorHeading = normalizarAngulo(
                    targetHeading - currentHeading
            );

            double distanceError = Math.hypot(errorX, errorY);

            if (distanceError < 0.7
                    && Math.abs(errorHeading) < 2.0) {
                break;
            }

            // Transforma o erro do campo para o referencial do robô.
            double headingRad = Math.toRadians(currentHeading);

            double robotErrorX =
                    errorX * Math.cos(headingRad)
                            + errorY * Math.sin(headingRad);

            double robotErrorY =
                    -errorX * Math.sin(headingRad)
                            + errorY * Math.cos(headingRad);

            double moveX = Range.clip(
                    robotErrorX * kpPosition,
                    -0.7,
                    0.7
            );

            double moveY = Range.clip(
                    robotErrorY * kpPosition,
                    -0.7,
                    0.7
            );

            double rotation = Range.clip(
                    errorHeading * kpHeading,
                    -0.5,
                    0.5
            );

            mecanum(moveX, moveY, rotation);

            telemetry.addData("Posição X", "%.2f / %.2f",
                    currentX, targetX);
            telemetry.addData("Posição Y", "%.2f / %.2f",
                    currentY, targetY);
            telemetry.addData("Heading", "%.2f / %.2f",
                    currentHeading, targetHeading);
            telemetry.addData("Erro de posição", "%.2f in",
                    distanceError);
            telemetry.update();

            sleep(10);
        }

        parar();
        resetPID();
    }

    /**
     * Move o chassi por um intervalo de tempo, mantendo o heading.
     */
    public void MRUY(double power, double dist, double time){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && odo.getPosition().getY(DistanceUnit.INCH) < dist && timer.seconds() < time) {
            mecanum(0, power, 0);
        }
        mecanum(0, 0, 0);
    }
    public void MRUX(double power, double dist, double time){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && odo.getPosition().getY(DistanceUnit.INCH) < dist && timer.seconds() < time) {
            mecanum(power, 0, 0);
        }
        mecanum(0, 0, 0);
    }


    /**
     * Aplica potência aos quatro motores Mecanum.
     *
     * x  = estrafe
     * y  = frente/trás
     * rx = rotação
     *
     * As direções dos motores são configuradas no runOpMode().
     */
    public void mecanum(double x, double y, double rx) {

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx),
                1.0
        );

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        esquerdaFrente.setPower(frontLeftPower);
        esquerdaTras.setPower(backLeftPower);
        direitaFrente.setPower(frontRightPower);
        direitaTras.setPower(backRightPower);
    }

    /**
     * Interrompe os motores de tração.
     */
    public void parar() {
        if (direitaFrente != null) direitaFrente.setPower(0.0);
        if (direitaTras != null) direitaTras.setPower(0.0);
        if (esquerdaFrente != null) esquerdaFrente.setPower(0.0);
        if (esquerdaTras != null) esquerdaTras.setPower(0.0);
    }

    // =================================================================
    // MÉTODOS DOS MECANISMOS - SHOOTER, TRANSFER, INTAKE E PASSADOR
    // =================================================================

    /**
     * Executa a rotina automatizada de aceleração e disparo do Shooter.
     */
    public void shootRoutine(int targetRPM, double shootDurationSeconds) {

        if (!opModeIsActive()) {
            return;
        }

        double targetTicksPerSec =
                (targetRPM * shooterTicksPerRev) / 60.0;

        shooter.setVelocity(targetTicksPerSec);

        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        // Aguarda a velocidade do Shooter estabilizar.
        while (opModeIsActive() && shootTimer.seconds() < 2.0) {

            double currentRPM =
                    (shooter.getVelocity() * 60.0) / shooterTicksPerRev;

            telemetry.addData("Shooter Target RPM", targetRPM);
            telemetry.addData("Shooter Current RPM", "%.2f", currentRPM);
            telemetry.update();

            if (currentRPM >= targetRPM - 50) {
                break;
            }

            sleep(20);
        }

        if (!opModeIsActive()) {
            stopShooter();
            return;
        }

        // Aciona os mecanismos de alimentação.
        passador.setVelocity(convertRPMtoTicks(5500));
        transfer.setVelocity(convertRPMtoTicks(3000));
        intake.setPower(1.0);
        servorampa.setPosition(0.5);

        shootTimer.reset();

        while (opModeIsActive()
                && shootTimer.seconds() < shootDurationSeconds) {

            double currentRPM =
                    (shooter.getVelocity() * 60.0) / shooterTicksPerRev;

            telemetry.addData("Shooter Atirando RPM",
                    "%.2f", currentRPM);
            telemetry.update();

            sleep(20);
        }

        stopShooter();
    }

    /**
     * Desliga Shooter, Passador, Transfer e Intake.
     */
    public void stopShooter() {
        if (shooter != null) shooter.setVelocity(0.0);
        if (passador != null) passador.setVelocity(0.0);
        if (transfer != null) transfer.setVelocity(0.0);
        if (intake != null) intake.setPower(0.0);
        if (servorampa != null) servorampa.setPosition(1.0);
    }

    /**
     * Converte RPM para ticks por segundo considerando motor de 28 ticks/rev.
     */
    public int convertRPMtoTicks(int rpm) {
        return rpm * 28 / 60;
    }

    /**
     * Desliga os motores do chassi e os mecanismos.
     */
    public void stopAll() {
        parar();
        stopShooter();

        if (torreta != null) {
            torreta.setPower(0.0);
        }

        if (torreta1 != null) {
            torreta1.setPower(0.0);
        }
    }
}
