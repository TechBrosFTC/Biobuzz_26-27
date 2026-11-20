
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.PID;

public class Shooter {

    // Encoder
    private static final double TICKS_PER_ROTATION = 28.0;

    // Ganhos iniciais para testes
    private static final double KP = 0.00005
            ;
    private static final double KD = 0.0000000;

    // Limite de variação da potência por segundo
    private static final double MAX_POWER_SLEW = 2.0;

    // Motor
    private final DcMotorEx shooterMotor;

    // Controlador
    private final PID shooterPID;

    // Variáveis
    private double shooterPower = 0.0;
    private double shooterRPM = 0.0;
    private double targetRPM = 0.0;

    private long previousTime;

    public Shooter(DcMotorEx shooterMotor) {

        this.shooterMotor = shooterMotor;

        shooterMotor.setMode(
                DcMotorEx.RunMode.RUN_USING_ENCODER
        );

        shooterPID = new PID(KP, KD);

        previousTime = System.nanoTime();
    }

    // Leitura direta da velocidade do encoder
    public double updateRPM() {

        double ticksPerSecond = shooterMotor.getVelocity();

        shooterRPM = Math.abs(
                (ticksPerSecond / TICKS_PER_ROTATION) * 60.0
        );

        return shooterRPM;
    }

    // Controle de velocidade
    public void setRPM(double targetRPM) {

        this.targetRPM = targetRPM;

        long currentTime = System.nanoTime();

        double deltaTime =
                (currentTime - previousTime) / 1_000_000_000.0;

        previousTime = currentTime;

        if (deltaTime <= 0) return;

        // Evita saltos excessivos em intervalos irregulares
        deltaTime = Math.min(deltaTime, 0.1);

        // RPM atual
        double currentRPM = updateRPM();

        // Erro
        double error = targetRPM - currentRPM;

        // Correção do PID
        double correction = shooterPID.update(
                error,
                currentRPM,
                deltaTime
        );

        // Limita a variação da potência
        double maxChange = MAX_POWER_SLEW * deltaTime;

        correction = Range.clip(
                correction,
                -maxChange,
                maxChange
        );

        // Controle incremental
        shooterPower += correction;

        // Limites do motor
        shooterPower = Range.clip(shooterPower, 0.0, 1.0);

        shooterMotor.setPower(shooterPower);
    }

    public double getRPM() {
        return shooterRPM;
    }

    public double getPower() {
        return shooterPower;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
    public void setPID(double kp, double kd){
        this.shooterPID.setCoefficients(kp, kd);
    }


    public void shooterOff() {

        shooterMotor.setPower(0.0);

        shooterPower = 0.0;
        shooterRPM = 0.0;
        targetRPM = 0.0;

        shooterPID.reset();

        previousTime = System.nanoTime();
    }
}
