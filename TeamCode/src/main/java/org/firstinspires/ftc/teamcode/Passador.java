package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Passador {

    private final DcMotorEx motor;

    private static final double TICKS_PER_REV = 28.0;

    // PIDF - ganhos para calibração
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    // Recovery Boost
    private double recoveryGain = 0.00008;
    private double maxBoost = 0.15;

    // Detecção de Passagem do Elemento
    private double passThreshold = 300;
    private double baselineRPM = 0.0;
    private double minPassRPM = 0.0;

    // Adaptação Automática / Gradual do Feedforward (kF)
    private boolean autoKFEnabled = true;
    private double kFAdaptRate = 0.000001; // Taxa de ajuste gradual por segundo
    private double underSpeedRatio = 0.90; // Se currentRPM < targetRPM * 0.90 (ficou muito abaixo do target)
    private double minKF = 0.00001;
    private double maxKF = 0.001;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double output = 0.0;
    private double boost = 0.0;

    // PID
    private double integral = 0.0;
    private double lastError = 0.0;
    private double filteredDerivative = 0.0;

    // Timing
    private long lastUpdate = 0;
    private long passTime = 0;
    private long recoveryStart = 0;
    private long recoveryStableStart = 0;

    private long lastPassTime = 0;
    private long lastRecoveryMs = 0;

    private boolean passDetected = false;
    private boolean recovering = false;
    private boolean recoveryCompleted = false;

    public Passador(DcMotorEx motor) {
        this.motor = motor;

        motor.setMode(
            DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }

    public void setTargetRPM(double rpm) {
        if (rpm != targetRPM) {
            integral = 0;
            lastError = 0;
            filteredDerivative = 0;
        }

        targetRPM = Math.max(0, rpm);

        if (targetRPM == 0) {
            stop();
        }
    }

    public void update() {

        long now = System.nanoTime();

        if (lastUpdate == 0) {
            lastUpdate = now;
            currentRPM = getMeasuredRPM();
            baselineRPM = currentRPM;
            return;
        }

        double dt = (now - lastUpdate) / 1e9;
        lastUpdate = now;

        dt = Math.max(0.001, Math.min(0.05, dt));

        currentRPM = getMeasuredRPM();

        if (targetRPM <= 0) {
            motor.setPower(0);
            return;
        }

        double error = targetRPM - currentRPM;

        // Ajuste Adaptativo Gradual do Feedforward (kF) caso fique muito abaixo do target
        if (autoKFEnabled && !recovering && targetRPM > 0) {
            if (currentRPM < targetRPM * underSpeedRatio && output < 1.0) {
                // Aumenta kF gradualmente proporcionalmente ao erro relativo e ao delta de tempo (dt)
                double errorRatio = (targetRPM - currentRPM) / targetRPM;
                kF += kFAdaptRate * errorRatio * dt;
                kF = Math.min(maxKF, kF);
            } else if (currentRPM > targetRPM * 1.08 && output > 0.0) {
                // Reduz kF gradualmente caso o RPM esteja consideravelmente acima da meta
                double overRatio = (currentRPM - targetRPM) / targetRPM;
                kF -= kFAdaptRate * overRatio * dt;
                kF = Math.max(minKF, kF);
            }
        }

        // Detecção de Passagem
        detectPass(now);

        // Medição de Recuperação
        updateRecovery(now);

        // PID
        double derivative = (error - lastError) / dt;

        filteredDerivative =
            0.8 * filteredDerivative +
            0.2 * derivative;

        double p = kP * error;
        double d = kD * filteredDerivative;
        double f = kF * targetRPM;

        // Recovery Boost
        boost = 0;

        if (recovering && error > 0) {
            boost = Math.min(
                maxBoost,
                error * recoveryGain
            );
        }

        // Anti-windup
        double candidateIntegral = integral + error * dt;

        candidateIntegral = Math.max(
            -1000,
            Math.min(1000, candidateIntegral)
        );

        double candidateOutput = p + kI * candidateIntegral + d + f + boost;

        if ((candidateOutput >= 0 && candidateOutput <= 1) ||
             (candidateOutput > 1 && error < 0) ||
             (candidateOutput < 0 && error > 0)) {

            integral = candidateIntegral;
        }

        output = p + kI * integral + d + f + boost;

        output = Math.max(0, Math.min(1, output));

        motor.setPower(output);

        lastError = error;

        // Baseline de RPM
        if (!recovering && currentRPM > targetRPM * 0.8) {
            baselineRPM = 0.95 * baselineRPM + 0.05 * currentRPM;
        }
    }

    private double getMeasuredRPM() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    private void detectPass(long now) {

        passDetected = false;

        if (recovering) return;

        if (baselineRPM < targetRPM * 0.8) return;

        if (now - lastPassTime < 250_000_000L) return;

        double drop = baselineRPM - currentRPM;

        if (drop >= passThreshold) {

            passDetected = true;
            recovering = true;

            passTime = now;
            recoveryStart = now;
            lastPassTime = now;

            minPassRPM = currentRPM;

            recoveryStableStart = 0;
            recoveryCompleted = false;
        }
    }

    private void updateRecovery(long now) {

        if (!recovering) return;

        minPassRPM = Math.min(
            minPassRPM,
            currentRPM
        );

        double recoveryTarget = targetRPM * 0.95;

        if (currentRPM >= recoveryTarget) {

            if (recoveryStableStart == 0) {
                recoveryStableStart = now;
            }

            if (now - recoveryStableStart >= 100_000_000L) {

                lastRecoveryMs = (now - recoveryStart) / 1_000_000L;

                recovering = false;
                recoveryCompleted = true;
            }

        } else {
            recoveryStableStart = 0;
        }
    }

    public double getRPM() {
        return currentRPM;
    }

    public double getOutput() {
        return output;
    }

    public double getBoost() {
        return boost;
    }

    public double getMinPassRPM() {
        return minPassRPM;
    }

    public long getRecoveryMs() {
        return lastRecoveryMs;
    }

    public boolean isPassDetected() {
        return passDetected;
    }

    public boolean isRecovering() {
        return recovering;
    }

    public boolean isRecoveryCompleted() {
        return recoveryCompleted;
    }

    public boolean isReady() {
        return targetRPM > 0 && currentRPM >= targetRPM * 0.85;
    }

    public void stop() {

        targetRPM = 0;
        integral = 0;
        boost = 0;
        output = 0;

        recovering = false;
        recoveryCompleted = false;

        motor.setPower(0);
    }

    public void setPIDF(double p, double i, double d, double f, double recoveryGain, double maxBoost) {
        kP = p;
        kI = i;
        kD = d;
        kF = f;
        this.maxBoost = maxBoost;
        this.recoveryGain = recoveryGain;
    }

    public void setPassThreshold(double threshold) {
        this.passThreshold = threshold;
    }

    public void setAutoKFEnabled(boolean enabled) {
        this.autoKFEnabled = enabled;
    }

    public boolean isAutoKFEnabled() {
        return autoKFEnabled;
    }

    public double getKF() {
        return kF;
    }

    public void setKFAdaptRate(double rate) {
        this.kFAdaptRate = rate;
    }

    public void setUnderSpeedRatio(double ratio) {
        this.underSpeedRatio = ratio;
    }
}
