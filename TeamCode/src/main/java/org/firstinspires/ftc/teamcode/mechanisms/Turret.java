package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    private DcMotor turretMotor;

    // Ganhos PID
    private double kP = 0.015;
    private double kI = 0.000;
    private double kD = 0.0005;

    // Força mínima para sair da inércia (vencer o atrito da caixa de redução)
    private final double MIN_POWER = 0.05;
    private final double MAX_POWER = 0.6;

    private double goalX = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private double angleTolerance = 0.2;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    // Limites de segurança em ticks do encoder (Altere após ler na telemetria)
    private final int MAX_TICKS = 1000;  // Limite Máximo Direita
    private final int MIN_TICKS = -1000; // Limite Máximo Esquerda

    public void init(HardwareMap hwmap) {
        turretMotor = hwmap.get(DcMotor.class, "turretMotor");

        // Define a posição atual (centro) como a posição 0 do encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void update(LLResult curID) {
        double deltaTime = timer.seconds();
        timer.reset();

        int currentTicks = getCurrentPosition();

        // Se o resultado for nulo ou inválido, desliga o motor
        if (curID == null || !curID.isValid()) {
            turretMotor.setPower(0);
            lastError = 0;
            integralSum = 0;
            return;
        }

        double error = goalX - curID.getTx();

        // Se estiver na margem de tolerância, para o motor
        if (Math.abs(error) < angleTolerance) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = error;
            return;
        }

        // Cálculo Proporcional
        double pTerm = error * kP;

        // Cálculo Integral
        integralSum += error * deltaTime;
        integralSum = Range.clip(integralSum, -0.2, 0.2);
        double iTerm = integralSum * kI;

        // Cálculo Derivativo
        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        power = pTerm + iTerm + dTerm;

        // Adiciona força mínima para vencer o atrito estático
        if (power > 0) {
            power += MIN_POWER;
        } else if (power < 0) {
            power -= MIN_POWER;
        }

        power = Range.clip(power, -MAX_POWER, MAX_POWER);

        // Trava de segurança (Soft Limits) para não enrolar/esticar cabos
        if (currentTicks >= MAX_TICKS && power > 0) {
            power = 0;
        } else if (currentTicks <= MIN_TICKS && power < 0) {
            power = 0;
        }

        turretMotor.setPower(power);
        lastError = error;
    }

    // --- MÉTODOS DE UTILIDADE E TELEMETRIA ---

    /**
     * Retorna a leitura atual do encoder em ticks.
     */
    public int getCurrentPosition() {
        if (turretMotor != null) {
            return turretMotor.getCurrentPosition();
        }
        return 0;
    }

    public void setkP(double newkP) { this.kP = newkP; }
    public double getkP() { return kP; }

    public void setkD(double newkD) { this.kD = newkD; }
    public double getkD() { return kD; }

    public void resetTimer() { timer.reset(); }
}