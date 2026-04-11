package org.firstinspires.ftc.teamcode.utils;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

@Config

public class BallTracker {




    private static final int TICKS_PER_REV = 1024;
    private static final int TICKS_PER_SLOT = 2730;
    private Spindexer spindexer;
    private Telemetry telemetry;
    private int spindTick; // spind ticks from encoder
    private int spindAbsTicks;
    public Slot thisBall;
    public enum BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }


    public static final List<BallColor> motif1 = new ArrayList<>(Arrays.asList(BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN));
    public static final List<BallColor> motif2 = new ArrayList<>(Arrays.asList(BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE));
    public static final List<BallColor> motif3 = new ArrayList<>(Arrays.asList(BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE));



    public static List<BallColor> targetMotif;
    public BallTracker(Spindexer spindexer) {
        this.spindexer = spindexer;
        this.targetMotif = motif1;
    }
    public class Slot {
        public String name;
        public BallColor color;
        public int pos; // 0 - whatever ------- actual enc positon
        public int currentAbsPos; // 0 - 1024
        public Slot(String name, int pos, int currentAbsPos) {
            this.name = name;
            this.pos = pos;
            this.color = BallColor.EMPTY;
            this.currentAbsPos = 0;
        }
    }
    public Slot slotA = new Slot("Slot A", 0, 0);
    public Slot slotC = new Slot("Slot C", 341, 341);
    public Slot slotB = new Slot("Slot B", 682, 682);

    public void update() {
        spindTick = spindexer.getCurrentPosition();
        spindAbsTicks = calcAbsPos(spindTick);

        updateSlotTicks(slotA);
        updateSlotTicks(slotB);
        updateSlotTicks(slotC);
    }
    private void updateSlotTicks(Slot slot) {
        int calculatedPos = spindTick + slot.pos;
        slot.currentAbsPos = calcAbsPos(calculatedPos);
    }
    private int calcAbsPos(int ticks) {
        return ticks % TICKS_PER_REV;
    }
    public void setSlotClr(Slot slot, BallColor color) {
        slot.color = color;
    }
    public Slot getSlotAtShootingPos() {
        if (slotA.currentAbsPos < 350 && slotA.currentAbsPos > 330) return slotA;
        if (slotB.currentAbsPos < 350 && slotB.currentAbsPos > 330) return slotB;
        return slotC;
    }

    public Slot getSlotAtCollectPos() {
        if (slotA.currentAbsPos < 50 || slotA.currentAbsPos > 970) return slotA;
        if (slotB.currentAbsPos < 50 || slotB.currentAbsPos > 970) return slotB;
        return slotC;
    }
    public Slot getSlotAtOtherPos() {
        if (slotA.currentAbsPos < 690 && slotA.currentAbsPos > 670) return slotA;
        if (slotB.currentAbsPos < 690 && slotB.currentAbsPos > 570) return slotB;
        return slotC;
    }



    public boolean isNextSlotEmpty() {
         thisBall = getSlotAtCollectPos();

         if (Objects.equals(thisBall.name, "Slot A")) {
             return slotB.color.equals(BallColor.EMPTY);
         }

        if (Objects.equals(thisBall.name, "Slot B")) {
            return slotC.color.equals(BallColor.EMPTY);
        }
        if (Objects.equals(thisBall.name, "Slot C")) {
            return slotA.color.equals(BallColor.EMPTY);
        }

        return false;

    }



    public int getBestRotation() {
        List<BallColor> stateA = Arrays.asList(slotA.color, slotB.color, slotC.color);

        List<BallColor> stateB = Arrays.asList(slotB.color, slotC.color, slotA.color);

        List<BallColor> stateC = Arrays.asList(slotC.color, slotA.color, slotB.color);

        int scoreA = calculateScore(stateA, targetMotif);
        int scoreB = calculateScore(stateB, targetMotif);
        int scoreC = calculateScore(stateC, targetMotif);

        int targetShooterTick = 1024/3;
        int bestDelta = 0;

        if (scoreA >= scoreB && scoreA >= scoreC) {
            bestDelta = (targetShooterTick - slotA.currentAbsPos + 1024) % 1024;
        } else if (scoreB >= scoreC) {
            bestDelta = (targetShooterTick - slotB.currentAbsPos + 1024) % 1024;
        } else {
            bestDelta = (targetShooterTick - slotC.currentAbsPos + 1024) % 1024;
        }

        return bestDelta;


    }

    public int getPPGRotation() {
        List<BallColor> stateA = Arrays.asList(slotA.color, slotB.color, slotC.color);

        List<BallColor> stateB = Arrays.asList(slotB.color, slotC.color, slotA.color);

        List<BallColor> stateC = Arrays.asList(slotC.color, slotA.color, slotB.color);

        int scoreA = calculateScore(stateA, motif1);
        int scoreB = calculateScore(stateB, motif1);
        int scoreC = calculateScore(stateC, motif1);

        int targetShooterTick = 1024/3;
        int bestDelta = 0;

        if (scoreA >= scoreB && scoreA >= scoreC) {
            bestDelta = (targetShooterTick - slotA.currentAbsPos + 1024) % 1024;
        } else if (scoreB >= scoreC) {
            bestDelta = (targetShooterTick - slotB.currentAbsPos + 1024) % 1024;
        } else {
            bestDelta = (targetShooterTick - slotC.currentAbsPos + 1024) % 1024;
        }

        return bestDelta;


    }
    public int getBestPGPRotation() {
        List<BallColor> stateA = Arrays.asList(slotA.color, slotB.color, slotC.color);

        List<BallColor> stateB = Arrays.asList(slotB.color, slotC.color, slotA.color);

        List<BallColor> stateC = Arrays.asList(slotC.color, slotA.color, slotB.color);

        int scoreA = calculateScore(stateA, motif2);
        int scoreB = calculateScore(stateB, motif2);
        int scoreC = calculateScore(stateC, motif2);

        int targetShooterTick = 1024/3;
        int bestDelta = 0;

        if (scoreA >= scoreB && scoreA >= scoreC) {
            bestDelta = (targetShooterTick - slotA.currentAbsPos + 1024) % 1024;
        } else if (scoreB >= scoreC) {
            bestDelta = (targetShooterTick - slotB.currentAbsPos + 1024) % 1024;
        } else {
            bestDelta = (targetShooterTick - slotC.currentAbsPos + 1024) % 1024;
        }

        return bestDelta;


    }
    public int getBestGPPRotation() {
        List<BallColor> stateA = Arrays.asList(slotA.color, slotB.color, slotC.color);

        List<BallColor> stateB = Arrays.asList(slotB.color, slotC.color, slotA.color);

        List<BallColor> stateC = Arrays.asList(slotC.color, slotA.color, slotB.color);

        int scoreA = calculateScore(stateA, motif3);
        int scoreB = calculateScore(stateB, motif3);
        int scoreC = calculateScore(stateC, motif3);

        int targetShooterTick = 1024/3;
        int bestDelta = 0;

        if (scoreA >= scoreB && scoreA >= scoreC) {
            bestDelta = (targetShooterTick - slotA.currentAbsPos + 1024) % 1024;
        } else if (scoreB >= scoreC) {
            bestDelta = (targetShooterTick - slotB.currentAbsPos + 1024) % 1024;
        } else {
            bestDelta = (targetShooterTick - slotC.currentAbsPos + 1024) % 1024;
        }

        return bestDelta;


    }


    private int calculateScore(List<BallColor> slotSeq, List<BallColor> target) {
        int score = 0;
        for (int i = 0; i < 3; i++) {
            BallColor ball = slotSeq.get(i);
            BallColor goal = target.get(i);


            if (ball != BallColor.EMPTY && ball == goal) {
                score++;
            }
        }
        return score;
    }
    public void removeAll() {
        slotA.color = BallColor.valueOf("EMPTY");
        slotB.color = BallColor.valueOf("EMPTY");
        slotC.color = BallColor.valueOf("EMPTY");

    }

    public void addBall(BallColor color) {
        getSlotAtCollectPos().color = color;

    }
}