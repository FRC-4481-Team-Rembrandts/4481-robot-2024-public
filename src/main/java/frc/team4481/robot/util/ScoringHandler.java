package frc.team4481.robot.util;

public class ScoringHandler {
    public static ScoringHandler instance = null;
    private ScoringPosition scoringPosition = ScoringPosition.SPEAKER;


    public static ScoringHandler getInstance(){
        if(instance == null){
            instance = new ScoringHandler();
        }

        return instance;
    }

    public enum ScoringPosition {
        AMP,
        SPEAKER,
        LOPJE
    }

    public ScoringPosition getScoringPosition() {
        return scoringPosition;
    }

    public void setScoringPosition(ScoringPosition scoringPosition) {
        this.scoringPosition = scoringPosition;
    }
}
