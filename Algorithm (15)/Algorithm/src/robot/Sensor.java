package robot;

import arena.ArenaView;
import arena.GridBox;
import navigation.Direction;

import java.util.HashMap;

import static view.SimulatorGUI.arenaView;

public class  Sensor {

    int xPos;
    int yPos;
    int lowerLimit;
    int upperLimit;
    SensorLocation location;
    Direction direction;
    SensorRange range;
    boolean hitObstacle;
    boolean hitWall;
    int[] sense_XY = new int[2];



    public Sensor(int xPos, int yPos, SensorLocation location, SensorRange range, Direction direction) {
        this.xPos = xPos;
        this.yPos = yPos;

        switch (range){
            case SHORT:
//                if (location == SensorLocation.BOTTOM_LEFT){
//                    this.lowerLimit = 1;
//                    this.upperLimit = 3;
//                } else {
                    this.lowerLimit = 1;
                    this.upperLimit = 3;
//                }
                break;
            case LONG:
                this.lowerLimit = 1;
                this.upperLimit = 4;
        }

        this.location = location;
        this.range = range;
        this.hitObstacle = false;
        this.hitWall = false;
        this.direction = direction;
    }

    public enum SensorRange{
        SHORT,
        LONG,
    }
    public enum SensorLocation{
        LEFT_FRONT(0),
        MIDDLE_FRONT(1),
        RIGHT_FRONT(2),
        LEFT(3),
        BOTTOM_LEFT(4),
        RIGHT(5);


        int type_value = 0;

        private SensorLocation (int type_value){
            this.type_value = type_value;
        }

        public int getValue(){
            return this.type_value;
        }

        public void setValue(int value){
            this.type_value = value;
        }
    }

    public void setSensor(int row, int col, Direction direction) {
        this.xPos = col;
        this.yPos = row;
        this.direction = direction;
    }

    public boolean senseArena (){
        switch (direction) {
            case NORTH:
                return getSensorValue(-1, 0);
            case EAST:
                return getSensorValue(0, 1);
            case SOUTH:
                return getSensorValue(+1, 0);
            case WEST:
                return getSensorValue(0, -1);
        }
        return false;
    }

    private boolean getSensorValue(int rowInc, int colInc) {
//        // Check if starting point is valid for sensors with lowerRange > 1.
//        if (lowerLimit > 1) {
//            for (int i = 1; i < this.lowerLimit; i++) {
//                int row = this.yPos + (rowInc * i);
//                int col = this.xPos + (colInc * i);
//
//                if (!arenaView.checkValidCoordinates(row, col)) return false;
//                if (arenaView.getGrid(row, col).isObstacle()) return true;
//            }
//        }

        for (int i = this.lowerLimit; i <= this.upperLimit; i++) {
            int row = this.yPos + (rowInc * i);
            int col = this.xPos + (colInc * i);

            if (!arenaView.checkValidCoordinates(row, col)) return false;

            arenaView.setExploredGrids(row,col);
            if (arenaView.isSimulatedObstacle(row,col)) {
                //if (!arenaView.isInStartPoint(col, row) && !arenaView.isInGoalPoint(col, row)) {
                    arenaView.setIsObstacleGrids(row, col, true);
                    return true;
               // }
            }
        }
        return false;
    }

    public void senseRealArena(float sensorValue) {
        switch (direction) {
            case NORTH:
                processInputValue(sensorValue, -1, 0);
                break;
            case EAST:
                processInputValue(sensorValue, 0, 1);
                break;
            case SOUTH:
                processInputValue(sensorValue, +1, 0);
                break;
            case WEST:
                processInputValue(sensorValue, 0, -1);
                break;
        }

    }

    public void processInputValue(float inputValue, int rowInc, int colInc ){
        int sensorValue = 0;
        int reduceConfidenceBy = 0;  // the better the confidence, the lower this value is
        // confidence only applies to the grid that is 1 less from sensorValue & at the sensorValue itself
        /// for now, will only be used for 1 grid away - can only be affected by 1 grid and 2 grids away

        if (inputValue >= 3.1f && inputValue <= 8.5f){
            sensorValue = 1;
            if (inputValue > 7.0f)
                reduceConfidenceBy = 2;
            else if (inputValue < 4.0f || inputValue > 6.0f)
                reduceConfidenceBy = 1;
        }
        // 8 to 13 not detected, can calibrate.
//        else if (inputValue >= 11.2 && inputValue <= 18.8 && this.location == SensorLocation.LEFT){
//            sensorValue = 2;
//        }
        else if (inputValue >= 11.2f && inputValue <= 18.8f) {
            sensorValue = 2;
            if (inputValue > 18.0f || inputValue < 12.0f)
                reduceConfidenceBy = 2;
            else if (inputValue > 17.0f || inputValue < 13.0f)
                reduceConfidenceBy = 1;
        }
        else if (inputValue >= 20.7f && inputValue < 27.6f){// && inputValue < 28){
            sensorValue = 3;
//            if (inputValue < 22.0f)
//                reduceConfidenceBy = 2;
//            else if (inputValue > 27.0f || inputValue < 23.0f)
//                reduceConfidenceBy = 1;
        }
        else if (inputValue >=30.7f && inputValue< 39.2f){
            sensorValue = 4;
//            if (inputValue > 38.0f || inputValue < 32.0f)
//                reduceConfidenceBy = 2;
//            else if (inputValue > 37.0f || inputValue < 33.0f)
//                reduceConfidenceBy = 1;
        }
        else if (inputValue >= 40.8f && inputValue <49.3f) {
            sensorValue = 5;
//            if (inputValue > 48.0f || inputValue < 42.0f)
//                reduceConfidenceBy = 2;
//            else if (inputValue > 47.0f || inputValue < 43.0f)
//                reduceConfidenceBy = 1;
        }
        else if (inputValue <= 81.1)
            sensorValue = 6;


        //if (sensorValue == 0) return;  // return value for LR sensor if obstacle before lowerRange

//        System.out.println("In processInput Value");
//        System.out.println("Lower and upper limit " + lowerLimit + ", "+ upperLimit);
//        for (int j = this.lowerLimit; j <= this.upperLimit; j++) {
//            System.out.println("In for loop process input value");
//            int row = this.yPos + (rowInc * j);
//            int col = this.xPos + (colInc * j);
//            if (!arenaView.checkValidCoordinates(row, col)) continue;
//            arenaView.setExploredGrids(row,col);
//            arenaView.getGrid(row, col).setExplored(true);
//            System.out.println("In for loop process input value after");
//
//        }

        if (sensorValue == 0)   // blank range; don't plot
            return;

        // Update map according to sensor's value.

        for (int i = this.lowerLimit; i <= this.upperLimit; i++) {
            int row = this.yPos + (rowInc * i);
            int col = this.xPos + (colInc * i);
            //System.out.println("In Second for loop");
            if (!arenaView.checkValidCoordinates(row, col)) continue;

            // get reference to grid
            GridBox grid = arenaView.getGrid(row, col);

            // skip new reading if previous grid data has equivalent or better confidence, or was detected before island exploration (FP)
            // i.e. only continue to override previous grid data if current sensor reading has better confidence
            boolean higherConfidence = (i < grid.getDetectedAtGridAway()) ||    // grid is detected nearer
                    (i == grid.getDetectedAtGridAway() && reduceConfidenceBy < grid.getReducedConfidence()  // grid has better confidence
                            && i == 1);    // only applies to 1-grid away - comment away if applying confidence to all grids
            boolean fpCannotBeMoreConfident = !GridBox.IsBeforeFP() // robot is currently in island exploration
                    && grid.isDetectedBeforeFP();   // grid is detected during exploration loop, thus should not get overwritten
            if (fpCannotBeMoreConfident || !higherConfidence)   // confidence of sensor reading is lower than that of this grid
//            if ((i >= grid.getDetectedAtGridAway() && i != 1) ||   // current reading is equivalent or from further away
//               (!GridBox.IsBeforeFP() && grid.isDetectedBeforeFP())) // currently in island exploration, thus less confident than a grid detected before that
            {
                // if sensorValue == 0 as sensor reading was between ranges, skip (TBC)
                if (sensorValue >= i)   // obstacle detected, cannot sense beyond this grid
                    break;  // exit loop; unable to interpret any further grids
                continue;   // otherwise, just proceed to next grid
            }
            // update grid data confidence based on sensor's grid distance
            if (location.equals(SensorLocation.RIGHT) && i != 1)  // right sensor is sometimes inaccurate; make it less confident
                grid.setDetectedAtGridAway(i+1);
            else
                grid.setDetectedAtGridAway(i);
            grid.setDetectedBeforeFP(GridBox.IsBeforeFP()); // true if still in exploration loop
            grid.setReducedConfidence(reduceConfidenceBy);

            // set grid to be explored
            grid.setExplored(true);
            arenaView.setExploredGrids(row,col);

            // place an obstacle if it's not already there (if it's already there, nothing changes)
            if (sensorValue == i) { // obstacle detected
                //if (!arenaView.isInStartPoint(col, row) && !arenaView.isInGoalPoint(col, row)) {
                    arenaView.setIsObstacleGrids(row, col, true);
                    break;  // exit loop; unable to interpret any further grids
                //}
            }
            // Override previous obstacle value if front/side sensors detect no obstacle.
            // (sensor reading is definitely better confidence to even reach this stage)
            if (grid.isObstacle()) {
                if (location.equals(SensorLocation.LEFT_FRONT) || location.equals(SensorLocation.MIDDLE_FRONT) || location.equals(SensorLocation.RIGHT_FRONT) || location.equals(SensorLocation.LEFT) || location.equals(SensorLocation.BOTTOM_LEFT)) {
                    arenaView.setIsObstacleGrids(row, col,false);
                } else {
                    break;
                }
            }
        }
    }
}
