//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package arena;

import java.io.PrintStream;
import java.util.ArrayList;
import navigation.Direction;
import navigation.Location;

public class GridBox implements Comparable {
    private Location location;
    private boolean isObstacle;
    private boolean isObstacleNeighbour;
    private boolean isExplored;
    private boolean clearStatus;
    private Direction robotFacingDirection;
    private GridBox pathParent = null;
    private GridBox gridAbove;
    private GridBox gridBelow;
    private GridBox gridLeft;
    private GridBox gridRight;
    public boolean northFaceSeen = false;
    public boolean southFaceSeen = false;
    public boolean eastFaceSeen = false;
    public boolean westFaceSeen = false;
    private ArrayList<GridBox> neighbouringGridBoxes = new ArrayList();
    private float costAccumulatedFromStart;
    private float costEstimatedToGoal;

    private int enteredCount = 0;

    // variables for sensor reading confidence for this grid
    private int detectedAtGridAway = 10;    // default: a large max value so any sensor reading is definitely smaller
    public int getDetectedAtGridAway() { return detectedAtGridAway; }
    public void setDetectedAtGridAway(int grid) { detectedAtGridAway = grid; }
    private int reducedConfidence = 0;     // most confident = 0; lower confidence = higher value
    public int getReducedConfidence() { return reducedConfidence; }
    public void setReducedConfidence(int confidence) { reducedConfidence = confidence; }
    private boolean detectedBeforeFP = false;   // default so island overwriting can trigger
    public boolean isDetectedBeforeFP() { return detectedBeforeFP; }
    public void setDetectedBeforeFP(boolean isBeforeFP) { detectedBeforeFP = isBeforeFP; }

    private static boolean isBeforeFP = true;
    public static boolean IsBeforeFP() { return isBeforeFP; }
    public static void SetIslandExplorationBegin() { isBeforeFP = false; }

    public GridBox(Location location, boolean isObstacle) {
        this.location = location;
        this.isObstacle = isObstacle;
        this.isExplored = false;
    }
    public void setFaceSeen (Direction direction, boolean seen){
        switch (direction){
            case NORTH:
                this.northFaceSeen = seen;
                break;
            case SOUTH:
                this.southFaceSeen = seen;
                break;
            case EAST:
                this.eastFaceSeen = seen;
                break;
            case WEST:
                this.westFaceSeen = seen;
        }
    }
    public int compareTo(Object o) {
        float currentGridCost = this.getCost();
        float otherGridCost = ((GridBox)o).getCost();
        float costDifference = currentGridCost - otherGridCost;
        if (costDifference > 0.0F) {
            return 1;
        } else {
            return costDifference < 0.0F ? -1 : 0;
        }
    }

    public float getCost() {
        float gridCost = this.costAccumulatedFromStart + this.costEstimatedToGoal;
        return gridCost;
    }

    public float estimateCostToGrid(GridBox goal) {
        float xDifference = (float)Math.abs(this.getX() - goal.getX());
        float yDifference = (float)Math.abs(this.getY() - goal.getY());
        return xDifference + yDifference;
    }

    public float getCost(GridBox neighbouringGrid, GridBox endGrid, boolean isStartGrid) {
        return this.costAccumulatedFromStart + this.getWeight(neighbouringGrid, isStartGrid);
    }

    public int compareX(GridBox grid) {
        return grid.getX() > this.getX() ? 1 : (grid.getX() < this.getX() ? -1 : 0);
    }

    public int compareY(GridBox grid) {
        return grid.getY() > this.getY() ? 1 : (grid.getY() < this.getY() ? -1 : 0);
    }

    private float getWeight(GridBox neighbouringGrid, boolean isStartGrid) {
        this.setRobotFacingDirection();
        return (((this.compareX(neighbouringGrid) == 1 && this.robotFacingDirection == Direction.EAST) ||
                (this.compareX(neighbouringGrid) == -1 && this.robotFacingDirection == Direction.WEST) ||
                (this.compareY(neighbouringGrid) == -1 && this.robotFacingDirection == Direction.SOUTH) ||
                (this.compareY(neighbouringGrid) == 1 && this.robotFacingDirection == Direction.NORTH))) ? 50F : 300F;
    }

    public void setRobotFacingDirection() {
        if (this.pathParent == null) {
            //this.robotFacingDirection = Direction.NORTH;
        } else {
            if (this.compareX(this.pathParent) == 1) {
                this.robotFacingDirection = Direction.WEST;
            } else if (this.compareX(this.pathParent) == -1) {
                this.robotFacingDirection = Direction.EAST;
            } else if (this.compareY(this.pathParent) == 1) {
                this.robotFacingDirection = Direction.SOUTH;
            } else if (this.compareY(this.pathParent) == -1) {
                this.robotFacingDirection = Direction.NORTH;
            }

        }
    }

    public Location getLocation() {
        return this.location;
    }

    public int getX() {
        return this.location.getX();
    }

    public int getY() {
        return this.location.getY();
    }

    public boolean isObstacle() {
        return this.isObstacle;
    }

    public void setObstacle(boolean isObstacle) {
        this.isObstacle = isObstacle;
    }

    public boolean isObstacleNeighbour() {
        return this.isObstacleNeighbour;
    }

    public void setObstacleNeighbour(boolean isObstacleNeighbour) {
        this.isObstacleNeighbour = isObstacleNeighbour;
    }

    public void setNeighbouringGridBoxes(GridBox gridBox) {
        this.neighbouringGridBoxes.add(gridBox);
    }

    public GridBox getGridAbove() {
        return this.gridAbove;
    }

    public void setGridAbove(GridBox gridAbove) {
        this.gridAbove = gridAbove;
    }

    public GridBox getGridBelow() {
        return this.gridBelow;
    }

    public void setGridBelow(GridBox gridBelow) {
        this.gridBelow = gridBelow;
    }

    public GridBox getGridLeft() {
        return this.gridLeft;
    }

    public void setGridLeft(GridBox gridLeft) {
        this.gridLeft = gridLeft;
    }

    public GridBox getGridRight() {
        return this.gridRight;
    }

    public void setGridRight(GridBox gridRight) {
        this.gridRight = gridRight;
    }

    public float getCostAccumulatedFromStart() {
        return this.costAccumulatedFromStart;
    }

    public void setCostAccumulatedFromStart(float costAccumulatedFromStart) {
        this.costAccumulatedFromStart = costAccumulatedFromStart;
    }

    public float getCostEstimatedToGoal() {
        return this.costEstimatedToGoal;
    }

    public void setCostEstimatedToGoal(float costEstimatedToGoal) {
        this.costEstimatedToGoal = costEstimatedToGoal;
    }

    public GridBox getPathParent() {
        return this.pathParent;
    }

    public void setPathParent(GridBox pathParent) {
        this.pathParent = pathParent;
    }

    public Direction getRobotFacingDirection() {
        return this.robotFacingDirection;
    }

    public void setRobotFacingDirection(Direction robotFacingDirection) {
        this.robotFacingDirection = robotFacingDirection;
    }

    public ArrayList<GridBox> getNeighbouringGridBoxes() {
        return this.neighbouringGridBoxes;
    }

    public void setNeighbouringGridBoxes(ArrayList<GridBox> neighbouringGridBoxes) {
        this.neighbouringGridBoxes = neighbouringGridBoxes;
    }

    public void printGridInfo() {
        PrintStream var10000 = System.out;
        int var10001 = this.getX();
        var10000.println(var10001 + "," + this.getY());
        System.out.println("Neighbour size:" + this.getNeighbouringGridBoxes().size() + "\n");
    }

    public boolean isClearStatus() {
        return this.clearStatus;
    }

    public void setClearStatus(boolean clearStatus) {
        this.clearStatus = clearStatus;
    }

    public boolean isExplored() {
        return this.isExplored;
    }

    public void setExplored(boolean explored) {
        this.isExplored = explored;
    }
    public int getEnteredCount() {
        return enteredCount;
    }

    public void setEnteredCount(int enteredCount) {
        this.enteredCount = enteredCount;
    }
}
