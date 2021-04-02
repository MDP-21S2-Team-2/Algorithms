//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package navigation;

import arena.ArenaLoader;
import arena.ArenaView;
import arena.GridBox;
import java.io.PrintStream;
import java.sql.SQLOutput;
import java.util.Stack;

import communications.TCPConstants;
import robot.Command;
import view.SimulatorGUI;

import static arena.ArenaLoader.explorationMapAndroid;
import static arena.ArenaLoader.generateMapDescriptor;
import static communications.TCPManager.tcp;
import static view.SimulatorGUI.arenaView;

public class ImageRecognition {
    private final int coverageLimit;
    private final int timeLimit;
    private int areaExplored;
    private long startTime;
    private long endTime;
    private int lastCalibrate;
    private boolean calibrationMode;
    private boolean realRun;
    private int loopCount = 1;
    private Command previousCommand;
    private boolean firstRun;
    private GridBox unexploredGrid;
    private boolean facedNorth = false;
    public ImageRecognition(int coverageLimit, int timeLimit, boolean realRun) {
        this.coverageLimit = coverageLimit;
        this.timeLimit = timeLimit;
        this.realRun = realRun;
        this.firstRun = true;
    }

    public void runExploration() {
        if (this.realRun) {
            System.out.println("Calibrated, Starting Exploration...");
        }

        System.out.println("Starting exploration...");
        this.startTime = System.currentTimeMillis();
        this.endTime = this.startTime + (long)(this.timeLimit * 1000);
        this.senseAndRepaint(firstRun, realRun);
        firstRun = false;
        arenaView.repaint();
        this.areaExplored = this.calculateAreaExplored();
        System.out.println("Explored Area: " + this.areaExplored);
        arenaView.calculateSpaceClearanceExploration();
        this.explorationLoop(SimulatorGUI.arenaView.robot.getCurrPosY(),SimulatorGUI.arenaView.robot.getCurrPosX());
        int count = 0;
        // exploration loop ended; FP begin
        GridBox.SetIslandExplorationBegin();    // exploration loop ended
        while (this.areaExplored < 300 && System.currentTimeMillis() <= this.endTime){
            if (!facedNorth){
                facedNorth = true;
                arenaView.robot.remote(Command.RIGHT,realRun);
            }
            if (!fastestPathToUnexploredCells()){
                count++;
                if (count >= 2)
                    break;
                continue;
            }
            count = 0;
            this.explorationLoopUnexploredAreas(SimulatorGUI.arenaView.robot.getCurrPosY(),SimulatorGUI.arenaView.robot.getCurrPosX());
        }
//        if (System.currentTimeMillis() < this.endTime)
        this.goHome();
        count = 0;
        while (System.currentTimeMillis() < this.endTime){

            if (this.explorationLoopLastResort(SimulatorGUI.arenaView.robot.getCurrPosY(),SimulatorGUI.arenaView.robot.getCurrPosX())){
                count++;
            }
            if (count >= 2) break;
        }
        if (System.currentTimeMillis() < this.endTime)
            this.goEnd();
        count = 0;
        while (System.currentTimeMillis() < this.endTime){
            if (this.explorationLoopLastResort(SimulatorGUI.arenaView.robot.getCurrPosY(),SimulatorGUI.arenaView.robot.getCurrPosX())){
                count ++;
            }
            if (count>=2) break;
        }
        System.out.println("Exploration ended");
        if (realRun){
            System.out.println("Output sent");
            String outputPacket = "IMG-DONE";
            tcp.sendPacket(outputPacket);
        }

    }

    private void senseAndRepaint(boolean firstRun, boolean realRun) {
        SimulatorGUI.arenaView.robot.setSensors();
        if (!firstRun || !realRun)
            SimulatorGUI.arenaView.robot.sense();
        SimulatorGUI.arenaView.repaint();
    }

    private void explorationLoop(int row, int col) {
        do {
            if(this.decideOnNextMove()) break;
            this.areaExplored = this.calculateAreaExplored();
            System.out.println("Area explored: " + this.areaExplored);
            if (SimulatorGUI.arenaView.robot.getCurrPosY()==row &&  SimulatorGUI.arenaView.robot.getCurrPosX()==col){
                if (areaExplored >=150){
//                    arenaView.robot.remote(Command.RIGHT,realRun);
                    break;
                }
            }
            else if (arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()].getEnteredCount() > 3) break;
        } while(this.areaExplored < this.coverageLimit && System.currentTimeMillis() <= this.endTime);

    }

    private void explorationNorthSideCorrection(){
        if (arenaView.gridArray[0][12].isObstacle()|| arenaView.gridArray[0][13].isObstacle() || arenaView.gridArray[0][14].isObstacle()){
            int obstacleCount = 0;
            for (int col = 0; col<=14; col++){
                if (arenaView.gridArray[0][col].isObstacle() &&  !arenaView.gridArray[1][col].isObstacle()){
                    obstacleCount++;
                }
                else obstacleCount = 0;

                if (obstacleCount > 3) {
                    for (int i = 0; i<obstacleCount; i++){
                        arenaView.gridArray[0][col-i].setExplored(true);
                        arenaView.gridArray[19][col-i].setObstacle(false);
                    }
                }
            }
            arenaView.repaint();
        }
    }

    private boolean explorationLoopLastResort(int row, int col) {
        arenaView.calculateSpaceClearanceExploration();
        String outputPacket = TCPConstants.SEND_RPI + TCPConstants.SEPARATOR + "DONE";
        tcp.sendPacket(outputPacket);
        tcp.receivePacket();
        do {
            if(this.decideOnNextMoveLastResort()) break;
            this.areaExplored = this.calculateAreaExplored();
            System.out.println("Area explored: " + this.areaExplored);
            if (SimulatorGUI.arenaView.robot.getCurrPosY()==row &&  SimulatorGUI.arenaView.robot.getCurrPosX()==col){
                if (areaExplored >=301){
                    arenaView.robot.remote(Command.RIGHT,realRun);
                    break;
                }
            }
            else if (arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()].getEnteredCount() > 2) break;
        } while(this.areaExplored < this.coverageLimit && System.currentTimeMillis() <= this.endTime);
        return true;

    }
    private void explorationLoopUnexploredAreas(int row, int col) {
        do {
            if(this.decideOnNextMove()) break;
            this.areaExplored = this.calculateAreaExplored();
            System.out.println("Area explored: " + this.areaExplored);
            if (SimulatorGUI.arenaView.robot.getCurrPosY()==row &&  SimulatorGUI.arenaView.robot.getCurrPosX()==col){
                if (areaExplored >=200){
                    break;
                }
            }
            else if (arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()].getEnteredCount() > 2) break;

        } while(this.areaExplored < this.coverageLimit && System.currentTimeMillis() <= this.endTime);

    }

    private boolean fastestPathToUnexploredCells(){
        arenaView.calculateSpaceClearance();
        GridBox currentGridBox = arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()];
        System.out.println("Current Gridbox X: "+ currentGridBox.getX() + " Y:"+currentGridBox.getY());
        GridBox gridNextToUnExplored = findGridNextToUnexploredCell();//arenaView.gridArray[findGridNextToUnexploredCell().getY()][findGridNextToUnexploredCell().getX()];
        if (gridNextToUnExplored == null){
            return false;
        }
        System.out.println("Nearest Grid Unxplored X: " + gridNextToUnExplored.getX() + " Y:" + gridNextToUnExplored.getY());
        navigation.FastestPath fp1 = new navigation.FastestPath(currentGridBox, gridNextToUnExplored);
        navigation.FastestPath fp2 = null;
        if (realRun){
            arenaView.robot.executeFastestPath(fp1.findFastestPath(),new Stack<GridBox>());
        }
        else{
            arenaView.robot.simulateFastestPath(fp1.findFastestPath(),new Stack<GridBox>());
        }
        makeRobotLeftFaceUnexploreGrid();
        //arenaView.robot.executeFastestPath(fp1.findFastestPath(),new Stack<GridBox>());
        System.out.println("Fastest Path to Unexplored Completed");
        arenaView.calculateSpaceClearanceExploration();
        tcp.sendPacket(tcp.updateAndroidExploration("RUNNING", arenaView.robot.getFacingDirection(), arenaView.robot.getCurrPosY(), arenaView.robot.getCurrPosX()) + "MDF," + explorationMapAndroid(arenaView));
        return true;
    }

    private void makeRobotLeftFaceUnexploreGrid(){
        Direction robotRequiredDirection = arenaView.robot.getFacingDirection();
        if (this.unexploredGrid.getY() > arenaView.robot.getCurrPosY()) {
            robotRequiredDirection = Direction.WEST;
        } else if (this.unexploredGrid.getY() < arenaView.robot.getCurrPosY()){
            robotRequiredDirection = Direction.EAST;
        } else if (this.unexploredGrid.getX() < arenaView.robot.getCurrPosX()){
            robotRequiredDirection = Direction.NORTH;
        } else if (this.unexploredGrid.getX() > arenaView.robot.getCurrPosX()){
            robotRequiredDirection = Direction.SOUTH;
        }
        turnBotDirection(robotRequiredDirection);
    }

    private GridBox findGridNextToUnexploredCell(){
        System.out.println("Start findGridNextToUnexploredCell");
        arenaView.gridsClearEnteredCount();

        GridBox currentGridBox = arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()];
        navigation.FastestPath fp1;
        GridBox gridNextToUnexplored = null;
        int pathSize = 0;

        outloop:
        for (int row = 0;  row <= 19 ; row++){
            for (int col = 0; col <=14; col++){
                if (!arenaView.gridArray[row][col].isExplored()) {
                    this.unexploredGrid = arenaView.gridArray[row][col];
                    //Top Grid First
                    if (arenaView.checkValidCoordinates(row-2, col)) {
                        if (arenaView.gridArray[row-2][col].isExplored() && arenaView.gridArray[row-2][col].isClearStatus() && !arenaView.gridArray[row-2][col].isObstacle()) {
                            fp1 = new navigation.FastestPath(currentGridBox, arenaView.getGrid(row-2, col));
                            if (!fp1.findFastestPath().isEmpty())
                                if (fp1.findFastestPath().size() < pathSize && pathSize != 0){
                                    gridNextToUnexplored = arenaView.getGrid(row-2, col);
                                    pathSize = fp1.findFastestPath().size();
                                } else if (pathSize == 0){
                                    gridNextToUnexplored = arenaView.getGrid(row-2, col);
                                    pathSize = fp1.findFastestPath().size();
                                }
                        }
                    }


                    //Left Grid first
                    if (arenaView.checkValidCoordinates(row, col - 2)) {
                        if (arenaView.gridArray[row][col -2].isExplored() && arenaView.gridArray[row][col -2].isClearStatus() && !arenaView.gridArray[row][col -2].isObstacle()) {
                            fp1 = new navigation.FastestPath(currentGridBox,arenaView.getGrid(row, col -2));
                            if (!fp1.findFastestPath().isEmpty())
                                if (fp1.findFastestPath().size() < pathSize && pathSize != 0) {
                                    gridNextToUnexplored = arenaView.getGrid(row, col - 2);
                                    pathSize = fp1.findFastestPath().size();
                                }else if (pathSize == 0){
                                    gridNextToUnexplored = arenaView.getGrid(row, col-2);
                                    pathSize = fp1.findFastestPath().size();
                                }
                        }
                    }
                    //Right Grid first
                    if (arenaView.checkValidCoordinates(row, col + 2)) {
                        if (arenaView.gridArray[row][col + 2].isExplored() && arenaView.gridArray[row][col + 2].isClearStatus() && !arenaView.gridArray[row][col +2].isObstacle()) {
                            fp1 = new navigation.FastestPath(currentGridBox,arenaView.getGrid(row, col + 2));
                            if (!fp1.findFastestPath().isEmpty())
                                if (fp1.findFastestPath().size() < pathSize && pathSize != 0) {
                                    gridNextToUnexplored = arenaView.getGrid(row, col + 2);
                                    pathSize = fp1.findFastestPath().size();
                                } else if (pathSize == 0){
                                    gridNextToUnexplored = arenaView.getGrid(row, col + 2);
                                    pathSize = fp1.findFastestPath().size();
                                }
                        }
                    }
                    //Bottom Grid first
                    if (arenaView.checkValidCoordinates(row+2, col)) {
                        if (arenaView.gridArray[row + 2][col].isExplored() && arenaView.gridArray[row+2][col].isClearStatus() && !arenaView.gridArray[row +2][col].isObstacle()) {
                            fp1 = new navigation.FastestPath(currentGridBox,arenaView.getGrid(row +2, col));
                            if (!fp1.findFastestPath().isEmpty())
                                if (fp1.findFastestPath().size() < pathSize && pathSize != 0) {
                                    gridNextToUnexplored = arenaView.getGrid(row + 2, col);
                                    pathSize = fp1.findFastestPath().size();
                                }else if (pathSize == 0){
                                    gridNextToUnexplored = arenaView.getGrid(row + 2, col);
                                    pathSize = fp1.findFastestPath().size();
                                }
                        }
                    }

                    if (arenaView.checkValidCoordinates(row+2, col)) {
                        if (arenaView.gridArray[row + 2][col].isExplored() && arenaView.gridArray[row+2][col].isClearStatus() && !arenaView.gridArray[row +2][col].isObstacle()) {
                            fp1 = new navigation.FastestPath(currentGridBox,arenaView.getGrid(row +2, col));
                            if (!fp1.findFastestPath().isEmpty())
                                if (fp1.findFastestPath().size() < pathSize && pathSize != 0) {
                                    gridNextToUnexplored = arenaView.getGrid(row + 2, col);
                                    pathSize = fp1.findFastestPath().size();
                                }else if (pathSize == 0){
                                    gridNextToUnexplored = arenaView.getGrid(row + 2, col);
                                    pathSize = fp1.findFastestPath().size();
                                }
                        }
                    }
                }
            }
        }
        if (gridNextToUnexplored != null){
            return gridNextToUnexplored;
        }
        System.out.println("No Fastest Path to Unexplored Areas");
        return null;
    }


    private boolean decideOnNextMove() {

        Command prevCommand = this.previousCommand;

        arenaView.robot.seeRightObstacle(realRun);
        //staircaseAvoidance();
        if (this.lookLeft(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX())) {
            this.moveBot(Command.LEFT);
            this.previousCommand = Command.LEFT;
            this.moveBot(Command.FORWARD);
            this.previousCommand = Command.FORWARD;
        } else if (this.lookForward(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX())) {
            this.moveBot(Command.FORWARD);
            this.previousCommand = Command.FORWARD;
        }
        else {
            this.moveBot(Command.RIGHT);
            this.previousCommand = Command.RIGHT;
        }

        return false;
    }

    private boolean decideOnNextMoveLastResort() {

        Command prevCommand = this.previousCommand;

        arenaView.robot.seeRightObstacle(realRun);
        //staircaseAvoidance();
        if (this.lookRight(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX())) {
            this.moveBot(Command.RIGHT);
            this.previousCommand = Command.RIGHT;
            this.moveBot(Command.FORWARD);
            this.previousCommand = Command.FORWARD;
        } else if (this.lookForward(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX())) {
            this.moveBot(Command.FORWARD);
            this.previousCommand = Command.FORWARD;
        } else {
            this.moveBot(Command.LEFT);
            this.previousCommand = Command.LEFT;
        }
        return false;
    }

    private void staircaseAvoidance(){
        int steps = 0;
        boolean stairsAlert = false;

        if (lookLeft(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX())) {
            int row = arenaView.robot.getCurrPosY();
            int col = arenaView.robot.getCurrPosX();
            switch (arenaView.robot.getFacingDirection()) {
                case NORTH:
                    if (arenaView.getIsObstacle(row+1,col-3) && arenaView.getIsClear(row,col-3) && arenaView.getIsClear(row-1,col-3)){
                        if (lookForward(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX()))
                            stairsAlert = true;

                    }
                    break;
                case SOUTH:
                    if (arenaView.getIsObstacle(row-1,col+3) && arenaView.getIsClear(row,col+3) && arenaView.getIsClear(row+1,col+3)){
                        if (lookForward(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX()))
                            stairsAlert = true;
                    }
                    break;
                case EAST:
                    if (arenaView.getIsObstacle(row-3,col-1) && arenaView.getIsClear(row-3,col) && arenaView.getIsClear(row-3,col+1)) {
                        if (lookForward(arenaView.robot.getCurrPosY(), arenaView.robot.getCurrPosX()))
                            stairsAlert = true;
                    }
                    break;
                case WEST:
                    if (arenaView.getIsObstacle(row+3,col+1) && arenaView.getIsClear(row+3,col) && arenaView.getIsClear(row+3,col-1)){
                        if (lookForward(arenaView.robot.getCurrPosY(),arenaView.robot.getCurrPosX()))
                            stairsAlert = true;
                    }
                    break;
            }
            if (stairsAlert){
                this.moveBot(Command.FORWARD);
                this.moveBot(Command.LEFT);
                this.moveBot(Command.FORWARD);
                if (this.lookForward(arenaView.robot.getCurrPosY(), arenaView.robot.getCurrPosX()))
                    this.moveBot(Command.FORWARD);
                //this.moveBot(Command.FORWARD2);
            }
        }
    }

    private boolean lookRight(int robotRow, int robotCol) {
        switch(SimulatorGUI.arenaView.robot.getFacingDirection()) {
            case NORTH:
                return this.checkEast(robotRow,robotCol);
            case EAST:
                return this.checkSouth(robotRow,robotCol);
            case SOUTH:
                return this.checkWest(robotRow,robotCol);
            case WEST:
                return this.checkNorth(robotRow,robotCol);
            default:
                return false;
        }
    }

    private boolean lookForward(int robotRow, int robotCol) {
        switch(SimulatorGUI.arenaView.robot.getFacingDirection()) {
            case NORTH:
                return this.checkNorth(robotRow,robotCol);
            case EAST:
                return this.checkEast(robotRow,robotCol);
            case SOUTH:
                return this.checkSouth(robotRow,robotCol);
            case WEST:
                return this.checkWest(robotRow,robotCol);
            default:
                return false;
        }
    }

    private boolean lookLeft(int robotRow, int robotCol) {
        switch(SimulatorGUI.arenaView.robot.getFacingDirection()) {
            case NORTH:
                return this.checkWest(robotRow,robotCol);
            case EAST:
                return this.checkNorth(robotRow,robotCol);
            case SOUTH:
                return this.checkEast(robotRow,robotCol);
            case WEST:
                return this.checkSouth(robotRow,robotCol);
            default:
                return false;
        }
    }

    private boolean checkNorth(int robotRow, int robotCol) {
        return this.isExploredNotObstacle(robotRow - 2, robotCol - 1) && this.isExploredAndFree(robotRow - 2, robotCol) && this.isExploredNotObstacle(robotRow - 2, robotCol + 1);
    }

    private boolean checkEast(int robotRow, int robotCol) {
        return this.isExploredNotObstacle(robotRow - 1, robotCol + 2) && this.isExploredAndFree(robotRow, robotCol + 2) && this.isExploredNotObstacle(robotRow + 1, robotCol + 2);
    }

    private boolean checkSouth(int robotRow, int robotCol) {
        return this.isExploredNotObstacle(robotRow + 2, robotCol + 1) && this.isExploredAndFree(robotRow + 2, robotCol) && this.isExploredNotObstacle(robotRow + 2, robotCol - 1);
    }

    private boolean checkWest(int robotRow, int robotCol) {
        return this.isExploredNotObstacle(robotRow + 1, robotCol - 2) && this.isExploredAndFree(robotRow, robotCol - 2) && this.isExploredNotObstacle(robotRow - 1, robotCol - 2);
    }

    private void goHome() {
        System.out.println("\nExploration complete!");
        //System.out.println(ArenaLoader.binToHex(explorationMapAndroid(arenaView)));
        System.out.println(generateMapDescriptor(arenaView));
        //String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredArena);
        this.areaExplored = this.calculateAreaExplored();
        System.out.printf("%.2f%% Coverage", (double)this.areaExplored / 300.0D * 100.0D);
        System.out.println(", " + this.areaExplored + " Cells");
        PrintStream var10000 = System.out;
        long var10001 = System.currentTimeMillis() - this.startTime;
        var10000.println(var10001 / 1000L + " Seconds");
        if (!this.realRun) {
            int row = SimulatorGUI.arenaView.robot.getCurrPosY();
            int col = SimulatorGUI.arenaView.robot.getCurrPosX();
            ArenaView var10002 = SimulatorGUI.arenaView;
            GridBox var6 = ArenaView.gridArray[row][col];
            ArenaView var10003 = SimulatorGUI.arenaView;
            arenaView.calculateSpaceClearance();
            FastestPath toHome = new FastestPath(var6, ArenaView.gridArray[18][1]);
            SimulatorGUI.arenaView.robot.simulateFastestPath(toHome.findFastestPath(),null);
            System.out.println("goHome() Simulator fastest path");
        } else {
            arenaView.calculateSpaceClearance();
            GridBox currentGridBox = arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()];
            GridBox startPoint = arenaView.gridArray[18][1];
            FastestPath fp1 = new FastestPath(currentGridBox,startPoint);
            System.out.println("Fastest Path Computed");

            SimulatorGUI.arenaView.robot.executeFastestPath(fp1.findFastestPath(),null);
            System.out.println("goHome() real run fastest path");
        }
        //this.turnBotDirection(Direction.NORTH);
    }

    private void goEnd() {
        System.out.println("\nExploration Last Resort, going end zone!");

        System.out.println(generateMapDescriptor(arenaView));
        //String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredArena);
        this.areaExplored = this.calculateAreaExplored();
        System.out.printf("%.2f%% Coverage", (double)this.areaExplored / 300.0D * 100.0D);
        System.out.println(", " + this.areaExplored + " Cells");
        PrintStream var10000 = System.out;
        long var10001 = System.currentTimeMillis() - this.startTime;
        var10000.println(var10001 / 1000L + " Seconds");
        if (!this.realRun) {
            int row = SimulatorGUI.arenaView.robot.getCurrPosY();
            int col = SimulatorGUI.arenaView.robot.getCurrPosX();
            ArenaView var10002 = SimulatorGUI.arenaView;
            GridBox var6 = ArenaView.gridArray[row][col];
            ArenaView var10003 = SimulatorGUI.arenaView;
            arenaView.calculateSpaceClearance();
            FastestPath toHome = new FastestPath(var6, ArenaView.gridArray[1][13]);
            SimulatorGUI.arenaView.robot.simulateFastestPath(toHome.findFastestPath(),null);
            System.out.println("goHome() Simulator fastest path");
        } else {
            arenaView.calculateSpaceClearance();
            GridBox currentGridBox = arenaView.gridArray[arenaView.robot.getCurrPosY()][arenaView.robot.getCurrPosX()];
            GridBox startPoint = arenaView.gridArray[18][1];
            FastestPath fp1 = new FastestPath(currentGridBox,startPoint);
            System.out.println("Fastest Path Computed");
            SimulatorGUI.arenaView.robot.executeFastestPath(fp1.findFastestPath(),null);
            System.out.println("goHome() real run fastest path");
        }
        this.turnBotDirection(Direction.SOUTH);
    }

    private boolean isExploredNotObstacle(int row, int col) {
        if (!SimulatorGUI.arenaView.checkValidCoordinates(row, col)) {
            return false;
        } else {
            GridBox grid = SimulatorGUI.arenaView.getGrid(row, col);

            return grid.isExplored() && !arenaView.gridArray[row][col].isObstacle();
        }
    }

    private boolean isExploredAndFree(int row, int col) {
        if (!SimulatorGUI.arenaView.checkValidCoordinates(row, col)) {
            return false;
        } else {
            GridBox grid = SimulatorGUI.arenaView.getGrid(row, col);
            return grid.isExplored() && grid.isClearStatus() && !grid.isObstacle();
        }
    }

    private int calculateAreaExplored() {
        int result = 0;

        for(int row = 0; row < 20; ++row) {
            for(int col = 0; col < 15; ++col) {
                if (SimulatorGUI.arenaView.getGrid(row, col).isExplored()) {
                    ++result;
                }
            }
        }

        return result;
    }

    private void moveBot(Command m) {
        int delay;
        if (realRun)
            delay = 0;
        else
            delay = 300;
        try {
            Thread.sleep(delay);
            SimulatorGUI.arenaView.robot.remote(m,realRun);
            SimulatorGUI.arenaView.repaint();

        } catch (InterruptedException var3) {
            var3.printStackTrace();
        }

    }

    private void turnBotDirection(Direction targetDirection) {
        int numOfTurn = Math.abs(SimulatorGUI.arenaView.robot.getFacingDirection().ordinal() - targetDirection.ordinal());
        if (numOfTurn > 2) {
            numOfTurn %= 2;
        }

        if (numOfTurn == 1) {
            if (Direction.getNextDirectionClockwise(SimulatorGUI.arenaView.robot.getFacingDirection()) == targetDirection) {
                this.moveBot(Command.RIGHT);
            } else {
                this.moveBot(Command.LEFT);
            }
        } else if (numOfTurn == 2) {
            this.moveBot(Command.RIGHT);
            this.moveBot(Command.RIGHT);
        }
    }
}
