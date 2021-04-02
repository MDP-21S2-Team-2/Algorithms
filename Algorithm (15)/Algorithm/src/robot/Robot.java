package robot;

import arena.ArenaConstants;
import arena.ArenaView;
import arena.GridBox;
import communications.TCPConstants;
import communications.TCPManager;
import navigation.Direction;
import navigation.Location;
import view.SimulatorGUI;

import javax.swing.*;
import java.util.Optional;
import java.util.Stack;

import static communications.TCPConstants.*;
import static communications.TCPManager.tcp;
import static java.lang.Float.parseFloat;
import static view.SimulatorGUI.arenaView;
import static arena.ArenaLoader.explorationMapAndroid;

public class Robot {
    private GridBox currentGridBox = new GridBox(new Location(RobotConstants.START_POSX, RobotConstants.START_POSY),false);
    private int currPosX = RobotConstants.START_POSX;
    private int currPosY = RobotConstants.START_POSY;

    private Location exactLocation;
    private Direction facingDirection = RobotConstants.INITIAL_FACING_DIRECTION;
    private boolean stopSignal = false;
    private Sensor BLSensor = new Sensor(RobotConstants.START_POSX-1,RobotConstants.START_POSY,Sensor.SensorLocation.BOTTOM_LEFT, Sensor.SensorRange.SHORT,Direction.WEST);
    private Sensor LSensor = new Sensor(RobotConstants.START_POSX-1,RobotConstants.START_POSY-1,Sensor.SensorLocation.LEFT, Sensor.SensorRange.SHORT,Direction.WEST);
    private Sensor LFSensor = new Sensor(RobotConstants.START_POSX-1,RobotConstants.START_POSY-1,Sensor.SensorLocation.LEFT_FRONT, Sensor.SensorRange.SHORT,Direction.NORTH);
    private Sensor MFSensor = new Sensor(RobotConstants.START_POSX,RobotConstants.START_POSY-1,Sensor.SensorLocation.MIDDLE_FRONT, Sensor.SensorRange.SHORT,Direction.NORTH);
    private Sensor RFSensor = new Sensor(RobotConstants.START_POSX+1,RobotConstants.START_POSY-1,Sensor.SensorLocation.RIGHT_FRONT, Sensor.SensorRange.SHORT,Direction.NORTH);
    private Sensor RSensor =  new Sensor(RobotConstants.START_POSX+1,RobotConstants.START_POSY-1,Sensor.SensorLocation.RIGHT, Sensor.SensorRange.LONG,Direction.EAST);

    private boolean simulatedRun;

    public Robot(boolean simulatedRun) {
        this.simulatedRun = simulatedRun;
    }

    public void remote (Command command,boolean realRun) {//}, Optional<Integer> steps) {
        //int s = steps.isPresent() ? steps.get() : 0;
        //int delay = 100;
        if (!stopSignal){
            if (command == Command.FORWARD) {
                if (realRun)
                    if (checkRobotMovingAlongWalls())
                        tcp.sendPacket(SEND_ARDUINO + SEPARATOR + "W");
                    else{
                        tcp.sendPacket(SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + 0);
                        move(command, 1);
                    }
                //tcp.receiveBytesArduino();
                //sendArduinoMultipleForward(1,100);
            }
            else if (command == Command.FORWARD2){
                move(command, 2);
                if (realRun)
                    if (checkRobotMovingAlongWalls())
                        tcp.sendPacket(SEND_ARDUINO + SEPARATOR + "W");
                    else
                        tcp.sendPacket(SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + 1);
            }
            else {
                rotate(command);
                if (realRun) {
                    expSendArduinoRotate(command, 0);
                }
            }
        setSensors();
        sense();
        //arenaView.repaint();
        if (realRun)
            tcp.sendPacket(tcp.updateAndroidExploration("RUNNING", this.getFacingDirection(), this.getCurrPosY(), this.getCurrPosX()) + "MDF," + explorationMapAndroid(arenaView));
        setGridsFaceSeens();
    }
    }

    public void setGridsFaceSeens(){
        int rowOffset = 0;
        int colOffset = 0;
        int rowOffset2 = 0;
        int colOffset2 = 0;
        switch (this.facingDirection){
            case NORTH:
                colOffset = -2;
                colOffset2 = -3;
                break;
            case SOUTH:
                colOffset = 2;
                colOffset2 = 3;
                break;
            case EAST:
                rowOffset = -2;
                rowOffset2 = -3;
                break;
            case WEST:
                rowOffset = 2;
                rowOffset2 = 3;
                break;
        }

        if (arenaView.checkValidCoordinates(currPosY+rowOffset,currPosX+colOffset)){
            if (arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].isObstacle()){
                switch (this.facingDirection){
                    case NORTH:
                        arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].setFaceSeen(Direction.EAST,true);
                        break;
                    case SOUTH:
                        arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].setFaceSeen(Direction.WEST,true);
                        break;
                    case EAST:
                        arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].setFaceSeen(Direction.SOUTH,true);
                        break;
                    case WEST:
                        arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].setFaceSeen(Direction.NORTH,true);
                        break;
                }
            }
        }

        if (arenaView.checkValidCoordinates(currPosY+rowOffset2,currPosX+colOffset2)){
            if (arenaView.gridArray[currPosY+rowOffset2][currPosX+colOffset2].isObstacle()){
                switch (this.facingDirection){
                    case NORTH:
                        arenaView.gridArray[currPosY+rowOffset2][currPosX+colOffset2].setFaceSeen(Direction.EAST,true);
                        break;
                    case SOUTH:
                        arenaView.gridArray[currPosY+rowOffset2][currPosX+colOffset2].setFaceSeen(Direction.WEST,true);
                        break;
                    case EAST:
                        arenaView.gridArray[currPosY+rowOffset2][currPosX+colOffset2].setFaceSeen(Direction.SOUTH,true);
                        break;
                    case WEST:
                        arenaView.gridArray[currPosY+rowOffset2][currPosX+colOffset2].setFaceSeen(Direction.NORTH,true);
                        break;
                }
            }
        }
        //forloop to set faceseens for obstacles beside wall and obstacles beside each other
        for(int row = 0; row < ArenaConstants.NUM_ROWS; ++row) {
            for(int col = 0; col < ArenaConstants.NUM_COLS; ++col) {
                if (arenaView.gridArray[row][col].getGridAbove() == null || arenaView.gridArray[row][col].getGridAbove().isObstacle()){
                    arenaView.gridArray[row][col].setFaceSeen(Direction.NORTH,true);
                }
                if (arenaView.gridArray[row][col].getGridBelow() == null || arenaView.gridArray[row][col].getGridBelow().isObstacle()){
                    arenaView.gridArray[row][col].setFaceSeen(Direction.SOUTH,true);
                }
                if (arenaView.gridArray[row][col].getGridRight() == null || arenaView.gridArray[row][col].getGridRight().isObstacle()){
                    arenaView.gridArray[row][col].setFaceSeen(Direction.EAST,true);
                }
                if (arenaView.gridArray[row][col].getGridLeft() == null || arenaView.gridArray[row][col].getGridLeft().isObstacle()){
                    arenaView.gridArray[row][col].setFaceSeen(Direction.WEST,true);
                }
            }
        }
    }

    public void seeRightObstacle(boolean realRun){
        int rowOffset = 0;
        int colOffset = 0;
        int rowOffset2 = 0;
        int colOffset2 = 0;
        switch (this.facingDirection){
            case NORTH:
                colOffset = 2;
                colOffset2 = 3;
                break;
            case SOUTH:
                colOffset = -2;
                colOffset2 = -3;
                break;
            case EAST:
                rowOffset = 2;
                rowOffset2 = 3;
                break;
            case WEST:
                rowOffset = -2;
                rowOffset2 = -3;
                break;
        }

        if (arenaView.checkValidCoordinates(currPosY+rowOffset,currPosX+colOffset)){
            if (arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].isObstacle()){
                switch (this.facingDirection){
                    case NORTH:
                        if (!arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].westFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].westFaceSeen = true;
                            try{
                                Thread.sleep(100);
                                remote(Command.TURN180, realRun);
                            } catch (InterruptedException e){
                                e.printStackTrace();
                            }
                        }
                        break;
                    case SOUTH:
                        if (!arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].eastFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].eastFaceSeen = true;
                            try{
                                Thread.sleep(100);
                                remote(Command.TURN180, realRun);
                            } catch (InterruptedException e){
                                e.printStackTrace();
                            }
                        }
                        break;
                    case EAST:
                        if (!arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].northFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].northFaceSeen = true;
                            try{
                                Thread.sleep(100);
                                remote(Command.TURN180, realRun);
                            } catch (InterruptedException e){
                                e.printStackTrace();
                            }
                        }
                        break;
                    case WEST:
                        if (!arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].southFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY+rowOffset][currPosX+colOffset].southFaceSeen = true;
                            try{
                            Thread.sleep(100);
                            remote(Command.TURN180, realRun);
                            } catch (InterruptedException e){
                            e.printStackTrace();
                            }
                        }
                        break;
                }
//                try{
//                    Thread.sleep(100);
//                    remote(Command.TURN180, realRun);
//                } catch (InterruptedException e){
//                    e.printStackTrace();
//                }
            }
        }

        if (arenaView.checkValidCoordinates(currPosY+rowOffset2,currPosX+colOffset2)) {
            if (arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].isObstacle()) {
                switch (this.facingDirection) {
                    case NORTH:
                        if (!arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].westFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].westFaceSeen = true;
                            remote(Command.TURN180, realRun);
                        }
                        break;
                    case SOUTH:
                        if (!arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].eastFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].eastFaceSeen = true;
                            remote(Command.TURN180, realRun);
                        }
                        break;
                    case EAST:
                        if (!arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].northFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].northFaceSeen = true;
                            remote(Command.TURN180, realRun);
                        }
                        break;
                    case WEST:
                        if (!arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].southFaceSeen) {
                            remote(Command.TURN180, realRun);
                            arenaView.gridArray[currPosY + rowOffset2][currPosX + colOffset2].southFaceSeen = true;
                            remote(Command.TURN180, realRun);
                        }
                        break;
                }
            }
        }
    }

    public boolean checkRobotMovingAlongWalls(){
        switch (this.facingDirection){
            case NORTH:
                if (currPosX - 1 == 0) return true;
                break;
            case SOUTH:
                if (currPosX + 1 == 14) return true;
                break;
            case EAST:
                if (currPosY - 1 == 0) return true;
                break;
            case WEST:
                if (currPosY + 1 == 19) return true;
                break;
        }
        return false;
    }
    public void move(Command command, int steps) {
        int s = (command == Command.FORWARD || command == Command.FORWARD2) ? steps : -steps;

        switch (this.facingDirection){
            case NORTH:
                currPosY -= s;
                break;
            case SOUTH:
                currPosY += s;
                break;
            case EAST:
                currPosX += s;
                break;
            case WEST:
                currPosX -= s;
                break;
        }
        arenaView.gridArray[currPosY][currPosX].setEnteredCount(arenaView.gridArray[currPosY][currPosX].getEnteredCount()+1);
        arenaView.repaint();
    }

    //Depending on command left or right that is passed in, this.facingDirection will change
    public void rotate(Command command) {
        if (command == Command.RIGHT) {
            switch (this.facingDirection){
                case NORTH:
                    this. facingDirection = Direction.EAST;
                    break;
                case EAST:
                    this.facingDirection = Direction.SOUTH;
                    break;
                case SOUTH:
                    this.facingDirection = Direction.WEST;
                    break;
                case WEST:
                    this.facingDirection = Direction.NORTH;
            }
        }
        else if (command == Command.LEFT){
            switch (this.facingDirection){
                case NORTH:
                    this. facingDirection = Direction.WEST;
                    break;
                case EAST:
                    this.facingDirection = Direction.NORTH;
                    break;
                case SOUTH:
                    this.facingDirection = Direction.EAST;
                    break;
                case WEST:
                    this.facingDirection = Direction.SOUTH;
            }
        }
        else {
            switch (this.facingDirection){
                case NORTH:
                    this. facingDirection = Direction.SOUTH;
                    break;
                case EAST:
                    this.facingDirection = Direction.WEST;
                    break;
                case SOUTH:
                    this.facingDirection = Direction.NORTH;
                    break;
                case WEST:
                    this.facingDirection = Direction.EAST;
            }
        }
    }

    public void calibrate(){}

    public GridBox getCurrentGridCell() {
        return currentGridBox;
    }

    public Location getExactLocation() {
        //exactLocation.setX(currPosX);
        //exactLocation.setY(currPosY);
        return exactLocation;
    }

    public void senseFront() {

    }

    public void senseLeft() {

    }

    public void senseRight() {

    }

    public Direction getHeading() {
        return facingDirection;
    }

    public void simulateFastestPath(Stack<GridBox> path1, Stack<GridBox> path2 ){
        arenaView.generateFastestPathObstacles();
        arenaView.repaint();
        System.out.println("Simulating fastestPath");
        while (!path1.empty()){
            GridBox grid;
                grid = path1.pop();


            int x = grid.getX();
            int y = grid.getY();
            arenaView.gridArray[y][x].setExplored(true);

            for(int i = y - 1; i <= y + 1; ++i) {
                for(int j = x - 1; j <= x + 1; ++j) {
                    if (i < 20 && i >= 0 && j < 15 && j >= 0) {
                        arenaView.gridArray[i][j].setExplored(true);
                    }
                }
            }

            try{
                Thread.sleep(100);
                if (grid.getX() > currPosX){
                    while(facingDirection != Direction.EAST){
                        if (facingDirection != Direction.SOUTH) {
                            this.rotate(Command.RIGHT);
                            System.out.println(Command.RIGHT);
                        }else {
                            this.rotate(Command.LEFT);
                            System.out.println(Command.LEFT);
                        }
                        senseDuringFastestPath();
                    }

                    move(Command.FORWARD,1);
                    senseDuringFastestPath();
                    System.out.println(Command.FORWARD);
                }
                else if (grid.getX()<currPosX){
                    while(facingDirection != Direction.WEST) {
                        if (facingDirection != Direction.SOUTH) {
                            this.rotate(Command.LEFT);
                            System.out.println(Command.LEFT);
                        } else{
                            this.rotate(Command.RIGHT);
                            System.out.println(Command.RIGHT);
                        }
                        senseDuringFastestPath();
                    }
                    move(Command.FORWARD,1);
                    senseDuringFastestPath();
                    System.out.println(Command.FORWARD);
                }
                else if (grid.getY()<currPosY){
                    while(facingDirection != Direction.NORTH){
                        if (facingDirection != Direction.WEST) {
                            this.rotate(Command.LEFT);
                            System.out.println(Command.LEFT);
                        } else{
                            this.rotate(Command.RIGHT);
                            System.out.println(Command.RIGHT);
                        }
                        senseDuringFastestPath();
                    }
                    move(Command.FORWARD,1);
                    senseDuringFastestPath();
                    System.out.println(Command.FORWARD);
                }
                else {
                    while(facingDirection != Direction.SOUTH){
                        if (facingDirection != Direction.EAST) {
                            this.rotate(Command.LEFT);
                            System.out.println(Command.LEFT);
                        } else{
                            this.rotate(Command.RIGHT);
                            System.out.println(Command.RIGHT);
                        }
                        senseDuringFastestPath();
                    }
                    move(Command.FORWARD,1);
                    senseDuringFastestPath();
                    System.out.println(Command.FORWARD);
                }
                arenaView.gridArray[this.currPosY][this.currPosX].setExplored(true);
                arenaView.repaint();
                System.out.println(this.currPosX+ ","+this.currPosY);
            }catch (InterruptedException e){
                e.printStackTrace();
            }
        }
    }

    public void simulateFastestPathCommands(Stack<GridBox> path1, Stack<GridBox> path2 ){
        {
            int delay = 0;
            String outputPacket;
            int forwardCount = 0;
            boolean firstMove = true;
            Stack <String> commands = new Stack<>();

            while (!path1.isEmpty()||!path2.empty()){
                GridBox grid;
                if (!path1.empty())
                    grid = path1.pop();
                else
                    grid = path2.pop();


                if (grid.getX() > currPosX){
                    while(facingDirection != Direction.EAST) {
                        System.out.println(forwardCount);
                        if (forwardCount>0 && firstMove){
                            commands.push(returnForwardMethodArduino(forwardCount));
                            forwardCount = 0;
                            firstMove = false;
                        }

                        //forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                        if (facingDirection != Direction.SOUTH) {
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                            commands.push(outputPacket);

                        } else{
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                            commands.push(outputPacket);

                        }
                    }
                    move(Command.FORWARD,1);
                    forwardCount += 1;
                    firstMove = true;
                }
                else if (grid.getX()<currPosX){

                    while(facingDirection != Direction.WEST) {
                        if (forwardCount>0 && firstMove){
                            commands.push(returnForwardMethodArduino(forwardCount));
                            forwardCount =0;
                            firstMove = false;
                        }
                        if (facingDirection != Direction.SOUTH) {
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                            commands.push(outputPacket);

                        } else{
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                            commands.push(outputPacket);

                        }
                    }
                    move(Command.FORWARD,1);
                    forwardCount += 1;
                    firstMove = true;
                    //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
                }
                else if (grid.getY()<currPosY) {
                    while (facingDirection != Direction.NORTH) {
                        if (forwardCount > 0 && firstMove) {
                            commands.push(returnForwardMethodArduino(forwardCount));
                            firstMove = false;
                            forwardCount = 0;
                        }
                        if (facingDirection != Direction.WEST) {
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                            commands.push(outputPacket);

                        } else {
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                            commands.push(outputPacket);

                        }
                        move(Command.FORWARD, 1);
                        forwardCount += 1;
                        firstMove = true;
                        System.out.println("Forward count increased");
                    }
                }
                else {
                    while(facingDirection != Direction.SOUTH) {
                        if (forwardCount>0 && firstMove){
                            commands.push(returnForwardMethodArduino(forwardCount));
                            firstMove = false;
                            forwardCount =0;
                        }
                        //forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);

                        if (facingDirection != Direction.EAST) {
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                            commands.push(outputPacket);

                        } else{
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                            commands.push(outputPacket);
                        }

                    }
                    move(Command.FORWARD,1);
                    forwardCount += 1;
                    firstMove = true;
                }

            }
            if (forwardCount>0){
                commands.push(returnForwardMethodArduino(forwardCount));
            }
            Stack <String> reverseCommands = new Stack();
            for (int i=0; i< commands.size(); i++){
                if (i==0|i==2){
                    String replaceString;
                    String toPush;
                    String toPushFinal;
                    replaceString = commands.pop();
                    toPush = replaceString.replace("M","E");
                    toPushFinal = toPush.replace("F","D");
                    reverseCommands.push(toPushFinal);
                }
                reverseCommands.push(commands.pop());
            }
            for (int i=0;i<reverseCommands.size();i++){
                try{
                    Thread.sleep(delay);
                    System.out.println(reverseCommands.pop());
                    //tcp.sendPacket(reverseCommands.pop());
                    //tcp.receivePacket();
                }catch (InterruptedException e){
                    e.printStackTrace();}
                try{
                    Thread.sleep(delay);
                    //tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                    //tcp.receivePacket();
                }catch (InterruptedException e){
                    e.printStackTrace();}
                arenaView.repaint();
            }

        }

    }

    public void executeFastestPath(Stack<GridBox> path1, Stack<GridBox> path2 )  {
        int delay = 0;
//        tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
//        tcp.receivePacket();
        String outputPacket;
        int forwardCount = 0;
        boolean lastMove = false;

        while (!path1.isEmpty()){
                GridBox grid;
                grid = path1.pop();


                int x = grid.getX();
                int y = grid.getY();
                arenaView.gridArray[y][x].setExplored(true);

                for(int i = y - 1; i <= y + 1; ++i) {
                    for(int j = x - 1; j <= x + 1; ++j) {
                        if (i < 20 && i >= 0 && j < 15 && j >= 0) {
                            arenaView.gridArray[i][j].setExplored(true);
                        }
                    }
                }

                if (grid.getX() > currPosX){
                    while(facingDirection != Direction.EAST) {
                        System.out.println(forwardCount);

                        forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                        if (facingDirection != Direction.SOUTH) {
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                        } else{
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                        }
                        try{
                            Thread.sleep(delay);
                            tcp.sendPacket(outputPacket);
                            //tcp.receivePacket();
                            senseDuringFastestPath();
                            try {
                                Thread.sleep(100);
                            }catch (InterruptedException e){
                                e.printStackTrace();}
                        }catch (InterruptedException e){
                            e.printStackTrace();
                        }
                        try{
                            Thread.sleep(delay);
                            tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                            tcp.receivePacket();
                        }catch (InterruptedException e){
                            e.printStackTrace();}
                    }
                    move(Command.FORWARD,1);
                    forwardCount += 1;
                    //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
                }
                else if (grid.getX()<currPosX){
                    while(facingDirection != Direction.WEST) {
                        forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                        if (facingDirection != Direction.SOUTH) {
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                        } else{
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                        }
                        try{
                            Thread.sleep(delay);
                            tcp.sendPacket(outputPacket);
                            //tcp.receivePacket();
                            senseDuringFastestPath();
                            try {
                                Thread.sleep(100);
                            }catch (InterruptedException e){
                                e.printStackTrace();}
                        }catch (InterruptedException e){
                        e.printStackTrace();}
                        try{
                            Thread.sleep(delay);
                            tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                            tcp.receivePacket();
                        }catch (InterruptedException e){
                                e.printStackTrace();}
                    }
                    move(Command.FORWARD,1);
                    forwardCount += 1;
                    //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
                }
                else if (grid.getY()<currPosY){
                    while(facingDirection != Direction.NORTH) {
                        forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                        if (facingDirection != Direction.WEST) {
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                        } else{
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                        }
                        try{
                            Thread.sleep(delay);
                            tcp.sendPacket(outputPacket);
                            senseDuringFastestPath();
                            //tcp.receivePacket();
                            try {
                                Thread.sleep(100);
                            }catch (InterruptedException e){
                                    e.printStackTrace();}
                        }catch (InterruptedException e){
                            e.printStackTrace();}
                        try{
                            Thread.sleep(delay);
                        tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                        tcp.receivePacket();}catch (InterruptedException e){
                            e.printStackTrace();}
                    }
                    move(Command.FORWARD,1);
                    forwardCount += 1;
                    System.out.println("Forward count increased");
                    //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
                }
                else {
                    while(facingDirection != Direction.SOUTH) {
                        forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                        if (facingDirection != Direction.EAST) {
                            this.rotate(Command.LEFT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                        } else{
                            this.rotate(Command.RIGHT);
                            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                        }
                        try{
                            Thread.sleep(delay);
                            tcp.sendPacket(outputPacket);
                            //tcp.receivePacket();
                            senseDuringFastestPath();
                            try {
                                Thread.sleep(100);
                            }catch (InterruptedException e){
                                e.printStackTrace();}
                        }catch (InterruptedException e){
                            e.printStackTrace();}
                        try{
                            Thread.sleep(delay);
                            tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                            tcp.receivePacket();
                        }catch (InterruptedException e){
                            e.printStackTrace();}

                    }
                    move(Command.FORWARD,1);

                    forwardCount += 1;
                    //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
                }
                arenaView.repaint();
        }
        sendArduinoMultipleForward(forwardCount,delay, false);
    }

    public void executeFastestPath_FP(Stack<GridBox> path1, Stack<GridBox> path2 )  {
        int delay = 0;
        String outputPacket;
        int forwardCount = 0;
        boolean lastMove = false;

        while (!path1.isEmpty()||!path2.empty()){
            GridBox grid;
            if (!path1.empty())
                grid = path1.pop();
            else
                grid = path2.pop();

            if (!path2.empty()){
                if (path2.size() < 2){
                    lastMove = true;
                }
            }

            int x = grid.getX();
            int y = grid.getY();
            arenaView.gridArray[y][x].setExplored(true);

            for(int i = y - 1; i <= y + 1; ++i) {
                for(int j = x - 1; j <= x + 1; ++j) {
                    if (i < 20 && i >= 0 && j < 15 && j >= 0) {
                        arenaView.gridArray[i][j].setExplored(true);
                    }
                }
            }

            if (grid.getX() > currPosX){
                while(facingDirection != Direction.EAST) {
                    System.out.println(forwardCount);

                    forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                    if (facingDirection != Direction.SOUTH) {
                        this.rotate(Command.RIGHT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                    } else{
                        this.rotate(Command.LEFT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                    }
                    try{
                        Thread.sleep(delay);
                        tcp.sendPacket(outputPacket);
                        tcp.receivePacket();
                    }catch (InterruptedException e){
                        e.printStackTrace();
                    }
                    try{
                        Thread.sleep(delay);
                        tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                        tcp.receivePacket();}catch (InterruptedException e){
                        e.printStackTrace();}
                }
                move(Command.FORWARD,1);
                forwardCount += 1;
                //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
            }
            else if (grid.getX()<currPosX){
                while(facingDirection != Direction.WEST) {
                    forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                    if (facingDirection != Direction.SOUTH) {
                        this.rotate(Command.LEFT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                    } else{
                        this.rotate(Command.RIGHT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                    }
                    try{
                        Thread.sleep(delay);
                        tcp.sendPacket(outputPacket);
                        tcp.receivePacket();}catch (InterruptedException e){
                        e.printStackTrace();}
                    try{
                        Thread.sleep(delay);
                        tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                        tcp.receivePacket();}catch (InterruptedException e){
                        e.printStackTrace();}
                }
                move(Command.FORWARD,1);
                forwardCount += 1;
                //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
            }
            else if (grid.getY()<currPosY){
                while(facingDirection != Direction.NORTH) {
                    forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                    if (facingDirection != Direction.WEST) {
                        this.rotate(Command.LEFT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                    } else{
                        this.rotate(Command.RIGHT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                    }
                    try{
                        Thread.sleep(delay);
                        tcp.sendPacket(outputPacket);
                        tcp.receivePacket();;}catch (InterruptedException e){
                        e.printStackTrace();}
                    try{
                        Thread.sleep(delay);
                        tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                        tcp.receivePacket();}catch (InterruptedException e){
                        e.printStackTrace();}
                }
                move(Command.FORWARD,1);
                forwardCount += 1;
                System.out.println("Forward count increased");
                //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
            }
            else {
                while(facingDirection != Direction.SOUTH) {
                    forwardCount = sendArduinoMultipleForward(forwardCount,delay, lastMove);
                    if (facingDirection != Direction.EAST) {
                        this.rotate(Command.LEFT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
                    } else{
                        this.rotate(Command.RIGHT);
                        outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
                    }
                    try{
                        Thread.sleep(delay);
                        tcp.sendPacket(outputPacket);
                        tcp.receivePacket();
                    }catch (InterruptedException e){
                        e.printStackTrace();}
                    try{
                        Thread.sleep(delay);
                        tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                        tcp.receivePacket();}catch (InterruptedException e){
                        e.printStackTrace();}

                }
                move(Command.FORWARD,1);
                forwardCount += 1;
                //outputPacket = SEND_ARDUINO + SEPARATOR + MOVE_FORWARD + "1";
            }
            arenaView.repaint();
        }
        sendArduinoMultipleForward(forwardCount+1,delay, true);
    }

    public void senseDuringFastestPath(){
        if (!SimulatorGUI.runningFastestPath) {
            setSensors();
            sense();
        }
    }


    public String returnForwardMethodArduino (int forwardCount) {
        String outputPacket;
            if (forwardCount <= 10)
                outputPacket = SEND_ARDUINO + SEPARATOR +  MOVE_FORWARD + (forwardCount - 1);
            else
                outputPacket = SEND_ARDUINO + SEPARATOR +  "F" + (forwardCount % 10 - 1);

        return outputPacket;
    }

    public int sendArduinoMultipleForward (int forwardCount, int delay, boolean lastMove){
        if (forwardCount >= 1){
            String outputPacket;
            if (forwardCount <= 10)
                outputPacket = SEND_ARDUINO + SEPARATOR +  MOVE_FORWARD + (forwardCount-1);
            else
                outputPacket = SEND_ARDUINO + SEPARATOR +  "F" + (forwardCount%10-1);
            try{
                Thread.sleep(delay);
                tcp.sendPacket(outputPacket);
                //tcp.receivePacket();
                setSensors();
                sense();
            }catch (InterruptedException e){
                e.printStackTrace();}
            try{
                Thread.sleep(delay);
                tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
                tcp.receivePacket();
            }catch (InterruptedException e){
                e.printStackTrace();}
        }
        else {
            System.out.println("Forward ignored");
        }
        return 0;
    }

    public void sendArduinoRotate(Command command, int delay) {
        String outputPacket;
        if (command == Command.RIGHT) {
            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
        }
        else if (command == Command.LEFT){
            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
        }
        else {
            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_180;
        }
        try{
            Thread.sleep(delay);
            tcp.sendPacket(outputPacket);
            //tcp.receivePacket();
        }catch (InterruptedException e){
            e.printStackTrace();}
//        try{
//            Thread.sleep(delay);
//            tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
//            tcp.receivePacket();
//        }catch (InterruptedException e){
//            e.printStackTrace();}
    }

    public void expSendArduinoRotate(Command command, int delay) {
        String outputPacket = "";
        if (command == Command.RIGHT) {
            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_RIGHT;
        } else if (command == Command.LEFT) {
            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_LEFT;
        } else if (command == Command.TURN180) {
            outputPacket = SEND_ARDUINO + SEPARATOR + ROTATE_180;
        }
        if (outputPacket != ""){
            try {
                Thread.sleep(delay);
                tcp.sendPacket(outputPacket);
                //tcp.receiveBytesArduino();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
//        try{
//            Thread.sleep(delay);
//            tcp.updateAndroid("RUNNING",this.getFacingDirection(),this.getCurrPosY(),this.getCurrPosX());
//            tcp.receivePacket();
//        }catch (InterruptedException e){
//            e.printStackTrace();}
    }

    public int getCurrPosX() {
        return currPosX;
    }

    public int getCurrPosY() {
        return currPosY;
    }

    public Direction getFacingDirection() {
        return facingDirection;
    }

    public void setFacingDirection(Direction facingDirection) {
        this.facingDirection = facingDirection;
    }

    public void setSensors() {
        switch (facingDirection) {
            case NORTH:
                BLSensor.setSensor(this.currPosY , this.currPosX - 1, Direction.WEST);
                LSensor.setSensor(this.currPosY - 1, this.currPosX - 1, Direction.WEST);
                LFSensor.setSensor(this.currPosY - 1, this.currPosX - 1, this.facingDirection);
                MFSensor.setSensor(this.currPosY - 1, this.currPosX , this.facingDirection);
                RFSensor.setSensor(this.currPosY -1, this.currPosX + 1, this.facingDirection);
                RSensor.setSensor(this.currPosY - 1, this.currPosX + 1, Direction.EAST);
                break;
            case EAST:
                BLSensor.setSensor(this.currPosY - 1, this.currPosX , Direction.NORTH);
                LSensor.setSensor(this.currPosY - 1, this.currPosX + 1, Direction.NORTH);
                LFSensor.setSensor(this.currPosY - 1, this.currPosX + 1, this.facingDirection);
                MFSensor.setSensor(this.currPosY, this.currPosX + 1, this.facingDirection);
                RFSensor.setSensor(this.currPosY + 1, this.currPosX + 1, this.facingDirection);
                RSensor.setSensor(this.currPosY + 1, this.currPosX + 1, Direction.SOUTH);
                break;
            case SOUTH:
                BLSensor.setSensor(this.currPosY , this.currPosX + 1, Direction.EAST);
                LSensor.setSensor(this.currPosY + 1, this.currPosX + 1, Direction.EAST);
                LFSensor.setSensor(this.currPosY + 1, this.currPosX + 1, this.facingDirection);
                MFSensor.setSensor(this.currPosY + 1, this.currPosX , this.facingDirection);
                RFSensor.setSensor(this.currPosY + 1, this.currPosX - 1, this.facingDirection);
                RSensor.setSensor(this.currPosY + 1 , this.currPosX - 1, Direction.WEST);
                break;
            case WEST:
                BLSensor.setSensor(this.currPosY + 1, this.currPosX, Direction.SOUTH);
                LSensor.setSensor(this.currPosY + 1, this.currPosX - 1, Direction.SOUTH);
                LFSensor.setSensor(this.currPosY + 1, this.currPosX - 1, this.facingDirection);
                MFSensor.setSensor(this.currPosY, this.currPosX - 1, this.facingDirection);
                RFSensor.setSensor(this.currPosY - 1, this.currPosX - 1, this.facingDirection);
                RSensor.setSensor(this.currPosY - 1, this.currPosX - 1, Direction.NORTH);
                break;
        }
    }

    public boolean sense() {
        boolean[] result = new boolean[6];

        if (simulatedRun) {
            System.out.println("Inside simulation loop robot.sense() method");
            result[0] = BLSensor.senseArena();
            result[1] = LSensor.senseArena();
            result[2] = LFSensor.senseArena();
            result[3] = MFSensor.senseArena();
            result[4] = RFSensor.senseArena();
            if (!SimulatorGUI.imageRegRun)
                result[5] = RSensor.senseArena();
            //System.out.print("Inside 1st If Loop");
        } else {
            System.out.println("Inside realRun loop robot.sense() method");
            boolean getData = true;
            while (getData) {
                System.out.println("Inside while loop, waiting to receive sensor data");
                String msg = tcp.receivePacket();
                if (msg.contains("STOP")){
                    this.stopSignal = true;
                }
                if (SimulatorGUI.runningFastestPath) {
                    return msg.contains("b'K");
                }

                System.out.println(msg);
                String[] msgArr = msg.split(",");
                System.out.println(msgArr[0]);
                if (msgArr[0].contains(IR_SENSORS)) {
                    for (int i = 1; i <= 5; i++) {
                        System.out.println(msgArr[i]);
                    }
                    BLSensor.senseRealArena(parseFloat(msgArr[1]));
                    LSensor.senseRealArena(parseFloat(msgArr[2]));
                    LFSensor.senseRealArena(parseFloat(msgArr[3]));
                    MFSensor.senseRealArena(parseFloat(msgArr[4]));
                    RFSensor.senseRealArena(parseFloat(msgArr[5]));
                    if (!SimulatorGUI.imageRegRun)
                        RSensor.senseRealArena(parseFloat(msgArr[6]));
                    System.out.println("Finished Sensed");
                    return true;
//                }
                } else if (msgArr[0].contains(ROBOT_SELF_CLEAR_GRIDS)){
                    int gridsCleared = Integer.parseInt(msgArr[1]);
                    setRobotSelfClearedGrids(gridsCleared, msgArr);
                    int offset = (SimulatorGUI.imageRegRun) ? 1 : gridsCleared; // no need process RSensor if ImgRec run
                    setSensors();
                    BLSensor.senseRealArena(parseFloat(msgArr[1+offset+1])); // Grids Cleared account for IR too
                    LSensor.senseRealArena(parseFloat(msgArr[1+offset+2]));
                    LFSensor.senseRealArena(parseFloat(msgArr[1+offset+3]));
                    MFSensor.senseRealArena(parseFloat(msgArr[1+offset+4]));
                    RFSensor.senseRealArena(parseFloat(msgArr[1+offset+5]));
                    if (!SimulatorGUI.imageRegRun)
                        RSensor.senseRealArena(parseFloat(msgArr[1+offset+6]));
                    return true;
                }
//                else if(msg == "STOP"|| msgArr[0] == "STOP"){
//                    this.stopSignal = true;
//                }
            }
        }
        return false;
    }

    public void setRobotSelfClearedGrids(int gridsCleared, String[] msgArr){

        for (int i = 1; i <= gridsCleared; i++){
            move(Command.FORWARD,1);
            switch (this.facingDirection) {
                case NORTH:
                    //currPosY--;
                    if (currPosY < 1) {
                        currPosY = 1;
                    }
                    arenaView.setExploredGrids(currPosY-1,currPosX);
                    arenaView.setExploredGrids(currPosY-1,currPosX+1);
                    arenaView.setExploredGrids(currPosY-1,currPosX-1);
                    setSensors();
                    if (i<gridsCleared && !SimulatorGUI.imageRegRun)
                        RSensor.senseRealArena(parseFloat(msgArr[1+i]));
                    break;
                case SOUTH:
                    //currPosY ++;
                    if (currPosY > 18) {
                        currPosY = 18;
                    }
                    arenaView.setExploredGrids(currPosY+1,currPosX);
                    arenaView.setExploredGrids(currPosY+1,currPosX+1);
                    arenaView.setExploredGrids(currPosY+1,currPosX-1);
                    setSensors();
                    if (i<gridsCleared && !SimulatorGUI.imageRegRun)
                        RSensor.senseRealArena(parseFloat(msgArr[1+i]));
                    break;
                case EAST:
                    //currPosX++;
                    if (currPosX > 13) {
                        currPosX = 13;
                    }
                    arenaView.setExploredGrids(currPosY,currPosX+1);
                    arenaView.setExploredGrids(currPosY-1,currPosX+1);
                    arenaView.setExploredGrids(currPosY+1,currPosX+1);
                    setSensors();
                    if (i<gridsCleared && !SimulatorGUI.imageRegRun)
                        RSensor.senseRealArena(parseFloat(msgArr[1+i]));
                    break;
                case WEST:
                    //currPosX--;
                    if (currPosX < 1) {
                        currPosX = 1;
                    }
                    arenaView.setExploredGrids(currPosY,currPosX-1);
                    arenaView.setExploredGrids(currPosY-1,currPosX-1);
                    arenaView.setExploredGrids(currPosY+1,currPosX-1);
                    setSensors();
                    if (i<gridsCleared && !SimulatorGUI.imageRegRun)
                        RSensor.senseRealArena(parseFloat(msgArr[1+i]));
                    break;
            }

            System.out.println("Current Pos Y:" + currPosY + " Current Pos X:" + currPosX);
        }
    }

    public boolean isSimulatedRun() {
        return simulatedRun;
    }

    public void setSimulatedRun(boolean simulatedRun) {
        this.simulatedRun = simulatedRun;
    }

    public void setCurrPosX(int currPosX) {
        this.currPosX = currPosX;
    }

    public void setCurrPosY(int currPosY) {
        this.currPosY = currPosY;
    }
}

