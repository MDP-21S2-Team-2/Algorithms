package view;

import arena.ArenaConstants;
import arena.ArenaView;
import arena.GridBox;
import communications.TCPConstants;
import navigation.Exploration;
import navigation.FastestPath;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import static arena.ArenaLoader.loadMapFromDisk;
import static arena.ArenaLoader.createArenaHex;
import static arena.ArenaLoader.generateMapDescriptor;

import navigation.ImageRecognition;
import view.EntryPoint;
import java.text.NumberFormat;

import static arena.ArenaView.gridArray;
import static communications.TCPManager.tcp;


public class SimulatorGUI {

    //private static GUI stimulatorInterface = new GUI();
    public SimulatorGUI(){


    }
    //Variable Declarations
    public static ArenaView arenaView;
    public static Robot robot;

    public boolean simulation = true;
    public JFrame frame;
    public int frameWidth;
    public int frameHeight;
    public JPanel arenaPanel;
    public JPanel fieldPanel;
    public JPanel controllerPanel;
    public JButton explorationButton;
    public JButton fastestPath;
    public JButton loadMap;
    public JButton randomObstacles;
    public JButton imageRecButton;
    public JLabel addingwaypoint;
    public JLabel wayX;
    public JButton createArena;
    public JFormattedTextField waypointX;
    public JLabel wayY;
    public JFormattedTextField waypointY;
    public JCheckBox realRunCheck;
    public JLabel timer;
    public JSpinner spinnerX;
    public JSpinner spinnerY;
    public JButton timeLimited;
    public JButton coverageLimited;
    public JButton reset;
    public int timeLimit = 340;
//    public int timeLimit = 30;
    public int coverageLimit = 300;
    public static int _waypointX = 13;
    public static int _waypointY = 1;
    public String p1_;
    public String p2_ = "0000000000000000000000000000000000000000000000000000000000000000000000000000";
    //public String p2_ = "01000000000000F00000000000400007E0000000000000001F80000780000000000004000800"; //Arena 1
    //public String p2_ = "000000000000010042038400000000000000030C000000000000021F84000800000000000400"; //Arena 2
    //public String p2_ = "0000000000000400080010C000F0000000000000007E00FC0000000180000100020004000800"; //Arena 3
    //public String p2_ = "00400080010000000000003F000000000000400100040F000000000380000000080010002000"; // Arena 4
    // public String p2_ = "0700000000000001C00002000400080010202040408001000200040000380000000020004200"; // Arena 5
    //public String p2_ = "01C0000000000000001F0000000000000007E000000000001F83C00400000000000000000000"; //Shared Arena
    public String[] mdf = new String[2];
    public static boolean imageRegRun = false;
    public static boolean runningFastestPath = false;




    //private static JFormattedTextField robotTimeField;


//    //Adding of various buttons with mouse listeners into Button Panel
//    private static void createButtons(JPanel btnPanel){
//        JButton eBtn = new JButton("Exploration");
//        JButton fpBtn = new JButton("Fastest Path");
//
//        //Exploration SwingWorker, a less computational alternative
//
//        btnPanel.add(eBtn);
//        btnPanel.add(fpBtn);
//    }

    public void setSimulation (Boolean simulate){
        simulate = simulation;
    }

    public boolean getSimulation (Boolean simulate){
        return simulation;
    }

    public void initializations(){
        //Variables that will be used to display the GUI
        arenaPanel = new JPanel(new CardLayout());
        arenaView = new ArenaView();
        arenaView.resetObstacles();
        arenaView.setAllUnexplored();
        arenaView.initializeGrids();
        arenaView.initializeNeighbouringGrids();
        //arenaView.generateRandomObstacles(ArenaConstants.NUM_OBSTACLES);
        arenaView.calculateSpaceClearance();
        arenaPanel.add(arenaView);
        frame = new JFrame("MDP Group 2");
        frame.setBackground(Color.WHITE);
        //frame.setBackground(Color.BLACK);
        frame.setLayout(new BorderLayout(10,10));
        frameWidth = ArenaConstants.FRAME_WIDTH;
        frameHeight = ArenaConstants.FRAME_HEIGHT +100;


        try {
            robot = new Robot();
        } catch (AWTException e) {
            e.printStackTrace();
        }


        //JPanel btnPanel = new JPanel();
        if (simulation){

        }
    }

    public void displayGUI(){
        // And JPanel needs to be added to the JFrame itself!
        //arenaPanel = new JPanel();
        //arenaPanel.setLayout(new FlowLayout());
        //arenaPanel.add(arenaView);
        frame.getContentPane().add(arenaPanel,BorderLayout.CENTER);
        //frame.add(arenaPanel, BorderLayout.CENTER);
        createButtonsPanel();
        frame.setSize(frameWidth,frameHeight);
        frame.setVisible(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
    }

    public void simulateRobotRuns(){

    }

    public void executeRobotRuns(){

    }

    public void createButtonsPanel (){

        //creating the buttons
        fastestPath = new JButton("Fastest Path");
        realRunCheck = new JCheckBox("Simulation");
        realRunCheck.setHorizontalTextPosition(SwingConstants.LEFT);
        loadMap = new JButton(("Load Map"));
        createArena = new JButton("Create Arena");
        //addingwaypoint = new JLabel("Add WayPoint (x,y):");
        //timeLimited = new JButton("Time Limited");
        //coverageLimited = new JButton("Coverage Limited");
        explorationButton = new JButton("Exploration");
        reset = new JButton("Reset");
        imageRecButton = new JButton("Image Recognition");

        //waypoint
//        wayX = new JLabel("X",JLabel.LEFT);
//        wayY = new JLabel("Y",JLabel.LEFT);
//        JSpinner spinnerX = new JSpinner(new SpinnerNumberModel(1, 0, 14, 1));
//        JSpinner spinnerY = new JSpinner(new SpinnerNumberModel(17, 0, 19, 1));

        //stopwatch
//        JLabel time = new JLabel("Stopwatch:");
//        timer = new JLabel("00:00");

        //Loading Map from disk
        loadMap.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadMapFromDisk(arenaView, "Exploration4");
                CardLayout cl = ((CardLayout) arenaPanel.getLayout());
                cl.show(arenaPanel, "Map");
                arenaView.calculateSpaceClearance();
                arenaView.repaint();
//                JDialog loadMapDialog = new JDialog(frame, "Load Map", true);
//                loadMapDialog.setSize(400, 60);
//                loadMapDialog.setLayout(new FlowLayout());
//
//                final JTextField loadTF = new JTextField(15);
//                JButton loadMapButton = new JButton("Load");
//
//                loadMapButton.addMouseListener(new MouseAdapter() {
//                    public void mousePressed(MouseEvent e) {
//                        loadMapDialog.setVisible(false);
//                        loadMapFromDisk(arenaView, loadTF.getText());
//                        CardLayout cl = ((CardLayout) arenaPanel.getLayout());
//                        cl.show(arenaPanel, "Map");
//                        arenaView.calculateSpaceClearance();
//                        arenaView.repaint();
//                    }
//                });
//
//                loadMapDialog.add(new JLabel("File Name: "));
//                loadMapDialog.add(loadTF);
//                loadMapDialog.add(loadMapButton);
//                loadMapDialog.setVisible(true);
            }
        });

        //Creating Arena
        createArena.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                p1_ = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF";
//                //p2_ = "01000000000000F00000000000400007E0000000000000001F80000780000000000004000800"; //Arena 1
                p2_ = "000000000000010042038400000000000000030C000000000000021F84000800000000000400"; //Arena 2
//                //p2_ = "0000000000000400080010C000F0000000000000007E00FC0000000180000100020004000800"; //Arena 3
//                p2_ = "00400080010000000000003F000000000000400100040F000000000380000000080010002000"; // Arena 4
//                //p2_ = "0700000000000001C00002000400080010202040408001000200040000380000000020004200"; // Arena 5
//                //p2_ = "01C0000000000000001F0000000000000007E000000000001F83C00400000000000000000000"; //Shared Arena
                createArenaHex(p1_, p2_, arenaView);
                arenaView.calculateSpaceClearance();
                arenaView.repaint();
                CardLayout cl = ((CardLayout) arenaPanel.getLayout());
                cl.show(arenaPanel, "Map");
//                JDialog createArenaDialog = new JDialog(frame, "Create Arena", true);
//                createArenaDialog.setSize(800, 120);
//                createArenaDialog.setLayout(new FlowLayout());
//
//                final JTextField p1 = new JTextField(25);
//                final JTextField p2 = new JTextField(25);
//                JButton drawArena = new JButton("Create");
//
//                drawArena.addMouseListener(new MouseAdapter() {
//                    public void mousePressed(MouseEvent e) {
//                        createArenaDialog.setVisible(false);
//                        p1_ = p1.getText();
//                        p2_ = p2.getText();
//                        createArenaHex(p1_, p2_, arenaView);
//                        arenaView.calculateSpaceClearance();
//                        arenaView.repaint();
//                        CardLayout cl = ((CardLayout) arenaPanel.getLayout());
//                        cl.show(arenaPanel, "Map");
//
//                    }
//                });
//
//                createArenaDialog.add(new JLabel("P1"));
//                createArenaDialog.add(p1);
//                createArenaDialog.add(new JLabel("P2"));
//                createArenaDialog.add(p2);
//                createArenaDialog.add(drawArena);
//                createArenaDialog.setVisible(true);
            }
        });


        // FastestPath Class for Multithreading
        class FastestPath extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                GridBox startGrid = gridArray[ArenaConstants.START_ROW][ArenaConstants.START_COL];
                startGrid.printGridInfo();
                //navigation.FastestPath fp3 = new navigation.FastestPath(startGrid,waypoint) + new navigation.FastestPath(waypoint,endGrid);;
                arenaView.repaint();
                boolean runnningwhile = true;
                runningFastestPath = true;
                if (!simulation){
                    tcp.establishConnection();
                    do{
                        String packet2 = tcp.receivePacket();
                        switch(packet2) {
                            case (TCPConstants.UPDATEMAP_ANDROID):
                                tcp.sendMDFAndroid(p2_);
                                break;//not sure if this has to wait for something
                            case (TCPConstants.INITIAL_CALIBRATE):
                            case (TCPConstants.ENABLE_ALIGNMENT):
                            case (TCPConstants.DISABLE_ALIGNMENT):
                            case (TCPConstants.ENABLE_E_BRAKES):
                            case (TCPConstants.DISABLE_E_BRAKES):
                                tcp.sendPacket(TCPConstants.SEND_ARDUINO + TCPConstants.SEPARATOR + packet2);
                                while (!tcp.receivePacket().equals("K")) {
                                    break;
                                }
                                System.out.println("Calibration done!");
                                break;
                            case (TCPConstants.START_FP):
                                runnningwhile = false;
                                break;
                            case ("START,1:1"):
                                System.out.println("Start Position Received");
                                break;
                            default:
                                if (packet2.contains(TCPConstants.SET_WP)) {
                                    tcp.setWayPoint(packet2);
                                    _waypointY = 19 - _waypointY;
                                    System.out.println("WayPoint set! " + _waypointY + " " + _waypointX);
                                }
                                else{
                                    System.out.println("Unknown Data Received");
                                }

                        }
                    }
                    while (runnningwhile);
                    GridBox waypoint = gridArray[_waypointY][_waypointX];
                    GridBox endGrid = gridArray[ArenaConstants.GOAL_ROW][ArenaConstants.GOAL_COL];
                    navigation.FastestPath fp1 = new navigation.FastestPath(startGrid, waypoint);
                    navigation.FastestPath fp2 = new navigation.FastestPath(waypoint, endGrid);
                    arenaView.robot.executeFastestPath_FP(fp1.findFastestPath(),fp2.findFastestPath());
                }
                else {
                    GridBox waypoint = gridArray[_waypointY][_waypointX];
                    GridBox endGrid = gridArray[ArenaConstants.GOAL_ROW][ArenaConstants.GOAL_COL];
                    endGrid.printGridInfo();
                    //navigation.FastestPath fp = new navigation.FastestPath(startGrid,endGrid);
                    navigation.FastestPath fp1 = new navigation.FastestPath(startGrid,waypoint);
                    navigation.FastestPath fp2 = new navigation.FastestPath(waypoint,endGrid);
                    arenaView.robot.simulateFastestPath(fp1.findFastestPath(),null);
                    //arenaView.robot.simulateFastestPath(fp1.findFastestPath(),fp2.findFastestPath());
                    arenaView.robot.simulateFastestPath(fp2.findFastestPath(),null);
                }

                return 222;
            }
        }

        // Fastest Path Button
        fastestPath.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                //Getting simulation value
                simulation = realRunCheck.isSelected();
                arenaView.robot.setSimulatedRun(simulation);
                //Setting Way Point
                //_waypointX = (int) spinnerX.getValue();
                //_waypointY = (int) spinnerY.getValue();
                //CardLayout cl = ((CardLayout) arenaPanel.getLayout());
                //cl.show(arenaPanel, "Fastest Path");
                //arenaView.calculateSpaceClearance();
                //arenaView.repaint();
                new FastestPath().execute();
            }
        });

        class ImageRecognitionExplore extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                ImageRecognition imgRecog = new ImageRecognition(coverageLimit, timeLimit, !simulation);
                imageRegRun = true;
                boolean runnningwhile = true;
                if (simulation) {
                    System.out.println("Exploration Started Part 1");
                    imgRecog.runExploration();
                } else {
                    tcp.establishConnection();
                    do {
                        boolean initialSense = false;
                        String packet1 = tcp.receivePacket();
                        switch (packet1) {
                            case (TCPConstants.INITIAL_CALIBRATE):
                                tcp.sendPacket(TCPConstants.SEND_ARDUINO + TCPConstants.SEPARATOR + packet1);
                                while (!initialSense) {
                                    System.out.println("In while loop simulatorGUI");
                                    arenaView.robot.setSensors();
                                    initialSense = arenaView.robot.sense();
                                }
                                arenaView.repaint();
                                System.out.println("Calibration done!");
                                break;
                            case (TCPConstants.START_EXP):
                                runnningwhile = false;
                                break;
                            case (TCPConstants.UPDATEMAP_ANDROID):
                                tcp.sendMDFAndroid(p2_);
                                tcp.sendPacket(TCPConstants.SEND_ANDROID + TCPConstants.SEPARATOR +"IMAGE;");
                                break;
                        }
                    }
                    while (runnningwhile);
                    imgRecog.runExploration();
                }
                System.out.println(generateMapDescriptor(arenaView));
                return 222;
            }
        }

        imageRecButton.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                //Getting simulation value
                simulation = realRunCheck.isSelected();
                arenaView.robot.setSimulatedRun(simulation);
                //Setting Way Point
                //_waypointX = (int) spinnerX.getValue();
                //_waypointY = (int) spinnerY.getValue();
                //CardLayout cl = ((CardLayout) arenaPanel.getLayout());
                //cl.show(arenaPanel, "Fastest Path");
                //arenaView.calculateSpaceClearance();
                //arenaView.repaint();
                new ImageRecognitionExplore().execute();
            }
        });


        //Exploration Multi-Threading
        class ExplorationSimulator extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                Exploration exp = new Exploration(coverageLimit, timeLimit, !simulation);
                boolean runnningwhile = true;
                if (simulation) {
                    System.out.println("Exploration Started Part 1");
                    exp.runExploration();
                    System.out.println(generateMapDescriptor(arenaView));

                }else {
                    tcp.establishConnection();
                    int loopStartTimes = 0;
//                    while (true){
//                        String packet1 = tcp.receivePacket();
//                        if (packet1.equals("C")){
//                            tcp.sendPacket(TCPConstants.SEND_ARDUINO + TCPConstants.SEPARATOR+packet1); //not sure if this has to wait for something
//                            break;
//                        }
//                    }
//                    while (true) {
//                        String packet1 = tcp.receivePacket();
//                        if (packet1.equals("UPDATE")) {
//                            tcp.sendMDFAndroid(p2_); //not sure if this has to wait for something
//                            break;
//                        }
//                    }
                    do {
                        boolean initialSense = false;
                        String packet1 = tcp.receivePacket();
                        switch (packet1) {
                            case (TCPConstants.INITIAL_CALIBRATE):
                                tcp.sendPacket(TCPConstants.SEND_ARDUINO + TCPConstants.SEPARATOR + packet1);
                                while (!initialSense) {
                                    System.out.println("In while loop simulatorGUI");
                                    arenaView.robot.setSensors();
                                    initialSense = arenaView.robot.sense();
                                }
                                arenaView.repaint();
                                System.out.println("Calibration done!");
                                break;
                            case (TCPConstants.START_EXP):
                                runnningwhile = false;
                                loopStartTimes++;
                                exp.runExploration();
                                System.out.println("EXPLORATION DONE and COMPLETED!!!!");
                                mdf = generateMapDescriptor(arenaView);
                                tcp.sendPacket("AN-P1," + mdf[0] + ";P2," + mdf[1]);
                                System.out.println(mdf);
                                if (loopStartTimes > 2){
                                    runnningwhile = false;
                                }
                                break;
                            case ("RESET"):
                                initializations();
                                displayGUI();
                                System.out.println("Reset!");
                                break;
                            case (TCPConstants.UPDATEMAP_ANDROID):
                                tcp.sendMDFAndroid(p2_);
                                tcp.sendPacket(TCPConstants.SEND_ANDROID + TCPConstants.SEPARATOR +"IMAGE;");
                                //tcp.receivePacket();
                                break;//not sure if this has to wait for something
                        }
                    }
                    while (runnningwhile);
                }
                //System.out.println(generateMapDescriptor(arenaView));
//                mdf = generateMapDescriptor(arenaView);
//                tcp.sendPacket("AN-P1: " + mdf[0] + ";P2: " + mdf[1]);
//                System.out.println(mdf);
                return 222;
            }
        }


        //Time Exploration Button
//        timeLimited.addMouseListener(new MouseAdapter() {
//            public void mousePressed(MouseEvent e) {
//                //Getting simulation value
//                simulation = realRunCheck.isSelected();
//                JDialog timeExploDialog = new JDialog(frame, "Time-Limited Exploration", true);
//                timeExploDialog.setSize(400, 60);
//                timeExploDialog.setLayout(new FlowLayout());
//                final JTextField timeTF = new JTextField(5);
//                JButton timeSaveButton = new JButton("Run");
//
//                timeSaveButton.addMouseListener(new MouseAdapter() {
//                    public void mousePressed(MouseEvent e) {
//                        timeExploDialog.setVisible(false);
//                        String time = timeTF.getText();
//                        String[] timeArr = time.split(":");
//                        timeLimit = (Integer.parseInt(timeArr[0]) * 60) + Integer.parseInt(timeArr[1]);
//                        new ExplorationSimulator().execute();
//                    }
//                });
//                timeExploDialog.add(new JLabel("Time Limit (in MM:SS): "));
//                timeExploDialog.add(timeTF);
//                timeExploDialog.add(timeSaveButton);
//                timeExploDialog.setVisible(true);
//            }
//        });

        //Coverage Exploration Button
//        coverageLimited.addMouseListener(new MouseAdapter() {
//            public void mousePressed(MouseEvent e) {
//                //Getting simulation value
//                simulation = realRunCheck.isSelected();
//
//                JDialog coverageExploDialog = new JDialog(frame, "Coverage-Limited Exploration", true);
//                coverageExploDialog.setSize(400, 60);
//                coverageExploDialog.setLayout(new FlowLayout());
//                final JTextField coverageTF = new JTextField(5);
//                JButton coverageSaveButton = new JButton("Run");
//
//                coverageSaveButton.addMouseListener(new MouseAdapter() {
//                    public void mousePressed(MouseEvent e) {
//                        coverageExploDialog.setVisible(false);
//                        int coveragePercent = (Integer.parseInt(coverageTF.getText()));
//                        coverageLimit = (int) ((coveragePercent) * 300 / 100.0);
//                        if (coveragePercent>100 || coveragePercent<0){
//                            JDialog errorDialog = new JDialog(frame,"Error");
//                            errorDialog.setSize(400, 60);
//                            JLabel errorMsg = new JLabel("Coverage Limit is not within range (0-100)");
//                            errorDialog.add(errorMsg);
//                            errorDialog.setVisible(true);
//                        }
//                        else{
//                            new ExplorationSimulator().execute();
//
//                        }
//                        //CardLayout cl = ((CardLayout) arenaPanel.getLayout());
//                        //cl.show(arenaPanel, "EXPLORATION");
//                        //new ExplorationSimulator().execute();
//                    }
//                });
//
//                coverageExploDialog.add(new JLabel("Coverage Limit (% of maze): "));
//                coverageExploDialog.add(coverageTF);
//                coverageExploDialog.add(coverageSaveButton);
//                coverageExploDialog.setVisible(true);
//            }
//        });


        //Exploration Button
        explorationButton.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                //Getting simulation value
                simulation = realRunCheck.isSelected();
                arenaView.robot.setSimulatedRun(simulation);
                System.out.println("Exploration Started");
                //runExploration();
                //CardLayout cl = ((CardLayout) arenaPanel.getLayout());
                //cl.show(arenaPanel, "EXPLORATION");
                new ExplorationSimulator().execute();
            }
        });


        //Reset Button
        reset.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                initializations();
                displayGUI();
                System.out.println("Reset!");

            }
        });

        //Adding the buttons
        JPanel controllerPanel = new JPanel();
        controllerPanel.setLayout(new BorderLayout());
        JPanel buttonPanel = new JPanel();
        buttonPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 2, 15));
        buttonPanel.setBackground(Color.decode("#AFEEEE"));
        buttonPanel.add(loadMap);
        buttonPanel.add(createArena);
        buttonPanel.add(fastestPath);
        buttonPanel.add(imageRecButton);
        //buttonPanel.add(timeLimited);
        //buttonPanel.add(coverageLimited);
        buttonPanel.add(explorationButton);


        JPanel fieldPanel = new JPanel();
        fieldPanel.setLayout(new FlowLayout(FlowLayout.CENTER, 5, 10));
        fieldPanel.setBackground(Color.decode("#AFEEEE"));
//        fieldPanel.add(addingwaypoint);
//        fieldPanel.add(wayX);
//
//        fieldPanel.add(spinnerX);
//        fieldPanel.add(wayY);
//        fieldPanel.add(spinnerY);
//        fieldPanel.add(time);
//        fieldPanel.add(timer);
        fieldPanel.add(realRunCheck);
        fieldPanel.add(reset);
        controllerPanel.add(fieldPanel, BorderLayout.PAGE_START);
        controllerPanel.add(buttonPanel, BorderLayout.PAGE_END);
        frame.add(controllerPanel, BorderLayout.PAGE_END);


    }


    public void addExplorationButtonListener(ActionListener actionListener) {
        explorationButton.addActionListener(actionListener);
    }

    public void addGenerateArenaListener(ActionListener actionListener) {
        randomObstacles.addActionListener(actionListener);
    }

    public void addFastestPathButtonListener(ActionListener actionListener) {
        fastestPath.addActionListener(actionListener);
    }

    public void addRealRunCheckBoxListener(ActionListener actionListener) {
        realRunCheck.addActionListener(actionListener);
    }

    public void addloadMapListener(ActionListener actionListener) {
        loadMap.addActionListener(actionListener);
    }
//
//    public void timeLimitedListener(ActionListener actionListener) {
//        timeLimited.addActionListener(actionListener);
//    }
//
//    public void coverageLimitedListener(ActionListener actionListener) {
//        coverageLimited.addActionListener(actionListener);
//    }

    public void disableButtons() {
        explorationButton.setEnabled(false);
        randomObstacles.setEnabled(false);
        fastestPath.setEnabled(false);
        realRunCheck.setEnabled(false);
        loadMap.setEnabled(false);
        timeLimited.setEnabled(false);
        coverageLimited.setEnabled(false);
    }

    public void enableButtons() {
        explorationButton.setEnabled(true);
        randomObstacles.setEnabled(false);
        fastestPath.setEnabled(true);
        realRunCheck.setEnabled(true);
        loadMap.setEnabled(true);
        timeLimited.setEnabled(true);
        coverageLimited.setEnabled(true);
    }
//
//    public void disableExplorationButton() {
//        explorationButton.setEnabled(false);
//    }
//
//    public void enableExplorationButton() {
//        explorationButton.setEnabled(true);
//    }
//
//    public void disableFastestPathButton() {
//        fastestPath.setEnabled(false);
//    }
//
//    public void enableFastestPathButton() {
//        fastestPath.setEnabled(true);
//    }
//
//    public void disableRealRunButton() {
//        realRunCheck.setEnabled(false);
//    }
//
//    public void enableRealRunButton() {
//        realRunCheck.setEnabled(true);
//    }
//
//    public void disableSetObstacles() {
//        loadMap.setEnabled(false);
//    }
//
//    public void enableSetObstacles() {
//        loadMap.setEnabled(true);
//    }
//
//    public void enableTimeLimited() {
//        timeLimited.setEnabled(true);
//    }
//
//    public void enableCoverageLimited() {
//        coverageLimited.setEnabled(true);
//    }
//
//    public boolean getIsRealRun() {
//        return realRunCheck.isSelected();
//    }
//
//    public int getWayPointX() {
//        //return waypointX.getText().equals("") ? 0 : Integer.parseInt(waypointX.getText());
//        return (int) spinnerX.getValue();
//    }
//
//    public int getWayPointY() {
//        //return waypointY.getText().equals("") ? 0 : Integer.parseInt(waypointY.getText());
//        return (int) spinnerY.getValue();
//    }

}
















