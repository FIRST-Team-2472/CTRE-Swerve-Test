package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveDriveToPointCmd;
import frc.robot.extras.AutoPose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.HashMap;
import java.util.Map;

public class CommandSequences {
    public static AutoPose2d[] cageNodes = new AutoPose2d[6];
    public static Map<Character, AutoPose2d> reefNodesMap = new HashMap<>(); // replaced reefNodes

    public static AutoPose2d leftHumanPlayer = simplePose(1.127, 6.982, 306);
    public static AutoPose2d rightHumanPlayer = simplePose(1.127, 1.035, 54);
    public static AutoPose2d middle = simplePose(7.115, 4, 180);
    public static AutoPose2d rightReefPassage = simplePose(5.8, 1.7, 75);
    public static AutoPose2d leftReefPassage = simplePose(5.8, 7, 285);
    public static AutoPose2d processor = simplePose(6.350, 0.550,270);

    public static void initPoses() {
        // x is centered on starting line
        cageNodes[0] = simplePose(7.114, 7.279, 180); //Cage on far left from driver POV
        cageNodes[1] = simplePose(7.114, 6.165, 180); //Cage Position 2
        cageNodes[2] = simplePose(7.114, 5.077, 180); //Cage Position 3
        cageNodes[3] = simplePose(7.114, 2.929, 180); //Cage Position 4
        cageNodes[4] = simplePose(7.114, 1.898, 180); //Cage Position 5
        cageNodes[5] = simplePose(7.114, 0.794, 180); //Cage on far right from driver POV

        reefNodesMap.put('A', simplePose(3.168, 4.190, 0));
        reefNodesMap.put('B', simplePose(3.168, 3.860, 0));
        reefNodesMap.put('C', simplePose(3.682, 2.958, 60));
        reefNodesMap.put('D', simplePose(3.97, 2.798, 60));
        reefNodesMap.put('E', simplePose(5.003, 2.796, 120));
        reefNodesMap.put('F', simplePose(5.294, 2.962, 120));
        reefNodesMap.put('G', simplePose(5.81, 3.86, 180));
        reefNodesMap.put('H', simplePose(5.79, 4.2, 180));
        reefNodesMap.put('I', simplePose(5.292, 5.094, 240)); 
        reefNodesMap.put('J', simplePose(5.006, 5.253, 240));
        reefNodesMap.put('K', simplePose(3.972, 5.249, 300));
        reefNodesMap.put('L', simplePose(3.686, 5.085, 300));
    }

    //Starting Position to reef

    public static Command CageOneToH(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[0].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('H').toPose2d());
    }

    public static Command setOdodmeteryToTest(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(simplePose(2.18, 4, 0));
        return null;
    }

    public static Command CageOneToI(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[0].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('I').toPose2d());
    }

    public static Command CageTwoToH(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[1].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('H').toPose2d());
    }

    public static Command CageTwoToI(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[1].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('I').toPose2d());
    }

    public static Command CageThreeToH(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[2].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('H').toPose2d());
    }

    public static Command CageThreeToG(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[2].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('G').toPose2d());
    }

    public static Command MiddleToH(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(middle.toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('H').toPose2d());
    }

    public static Command MiddleToG(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(middle.toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('G').toPose2d());
    }

    public static Command CageFourToH(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[3].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('H').toPose2d());
    }

    public static Command CageFourToG(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[3].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('G').toPose2d());
    }

    public static Command CageFourToF(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[3].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('F').toPose2d());
    }

    public static Command CageFiveToG(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[4].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('G').toPose2d());
    }

    public static Command CageFiveToF(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[4].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('F').toPose2d());
    }

    public static Command CageSixToG(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[5].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('G').toPose2d());
    }

    public static Command CageSixToF(CommandSwerveDrivetrain drivetrain){
        drivetrain.resetPose(cageNodes[5].toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('F').toPose2d());
    }

    //Reef to Source

//    public static Command HToLeftPlayer(CommandSwerveDrivetrain drivetrain){
//        drivetrain.resetPose(reefNodesMap.get('H').toPose2d());
//        return new SwerveFollowTransitionCmd(drivetrain, leftReefPassage, leftHumanPlayer, 1);
//    }
//
//    public static Command GToRightPlayer(CommandSwerveDrivetrain drivetrain){
//        drivetrain.resetPose(reefNodesMap.get('G').toPose2d().toPose2d());
//        return new SwerveFollowTransitionCmd(drivetrain, rightReefPassage, rightHumanPlayer, 1);
//    }
//
//    public static Command FToRightPlayer(CommandSwerveDrivetrain drivetrain){
//        //drivetrain.resetPose(reefNodesMap.get('F').toPose2d().toPose2d());
//        return new SwerveFollowTransitionCmd(drivetrain, rightReefPassage, rightHumanPlayer, 1);
//    }
//
//    public static Command IToLeftPlayer(CommandSwerveDrivetrain drivetrain){
//        //drivetrain.resetPose(reefNodesMap.get('I').toPose2d().toPose2d());
//        return new SwerveFollowTransitionCmd(drivetrain, leftReefPassage, leftHumanPlayer, 1);
//        //return new SwerveDriveToPointCmd(drivetrain, leftHumanPlayer);
//    }

    //Source to Reef

    public static Command RightPlayerToD(CommandSwerveDrivetrain drivetrain){
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('D'));
    }

//    public static Command LeftPlayerToK(CommandSwerveDrivetrain drivetrain){
//        return new SwerveFollowTransitionCmd(drivetrain, leftReefPassage, reefNodesMap.get('K').toPose2d(), 1);
//    }

    public static Command RightPlayerToC(CommandSwerveDrivetrain drivetrain){
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('C').toPose2d());
    }

    public static Command LeftPlayerToL(CommandSwerveDrivetrain drivetrain){
        return new SwerveDriveToPointCmd(drivetrain, reefNodesMap.get('L').toPose2d());
    }

    public static Command driveForwardTest(CommandSwerveDrivetrain drivetrain) {
        drivetrain.resetPose(simplePose(1, 0, 0).toPose2d());
        return new SwerveDriveToPointCmd(drivetrain, simplePose(3, 0, 0));
    }

    public static AutoPose2d simplePose(double x, double y, double angleDegrees) {
        return new AutoPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}