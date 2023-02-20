package frc.robot.commands.autonomous;

import frc.robot.resources.math.splines.CubicSpline;
import frc.robot.resources.math.splines.SplineGenerator;

public class Paths 
{
    static final double [][] examplePathControlPoints = 
    {
        {0,0},
        {3,5},
        {7,2}
    }; 
    public static final CubicSpline examplePath = 
        SplineGenerator.generateNaturalSpline(examplePathControlPoints);

    static final double[][] piece1ToPositionCBlueControlPoints = 
    {
        {1.89, 3.87},
        {2.70, 4.40},
        {6.59,4.58}
    };
    static final double piece1ToPositionCBlueInitialDerivative = 0;
    static final double piece1ToPositionCBlueFinalDerivative = 0.034;

    public static final CubicSpline piece1ToPositionCBlue = 
        SplineGenerator.generateHermiteClampedSpline(piece1ToPositionCBlueControlPoints, 
        piece1ToPositionCBlueInitialDerivative, piece1ToPositionCBlueFinalDerivative);



    static final double[][] piece1ToPositionCRedControlPoints = 
    {
        {1.89, 4.15},
        {2.70, 3.62},
        {6.59, 3.44}
    };
    static final double piece1ToPositionCRedInitialDerivative = 0;
    static final double piece1ToPositionCRedFinalDerivative = -0.034;

    public static final CubicSpline piece1ToPositionCRed = 
        SplineGenerator.generateHermiteClampedSpline(piece1ToPositionCRedControlPoints, 
        piece1ToPositionCRedInitialDerivative, piece1ToPositionCRedFinalDerivative);

    static final double[][] positionCToChargingBlueControlPoints = 
    {
        {1.89, 3.87},
        {3.54, 3.40}
    };
    static final double positionCToChargingBlueInitialDerivative = 0;
    static final double positionCToChargingBlueFinalDerivative = -0.25;

    public static final CubicSpline positionCToChargingBlue = 
     SplineGenerator.generateHermiteClampedSpline(positionCToChargingBlueControlPoints, 
        positionCToChargingBlueFinalDerivative, positionCToChargingBlueFinalDerivative);


    static final double[][] positionCToChargingRedControlPoints = 
    {
        {1.89, 4.15},
        {3.54, 4.62}
    };
    static final double positionCToChargingRedInitialDerivative = 0;
    static final double positionCToChargingRedFinalDerivative = 0.25;

    public static final CubicSpline positionCToChargingRed = 
        SplineGenerator.generateHermiteClampedSpline(positionCToChargingRedControlPoints, 
        positionCToChargingRedFinalDerivative, positionCToChargingRedFinalDerivative);
    
}
