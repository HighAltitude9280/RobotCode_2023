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

    static final double[][] piece1ToPositionBBlueControlPoints = 
    {
        {1.89, 3.87},
        {2.70, 4.40},
        {6.59,4.58}
    };
    static final double piece1ToPositionBBlueInitialDerivative = 0;
    static final double piece1ToPositionBBlueFinalDerivative = 0.034;

    public static final CubicSpline piece1ToPositionBBlue = 
        SplineGenerator.generateHermiteClampedSpline(piece1ToPositionBBlueControlPoints, 
        piece1ToPositionBBlueInitialDerivative, piece1ToPositionBBlueFinalDerivative);



    static final double[][] piece1ToPositionBRedControlPoints = 
    {
        {1.89, 4.15},
        {2.70, 3.62},
        {6.59, 3.44}
    };
    static final double piece1ToPositionBRedInitialDerivative = 0;
    static final double piece1ToPositionBRedFinalDerivative = -0.034;

    public static final CubicSpline piece1ToPositionBRed = 
        SplineGenerator.generateHermiteClampedSpline(piece1ToPositionBRedControlPoints, 
        piece1ToPositionBRedInitialDerivative, piece1ToPositionBRedFinalDerivative);

    static final double[][] positionBToChargingBlueControlPoints = 
    {
        {1.89, 3.87},
        {3.54, 3.40}
    };
    static final double positionBToChargingBlueInitialDerivative = 0;
    static final double positionBToChargingBlueFinalDerivative = -0.25;

    public static final CubicSpline positionBToChargingBlue = 
     SplineGenerator.generateHermiteClampedSpline(positionBToChargingBlueControlPoints, 
        positionBToChargingBlueFinalDerivative, positionBToChargingBlueFinalDerivative);


    static final double[][] positionBToChargingRedControlPoints = 
    {
        {1.89, 4.15},
        {3.54, 4.62}
    };
    static final double positionBToChargingRedInitialDerivative = 0;
    static final double positionBToChargingRedFinalDerivative = 0.25;

    public static final CubicSpline positionBToChargingRed = 
        SplineGenerator.generateHermiteClampedSpline(positionBToChargingRedControlPoints, 
        positionBToChargingRedFinalDerivative, positionBToChargingRedFinalDerivative);
    
}
