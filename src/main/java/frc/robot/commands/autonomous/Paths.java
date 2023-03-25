package frc.robot.commands.autonomous;

import frc.robot.resources.math.splines.CubicSpline;
import frc.robot.resources.math.splines.SplineGenerator;

public class Paths {
    static final double[][] examplePathControlPoints = {
            { 0, 0 },
            { 3, 3 },
            { 5.5, 2 }
    };
    public static final CubicSpline examplePath = SplineGenerator.generateNaturalSpline(examplePathControlPoints);

    static final double[][] piece1ToPositionCBlueControlPoints = {
            { 1.89, 3.87 },
            { 2.70, 4.40 },
            { 6.59, 4.58 }
    };
    static final double piece1ToPositionCBlueInitialDerivative = 0;
    static final double piece1ToPositionCBlueFinalDerivative = 0.034;

    public static final CubicSpline piece1ToPositionCBlue = SplineGenerator.generateHermiteClampedSpline(
            piece1ToPositionCBlueControlPoints,
            piece1ToPositionCBlueInitialDerivative, piece1ToPositionCBlueFinalDerivative);

    static final double[][] piece1ToPositionCRedControlPoints = {
            { 1.89, 4.15 },
            { 2.70, 3.62 },
            { 6.59, 3.44 }
    };
    static final double piece1ToPositionCRedInitialDerivative = 0;
    static final double piece1ToPositionCRedFinalDerivative = -0.034;

    public static final CubicSpline piece1ToPositionCRed = SplineGenerator.generateHermiteClampedSpline(
            piece1ToPositionCRedControlPoints,
            piece1ToPositionCRedInitialDerivative, piece1ToPositionCRedFinalDerivative);

    static final double[][] positionCToChargingBlueControlPoints = {
            { 1.89, 3.87 },
            { 3.54, 3.40 }
    };
    static final double positionCToChargingBlueInitialDerivative = -0.25;
    static final double positionCToChargingBlueFinalDerivative = 0;

    public static final CubicSpline positionCToChargingBlue = SplineGenerator.generateHermiteClampedSpline(
            positionCToChargingBlueControlPoints,
            positionCToChargingBlueInitialDerivative, positionCToChargingBlueFinalDerivative);

    static final double[][] positionCToChargingRedControlPoints = {
            { 1.89, 4.15 },
            { 3.54, 4.62 }
    };
    static final double positionCToChargingRedInitialDerivative = 0.25;
    static final double positionCToChargingRedFinalDerivative = 0;

    public static final CubicSpline positionCToChargingRed = SplineGenerator.generateHermiteClampedSpline(
            positionCToChargingRedControlPoints,
            positionCToChargingRedInitialDerivative, positionCToChargingRedFinalDerivative);

    static final double[][] piece1ToPositionBBlueControlPoints = {
            { 1.82, 4.42 },
            { 6.58, 4.64 }
    };
    static final double piece1ToPositionBBlueInitialDerivative = 0;
    static final double piece1ToPositionBBlueFinalDerivative = -0.068;

    public static final CubicSpline piece1ToPositionBBlue = SplineGenerator.generateHermiteClampedSpline(
            piece1ToPositionBBlueControlPoints,
            piece1ToPositionBBlueInitialDerivative, piece1ToPositionBBlueFinalDerivative);

    static final double[][] piece1ToPositionBRedControlPoints = {
            { 1.82, 3.60 },
            { 6.58, 3.38 }
    };
    static final double piece1ToPositionBRedInitialDerivative = 0;
    static final double piece1ToPositionBRedFinalDerivative = 0.068;

    public static final CubicSpline piece1ToPositionBRed = SplineGenerator.generateHermiteClampedSpline(
            piece1ToPositionBRedControlPoints,
            piece1ToPositionBRedInitialDerivative, piece1ToPositionBRedFinalDerivative);

    static final double[][] positionBToChargingBlueControlPoints = {
            { 1.82, 4.42 },
            { 3.66, 3.46 }
    };
    static final double positionBToChargingBlueInitialDerivative = -1;
    static final double positionBToChargingBlueFinalDerivative = 0;

    public static final CubicSpline positionBToChargingBlue = SplineGenerator.generateHermiteClampedSpline(
            positionBToChargingBlueControlPoints,
            positionBToChargingBlueInitialDerivative, positionBToChargingBlueFinalDerivative);

    static final double[][] positionBToChargingRedControlPoints = {
            { 1.82, 3.60 },
            { 3.66, 4.56 }
    };
    static final double positionBToChargingRedInitialDerivative = 1;
    static final double positionBToChargingRedFinalDerivative = 0;

    public static final CubicSpline positionBToChargingRed = SplineGenerator.generateHermiteClampedSpline(
            positionBToChargingRedControlPoints,
            positionBToChargingRedInitialDerivative, positionBToChargingRedFinalDerivative);

    static final double[][] piece4ToPositionHBlueControlPoints = {
            { 1.89, 1.05 },
            { 6.58, 0.89 }
    };
    static final double piece4ToPositionHBlueInitialDerivative = 0;
    static final double piece4ToPositionHBlueFinalDerivative = 0.083;

    public static final CubicSpline piece4ToPositionHBlue = SplineGenerator.generateHermiteClampedSpline(
            piece4ToPositionHBlueControlPoints,
            piece4ToPositionHBlueInitialDerivative, piece4ToPositionHBlueFinalDerivative);

    static final double[][] piece4ToPositionHRedControlPoints = {
            { 1.89, 6.97 },
            { 6.58, 7.13 }
    };
    static final double piece4ToPositionHRedInitialDerivative = 0;
    static final double piece4ToPositionHRedFinalDerivative = -0.083;

    public static final CubicSpline piece4ToPositionHRed = SplineGenerator.generateHermiteClampedSpline(
            piece4ToPositionHRedControlPoints,
            piece4ToPositionHRedInitialDerivative, piece4ToPositionHRedFinalDerivative);

    static final double[][] positionHToChargingBlueControlPoints = {
            { 1.89, 1.05 },
            { 3.78, 2.11 }
    };
    static final double positionHToChargingBlueInitialDerivative = 1;
    static final double positionHToChargingBlueFinalDerivative = 0;

    public static final CubicSpline positionHToChargingBlue = SplineGenerator.generateHermiteClampedSpline(
            positionHToChargingBlueControlPoints,
            positionHToChargingBlueInitialDerivative, positionHToChargingBlueFinalDerivative);

    static final double[][] positionHToChargingRedControlPoints = {
            { 1.89, 6.97 },
            { 3.78, 5.91 }
    };
    static final double positionHToChargingRedInitialDerivative = -1;
    static final double positionHToChargingRedFinalDerivative = 0;

    public static final CubicSpline positionHToChargingRed = SplineGenerator.generateHermiteClampedSpline(
            positionHToChargingRedControlPoints,
            positionHToChargingRedInitialDerivative, positionHToChargingRedFinalDerivative);

}
