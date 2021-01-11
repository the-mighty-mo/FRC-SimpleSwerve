package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team217.Converter;
import org.team217.Num;
import org.team217.ctre.WPI_TalonSRX;

import frc.robot.Constants;
import frc.robot.sim.PhysicsSim;

/**
 * Manages a swerve drivebase
 * 
 * @author Benjamin Hall
 */
public class SwerveDrivebase extends SubsystemBase {
    // drive motors
    private final WPI_TalonSRX[] kDriveMotors;
    // turn motors
    private final WPI_TalonSRX[] kTurnMotors;

    private final double kBotLength;
    private final double kBotWidth;

    private double[][] lastAngles = new double[4][2]; // 2 elements are angle and inverted (1 or -1)

    /**
     * Manages a swerve drivebase.
     * 
     * @param botLength
     *        Length of the bot, measured as the distance between the center of the front wheel and the center of the rear wheel
     * @param botWidth
     *        Width of the bot, measured as the distance between the center of the left wheel and the center of the right wheel
     * 
     * @author Benjamin Hall
     */
    public SwerveDrivebase(double botLength, double botWidth) {
        this.kBotLength = botLength;
        this.kBotWidth = botWidth;

        // Create motors
        WPI_TalonSRX frontLeft = new WPI_TalonSRX(Constants.kFrontLeftID);
        WPI_TalonSRX frontRight = new WPI_TalonSRX(Constants.kFrontRightID);
        WPI_TalonSRX backLeft = new WPI_TalonSRX(Constants.kBackLeftID);
        WPI_TalonSRX backRight = new WPI_TalonSRX(Constants.kBackRightID);

        WPI_TalonSRX frontLeftTurn = new WPI_TalonSRX(Constants.kFrontLeftTurnID);
        WPI_TalonSRX frontRightTurn = new WPI_TalonSRX(Constants.kFrontRightTurnID);
        WPI_TalonSRX backLeftTurn = new WPI_TalonSRX(Constants.kBackLeftTurnID);
        WPI_TalonSRX backRightTurn = new WPI_TalonSRX(Constants.kBackRightTurnID);

        // Add motors to arrays
        this.kDriveMotors = new WPI_TalonSRX[] { frontLeft, frontRight, backRight, backLeft };
        this.kTurnMotors = new WPI_TalonSRX[] { frontLeftTurn, frontRightTurn, backRightTurn, backLeftTurn };

        // Configure motors
        for (WPI_TalonSRX driveMotor : kDriveMotors) {
            driveMotor.configFactoryDefault();
            driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        }
        for (WPI_TalonSRX turnMotor : kTurnMotors) {
            turnMotor.configFactoryDefault();
            turnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

            // config Motion Magic
            turnMotor.selectProfileSlot(0, 0);
            turnMotor.config_kF(0, 1023.0 / 2000); // 2000 is the motor's max velocity
            turnMotor.config_kP(0, 0.2);
            turnMotor.config_kI(0, 0);
            turnMotor.config_kD(0, 0);

            turnMotor.configMotionCruiseVelocity(1000);
            turnMotor.configMotionAcceleration(10000);
            turnMotor.configMotionSCurveStrength(4);
        }

        // Set the inverted portion to 1 by default
        for (double[] lastAngle : lastAngles) {
            lastAngle[1] = 1;
        }
    }

    public void simulationInit() {
        for (WPI_TalonSRX driveMotor : kDriveMotors) {
            PhysicsSim.getInstance().addTalonSRX(driveMotor, 0.5, 6800);
        }
        for (WPI_TalonSRX turnMotor : kTurnMotors) {
            PhysicsSim.getInstance().addTalonSRX(turnMotor, 0.1, 2000);
        }
    }

    /**
     * Calculates and runs swerve drive.
     * 
     * @param speed
     *        forward speed from -1.0 (backward) to 1.0 (forward)
     * @param strafe
     *        strafe speed from -1.0 (left) to 1.0 (right)
     * @param turn
     *        turn speed from -1.0 (left) to 1.0 (right)
     */
    public void set(double speed, double strafe, double turn) {
        // NOTE: matrix starts with the leftMaster and goes clockwise
        double[][] wheelMatrix = getStrafeMatrix(speed, strafe);

        // need to keep track of the max magnitude of the wheel vectors
        // since none of them can be greater than 1
        double maxWheelMagnitude = 1;

        // variables in this block are not accessible elsewhere
        {
            // get turn matrix
            double[][] turnMatrix = getTurnMatrix(turn);

            for (int i = 0; i < wheelMatrix.length; i++) {
                // add turnMatrix to wheel matrix
                wheelMatrix[i][0] += turnMatrix[i][0];
                wheelMatrix[i][1] += turnMatrix[i][1];
                // update max magnitude
                double wheelMagnitude = Num.distance(wheelMatrix[i]); // fun fact: Num.distance(double... axis) can accept a 1-D array
                maxWheelMagnitude = Math.max(maxWheelMagnitude, wheelMagnitude);
            }
        }

        // divide all wheel vectors by the max magnitude so none exceed 1
        for (double[] wheelVector : wheelMatrix) {
            wheelVector[0] /= maxWheelMagnitude;
            wheelVector[1] /= maxWheelMagnitude;
        }

        // set the motors given the wheel matrix
        set(wheelMatrix);
    }

    /**
     * Calculates and runs swerve drive.
     * 
     * @param wheelMatrix
     *        The matrix of wheel vectors to run
     */
    public void set(double[][] wheelMatrix) {
        /*
         * We need to convert all vectors into:
         * - speed
         * - angle
         */
        for (int i = 0; i < wheelMatrix.length; i++) {
            // speed is magnitude of vector
            double speed = Num.distance(wheelMatrix[i]);
            // only calculate angle if we're trying to move
            if (speed != 0) {
                // calculate angle
                // NOTE: 0 rad is up, increases going clockwise, so x and y are flipped
                double angle = Math.atan2(wheelMatrix[i][0], wheelMatrix[i][1]);
    
                /*
                 * We want to make it so the wheels turn as little as possible, so
                 * we need to optimize the angle. Theoretically, a wheel should never
                 * have to turn more than 90 degrees from its current position.
                 */
                lastAngles[i][1] = 1;
                while (angle - lastAngles[i][0] > Math.PI / 2) {
                    angle -= Math.PI;
                    lastAngles[i][1] = -1;
                }
                while (angle - lastAngles[i][0] < -Math.PI / 2) {
                    angle += Math.PI;
                    lastAngles[i][1] = -1;
                }
                lastAngles[i][0] = angle;
                speed *= lastAngles[i][1]; // invert speed if necessary
                
                // set the turn motor
                kTurnMotors[i].set(ControlMode.MotionMagic, Converter.radToEnc(angle, 4096));
            }
            // set the drive motor
            kDriveMotors[i].set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Returns the matrix representing movement vectors for all the wheels.
     * 
     * @param speed
     *        forward speed from -1.0 (backward) to 1.0 (forward)
     * @param strafe
     *        strafe speed from -1.0 (left) to 1.0 (right)
     */
    private double[][] getStrafeMatrix(double speed, double strafe) {
        double[][] strafeMatrix = new double[4][2]; // 4 vectors, each containing 2 elements (x and y)
        for (double[] strafeVector : strafeMatrix) {
            strafeVector[0] = strafe; // strafe is x
            strafeVector[1] = speed; // speed is y
        }
        return strafeMatrix;
    }

    /**
     * Returns the matrix representing turning vectors for all the wheels.
     * 
     * @param turn
     *        turn speed from -1.0 (left) to 1.0 (right)
     */
    private double[][] getTurnMatrix(double turn) {
        double[][] turnMatrix = new double[4][2]; // 4 vectors, each containing 2 elements (x and y)

        /* 
         * Wheels should form a circle during a turn. The x component of the
         * turn vector is kBotLength, and the y component is kBotWidth.
         * Convert to unit vector and multiply by turn.
         * 
         * Wheel orientations for a clockwise turn:
         * 
         *   ^                \
         *  /                  \
         * /                    v
         * 
         * ^                    /
         *  \                  /
         *   \                v
         * 
         * If turn is negative, flip the direction.
         */
        turnMatrix[0][0] = kBotLength;
        turnMatrix[0][1] = kBotWidth;

        turnMatrix[1][0] = kBotLength;
        turnMatrix[1][1] = -kBotWidth;

        turnMatrix[2][0] = -kBotLength;
        turnMatrix[2][1] = -kBotWidth;

        turnMatrix[3][0] = -kBotLength;
        turnMatrix[3][1] = kBotWidth;

        for (double[] turnVector : turnMatrix) {
            // convert turnVector into unit vector
            turnVector[0] /= Num.distance(kBotLength, kBotWidth);
            turnVector[1] /= Num.distance(kBotLength, kBotWidth);
            // multiply by scalar (turn)
            turnVector[0] *= turn;
            turnVector[1] *= turn;
        }
        
        return turnMatrix;
    }

    /**
     * Stops the drivebase.
     */
    public void stop() {
        for (WPI_TalonSRX driveMotor : kDriveMotors) {
            driveMotor.neutralOutput();
        }
        for (WPI_TalonSRX turnMotor : kTurnMotors) {
            turnMotor.neutralOutput();
        }
    }

    /**
     * Resets the encoders for the drive motors.
     */
    public void resetDriveEncoders() {
        for (WPI_TalonSRX driveMotor : kDriveMotors) {
            driveMotor.setSelectedSensorPosition(0);
        }
    }

    /**
     * Initializes the encoders for the turn motors.
     * <p>
     * Move your turn motors into their zero position, then call this method.
     */
    public void initTurnEncoders() {
        for (WPI_TalonSRX turnMotor : kTurnMotors) {
            turnMotor.setSelectedSensorPosition(0);
        }
    }
}
