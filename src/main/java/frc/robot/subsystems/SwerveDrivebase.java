package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team217.*;
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
    // bot dimensions
    private final double kBotLength;
    private final double kBotWidth;
    
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

        // Configure drive motors
        for (WPI_TalonSRX driveMotor : kDriveMotors) {
            driveMotor.configFactoryDefault();
            driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        }
        // Configure turn motors
        for (WPI_TalonSRX turnMotor : kTurnMotors) {
            turnMotor.configFactoryDefault();
            turnMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

            // config Motion Magic
            turnMotor.selectProfileSlot(0, 0);
            turnMotor.config_kF(0, 1023.0 / 1000); // 1000 is the motor's max velocity
            turnMotor.config_kP(0, 0.2);
            turnMotor.config_kI(0, 0);
            turnMotor.config_kD(0, 0);

            turnMotor.configMotionCruiseVelocity(800); // around 80% of the max velocity
            turnMotor.configMotionAcceleration(8000); // around 5-10x the cruise velocity
            turnMotor.configMotionSCurveStrength(4);
        }

        // unique motor settings
        frontLeft.setSensorPhase(false);
        frontRight.setSensorPhase(false);
        backLeft.setSensorPhase(false);
        backRight.setSensorPhase(false);

        frontRight.setInverted(true); // right is inverted from left
        backRight.setInverted(true);
        
        frontLeftTurn.setSensorPhase(false);
        frontRightTurn.setSensorPhase(false);
        backLeftTurn.setSensorPhase(false);
        backRightTurn.setSensorPhase(false);
    }

    public void simulationInit() {
        for (WPI_TalonSRX driveMotor : kDriveMotors) {
            PhysicsSim.getInstance().addTalonSRX(driveMotor, 0.5, 6800);
        }
        for (WPI_TalonSRX turnMotor : kTurnMotors) {
            PhysicsSim.getInstance().addTalonSRX(turnMotor, 0.1, 1000);
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
        // NOTE: matrix starts with the front left motor and goes clockwise
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
                 * 
                 * The code block below is equivalent to and more efficient than:
                 * while (angle - currentAngle > Math.PI / 2) {
                 *     angle -= Math.PI;
                 *     speed *= -1;
                 * }
                 * while (angle - currentAngle < -Math.PI / 2) {
                 *     angle += Math.PI;
                 *     speed *= -1;
                 * }
                 */
                {
                    double currentAngle = Converter.encToRad(kTurnMotors[i].getSelectedSensorPosition(), 4096);
                    double angleDiff = angle - currentAngle;
                    // get how many half rotations we have to make to get within 90 degrees of currentAngle
                    // add signof(angleDiff) * Math.PI / 2 to angleDiff so we get within 90 degrees and not 180
                    int numHalfRotations = (int)((angleDiff + Math.signum(angleDiff) * Math.PI / 2) / Math.PI);
                    // subtract off that many half rotations
                    angle -= numHalfRotations * Math.PI;
                    if (Math.abs(numHalfRotations) % 2 == 1) { // abs on numHalfRotations so the modulus result is positive
                        // every half rotation, the wheel is flipped, so we need to flip speed
                        // odd numbers of half rotations (% 2 == 1) results in a flipped speed
                        speed *= -1;
                    }
                }
                
                // set the turn motor
                kTurnMotors[i].set(ControlMode.MotionMagic, Converter.radToEnc(angle, 4096));
            }
            else {
                kTurnMotors[i].set(ControlMode.PercentOutput, 0);
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

        double botMagnitude = Num.distance(kBotLength, kBotWidth);
        for (double[] turnVector : turnMatrix) {
            // convert turnVector into unit vector
            turnVector[0] /= botMagnitude;
            turnVector[1] /= botMagnitude;
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
