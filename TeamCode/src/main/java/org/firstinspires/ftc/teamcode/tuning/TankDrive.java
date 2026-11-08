package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.unused.Drawing;

import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

@Config
public final class TankDrive {
    public static class Params {
        // IMU orientation
        // TODO: fill in these values based on
        //   see https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        public double inPerTick = 0.00195606972005;
        public double trackWidthTicks = 6014.062817079626;

        // feedforward parameters (in tick units)
        public double kS = 0.7575599173455769;
        public double kV = 0.0003949165535798532;
        public double kA = 0.0000255;

        // path profile parameters (in inches)
        public double maxWheelVel = 45;
        public double minProfileAccel = -20;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = 1.5*Math.PI; // shared with path
        public double maxAngAccel = 1.5*Math.PI;

        // path controller gains
        public double ramseteZeta = 0.8; // in the range (0, 1)
        public double ramseteBBar = 4; // positive

        // turn controller gains
        // sign-corrected in TurnAction below: error = target - actual, so positive gains are correct
        public double turnGain = 2.0;
        public double turnVelGain = 0.1;

        // extra voltage that makes the outer wheel push harder / inner wheel less whenever the robot
        // is rotating (skid-steer scrub). 0 = off. Tune with TrackWidthCheck until it reads ~360.
        public double kTurn = 3.5;

        // closed-loop finishing for turn() : keep correcting after the timed profile ends
        public double turnTolDeg = 1.0;         // done when within this many degrees...
        public double turnSettleTimeout = 2.0; // ...or after this many seconds (0 = off, pure open loop)
        public double turnMinPower = 0.25;      // motor power floor while still short of the target

        // closed-loop finishing for splines / lines: correct the last few inches
        public double endPosTolIn = .75;
        public double endHeadTolDeg = 3.0;
        public double endSettleTimeout = 1.5;   // seconds (0 = off)
        public double endPosGain = 4.0;         // in/s of speed per inch of error
        public double endHeadGain = 3.0;        // rad/s per rad of error
        public double endMinPower = 0.15;

        public double kTurnSpin = 0.0;            // volts, used only by turn()
        public double turnSettleMaxAngVel = 1.5;  // rad/s cap while finishing a turn

    }

    public static Params PARAMS = new Params();

    public final TankKinematics kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final List<DcMotorEx> leftMotors, rightMotors;

    public final LazyImu lazyImu;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);

    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final List<Encoder> leftEncs, rightEncs;
        private Pose2d pose;

        private double lastLeftPos, lastRightPos;
        private boolean initialized;

        public DriveLocalizer(Pose2d pose) {
            {
                List<Encoder> leftEncs = new ArrayList<>();
                for (DcMotorEx m : leftMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    leftEncs.add(e);
                }
                this.leftEncs = Collections.unmodifiableList(leftEncs);
            }

            {
                List<Encoder> rightEncs = new ArrayList<>();
                for (DcMotorEx m : rightMotors) {
                    Encoder e = new OverflowEncoder(new RawEncoder(m));
                    rightEncs.add(e);
                }
                this.rightEncs = Collections.unmodifiableList(rightEncs);
            }

            // TODO: reverse encoder directions if needed
            //   leftEncs.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            Twist2dDual<Time> delta;

            List<PositionVelocityPair> leftReadings = new ArrayList<>(), rightReadings = new ArrayList<>();
            double meanLeftPos = 0.0, meanLeftVel = 0.0;
            for (Encoder e : leftEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanLeftPos += p.position;
                meanLeftVel += p.velocity;
                leftReadings.add(p);
            }
            meanLeftPos /= leftEncs.size();
            meanLeftVel /= leftEncs.size();

            double meanRightPos = 0.0, meanRightVel = 0.0;
            for (Encoder e : rightEncs) {
                PositionVelocityPair p = e.getPositionAndVelocity();
                meanRightPos += p.position;
                meanRightVel += p.velocity;
                rightReadings.add(p);
            }
            meanRightPos /= rightEncs.size();
            meanRightVel /= rightEncs.size();

            FlightRecorder.write("TANK_LOCALIZER_INPUTS",
                    new TankLocalizerInputsMessage(leftReadings, rightReadings));

            if (!initialized) {
                initialized = true;

                lastLeftPos = meanLeftPos;
                lastRightPos = meanRightPos;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);

            }

            Twist2dDual<Time> twist = kinematics.forward(new TankKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            meanLeftPos - lastLeftPos,
                            meanLeftVel
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            meanRightPos - lastRightPos,
                            meanRightVel,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftPos = meanLeftPos;
            lastRightPos = meanRightPos;

            pose = pose.plus(twist.value());

            return twist.velocity().value();
        }
    }

    public TankDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   add additional motors on each side if you have them
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "left_drive"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "right_drive"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse motor directions if needed
        rightMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        FlightRecorder.write("TANK_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    private void setSidePowers(double left, double right) {
        for (DcMotorEx m : leftMotors) m.setPower(left);
        for (DcMotorEx m : rightMotors) m.setPower(right);
    }

    /** feedforward motor powers {left, right} for a robot-velocity command */
    private double[] feedforwardPowers(PoseVelocity2dDual<Time> command) {
        return feedforwardPowers(command, PARAMS.kTurn);
    }

    private double[] feedforwardPowers(PoseVelocity2dDual<Time> command, double kTurn) {
        TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
        double voltage = voltageSensor.getVoltage();
        final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
        double omega = command.angVel.value();
        double dv = kTurn * Math.tanh(omega / 0.5);
        double left = (feedforward.compute(wheelVels.left) - dv) / voltage;
        double right = (feedforward.compute(wheelVels.right) + dv) / voltage;
        tankCommandWriter.write(new TankCommandMessage(voltage, left, right));
        return new double[]{left, right};
    }

    /** scale both powers (keeping their ratio) so the larger one is at least min */
    private static double[] liftToMin(double[] pw, double min) {
        double m = Math.max(Math.abs(pw[0]), Math.abs(pw[1]));
        if (m > 0 && m < min) {
            double k = min / m;
            return new double[]{pw[0] * k, pw[1] * k};
        }
        return pw;
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                // the profile is over: keep correcting the last few inches instead of coasting past the end
                if (PARAMS.endSettleTimeout > 0
                        && t < timeTrajectory.duration + PARAMS.endSettleTimeout
                        && settleToEnd(p)) {
                    return true;
                }
                setSidePowers(0, 0);
                return false;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            // shared feedforward (includes kTurn)
            double[] pw = feedforwardPowers(command);
            double leftPower = pw[0];
            double rightPower = pw[1];

            for (DcMotorEx m : leftMotors) {
                m.setPower(leftPower);
            }
            for (DcMotorEx m : rightMotors) {
                m.setPower(rightPower);
            }

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        /** returns true while it is still driving toward the end pose, false once close enough */
        private boolean settleToEnd(TelemetryPacket p) {
            Pose2d goal = timeTrajectory.path.get(timeTrajectory.path.length(), 1).value();
            updatePoseEstimate();
            Pose2d pose = localizer.getPose();

            double dx = goal.position.x - pose.position.x;
            double dy = goal.position.y - pose.position.y;
            double h = pose.heading.toDouble();
            double forwardErr = dx * Math.cos(h) + dy * Math.sin(h);   // + = still short of the end
            double headErr = goal.heading.minus(pose.heading);          // target - actual

            p.put("end forwardErr", forwardErr);
            p.put("end headErr (deg)", Math.toDegrees(headErr));

            if (Math.abs(forwardErr) < PARAMS.endPosTolIn
                    && Math.abs(headErr) < Math.toRadians(PARAMS.endHeadTolDeg)) {
                return false;
            }

            double v = Math.max(-12, Math.min(12, PARAMS.endPosGain * forwardErr));
            double w = Math.max(-Math.PI, Math.min(Math.PI, PARAMS.endHeadGain * headErr));
            PoseVelocity2dDual<Time> command =
                    PoseVelocity2dDual.constant(new PoseVelocity2d(new Vector2d(v, 0), w), 3);
            double floor = Math.abs(forwardErr) < PARAMS.endPosTolIn ? PARAMS.turnMinPower : PARAMS.endMinPower;
            double[] pw = liftToMin(feedforwardPowers(command), floor);
            setSidePowers(pw[0], pw[1]);
            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            boolean settling = t >= turn.duration;
            if (settling && (PARAMS.turnSettleTimeout <= 0
                    || t >= turn.duration + PARAMS.turnSettleTimeout)) {
                setSidePowers(0, 0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(Math.min(t, turn.duration));
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            // target - actual (the original quickstart had this reversed, which made positive gains unstable)
            double err = txWorldTarget.heading.value().minus(localizer.getPose().heading);
            p.put("turn err (deg)", Math.toDegrees(err));

            if (settling && Math.abs(err) < Math.toRadians(PARAMS.turnTolDeg)
                    && Math.abs(robotVelRobot.angVel) < 0.6) {
                setSidePowers(0, 0);
                return false;
            }

            DualNum<Time> w;
            if (!settling) {
                w = txWorldTarget.heading.velocity().plus(
                        PARAMS.turnGain * err
                                + PARAMS.turnVelGain * (txWorldTarget.heading.velocity().value() - robotVelRobot.angVel));
            } else {
                double wSettle = Math.max(-PARAMS.maxAngVel, Math.min(PARAMS.maxAngVel, PARAMS.turnGain * err));
                w = DualNum.<Time>constant(wSettle, 3);
            }

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3), w);
            driveCommandWriter.write(new DriveCommandMessage(command));

            double[] pw = feedforwardPowers(command);
            if (Math.abs(err) > Math.toRadians(PARAMS.turnTolDeg)) {
                pw = liftToMin(pw, PARAMS.turnMinPower);
            }
            setSidePowers(pw[0], pw[1]);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));


        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}