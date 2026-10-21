// Units: field frame in inches, time in seconds, angles in radians.
#pragma once

#include "Eigen/Dense"
#include <array>
#include <memory>
#include <vector>

using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

struct TrajPoint {
    double t;
    Vec3 pos;  // x, y, theta (field frame, theta unwrapped/continuous)
    Vec3 vel;  // vx, vy, omega
    Vec3 acc;  // ax, ay, alpha
};

// Length (inches) of the "roughly straight" lead-in and lead-out.
constexpr double kStraightLen = 6.0;

// How far (inches) the curve may drift sideways from the start/end heading line
// within kStraightLen of the start or end. buildBezierPath scales the control point
// distance up until the curve stays within this.
constexpr double kStraightTol = 0.25;

// Arc length (inches) at the start and at the end over which heading is held
// constant. Heading changes only between these two sections.
constexpr double kHeadingHoldLen = 6.0;

// Follower control-loop period (seconds). The trajectory time step matches it, so the
// follower reads one TrajPoint per tick.
constexpr double kControlDt = 0.010;

// Pointer to a long vector of TrajPoints. shared_ptr so the generator and a path
// follower can both hold it; switch to std::unique_ptr if only one owner is needed.
// Two kinds of vector are passed around:
//   path vector:       evenly spaced in DISTANCE (ds); only pos is filled in
//   trajectory vector: evenly spaced in TIME (dt);     t, pos, vel, acc all filled in
using TrajPtr = std::shared_ptr<std::vector<TrajPoint>>;

// Limits on the speed profile. Set these before generating.
// The defaults are placeholders.
struct TrajConstraints {
    double maxVel   = 60.0;  // in/s    cap on linear speed
    double maxAccel = 60.0;  // in/s^2  cap on total linear acceleration magnitude
    double maxOmega = 6.0;   // rad/s   cap on heading rate
    double maxAlpha = 20.0;  // rad/s^2 cap on heading angular acceleration
};

// One Bezier curve (degree 4) in x, y. ctrl is ordered start -> end:
//   ctrl[0] = start.xy
//   ctrl[1] = start.xy + leadDist * (cos(start.theta), sin(start.theta))
//   ctrl[2] = middle control point, chosen by computeMiddleControlPoint
//   ctrl[3] = end.xy   - leadDist * (cos(end.theta),   sin(end.theta))
//   ctrl[4] = end.xy
// ctrl[1] and ctrl[3] make the curve leave and arrive along start.theta and end.theta.
// theta of start / end is the DIRECTION OF TRAVEL at that end. The robot heading is
// separate: generatePath takes startHeading and endHeading.
// leadDist is NOT simply 6 in: a control point 6 in out only gives a curve that is
// straight for well under 6 in of path length, so leadDist is scaled up (see
// buildBezierPath) until the first and last kStraightLen of the curve stay within
// kStraightTol of the heading line.
struct BezierPath {
    Vec3 start;                 // x, y, theta (given); theta = direction of travel
    Vec3 end;                   // x, y, theta (given); theta = direction of travel
    double leadDist;            // distance of ctrl[1] / ctrl[3] from start / end
    std::array<Vec2, 5> ctrl;
};

// Step 1 (geometry): takes the start and end points (x, y, theta) plus the robot heading
// at each end (startHeading, endHeading, radians), and returns the path as a vector of
// points evenly spaced along the curve, no more than ds inches apart (first point =
// start, last point = end; ds below 0.01 is treated as 0.01).
//   start.theta, end.theta  direction of travel where the curve leaves / arrives. They
//                           set where the outer control points go; they are not the
//                           robot heading.
//   startHeading, endHeading  robot heading. Fills pos.z of every point through
//                           headingAtArcLength: held at startHeading for the first
//                           kHeadingHoldLen of the path and at endHeading (unwrapped)
//                           for the last, with a smooth change in between.
// Only pos is filled in; t, vel and acc are zero. x, y come from the Bezier built by
// buildBezierPath. omega and alpha are not computed here: applyMotionProfile derives
// them from the theta values in the points.
TrajPtr generatePath(const Vec3& start, const Vec3& end,
                     double startHeading, double endHeading,
                     double ds = 0.1);

// Builds the Bezier curve. start.theta and end.theta (direction of travel) set where
// control points 1 and 3 go.
// Control point distance scaling: starts at leadDist = kStraightLen (or a third of the
// start-to-end distance if that is smaller, so a short straight move stays straight
// and doesn't double back) and grows in steps
// of 25% (up to 4x kStraightLen) until the curve is within kStraightTol of the heading
// line at kStraightLen of arc length from both ends. It is not limited by the
// start-to-end distance, so a short move with a sharp turn gets a long leadDist (and
// the curve swings out past the end point). If the tolerance can't be met, the
// closest candidate is used.
BezierPath buildBezierPath(const Vec3& start, const Vec3& end);

// Middle control point from ctrl[1], ctrl[3] and the start/end travel directions: intersection of
// the line from ctrl[1] along start.theta with the line into ctrl[3] along end.theta.
// If those directions are (nearly) parallel, the intersection is behind either point,
// or it is more than 2x the ctrl[1]-ctrl[3] distance away, fall back to the midpoint
// of ctrl[1] and ctrl[3].
Vec2 computeMiddleControlPoint(const Vec2& p1, double startTheta,
                               const Vec2& p3, double endTheta);

// Curve evaluation in the Bezier parameter u in [0, 1].
Vec2 bezierPoint(const BezierPath& path, double u);
Vec2 bezierFirstDeriv(const BezierPath& path, double u);   // dP/du
Vec2 bezierSecondDeriv(const BezierPath& path, double u);  // d2P/du2

// Arc-length lookup (the Bezier parameter u is not proportional to distance).
// Table holds cumulative arc length at n+1 evenly spaced values of u.
std::vector<double> buildArcLengthTable(const BezierPath& path, int n = 500);
double uFromArcLength(const std::vector<double>& table, double s);  // inverse lookup
double totalLength(const std::vector<double>& table);

// ---- Step 2 (motion): run later, on the path vector from generatePath ----

// Cumulative distance along the path vector: result[i] = distance from point 0 to i,
// measured along xy (so result.back() is the total length).
std::vector<double> cumulativeArcLength(const std::vector<TrajPoint>& path);

// Speed v at each point of the path vector. Starts and ends at rest and is limited by:
//   maxVel                           -> v <= maxVel
//   maxAccel                         -> trapezoid in distance, and v^2 * |curvature| within maxAccel
//   maxOmega (omega = v * dtheta/ds) -> v <= maxOmega / |dtheta/ds|, dtheta/ds from pos.z
//   maxAlpha (alpha = d(omega)/dt)   -> forward/backward pass so |alpha| <= maxAlpha
// Acceleration is constant between neighbouring points, so each segment uses the smaller
// of the limits at its two ends. Curvature and dtheta/ds come from finite differences of
// pos (x, y, theta) along the path vector, so ds in generatePath should be small relative
// to the tightest curve radius; on very tight curves the limits can be exceeded by a few
// percent at ds = 0.1 (about 6% at 0.25, under 2% at 0.05).
std::vector<double> buildSpeedProfile(const std::vector<TrajPoint>& path,
                                      const std::vector<double>& arcLength,
                                      const TrajConstraints& limits);

// Time at which each path point is reached, from the speed profile. Uses the average
// speed between neighbours (dt_i = 2 * ds_i / (v_i + v_i+1)) so the zero speed at the
// ends doesn't cause a divide by zero. result[0] = 0.
std::vector<double> timeAtPoints(const std::vector<double>& arcLength,
                                 const std::vector<double>& speed);

// Takes the path vector from generatePath and the limits, and returns a NEW
// trajectory vector sampled at a fixed time step dt: t = 0, dt, 2*dt, ...
// For each sample time, the distance travelled is found from the speed profile, then
//   pos:   interpolated from the path vector (x, y and theta as generatePath set them)
//   vel:   vx, vy = v * unit tangent;  omega = v * dtheta/ds
//   acc:   ax, ay = dv/dt * tangent + v^2 * curvature * normal;
//          alpha = dv/dt * dtheta/ds + v^2 * d2theta/ds2     (= d(omega)/dt)
// omega and alpha are worked out here from the theta values in the path points.
// Acceleration is piecewise constant (trapezoidal speed profile), so it steps between
// path-point segments and there is no jerk limit.
// The last sample is the end pose with zero vel and acc
// (the final step is held at the end, so the vector length is ceil(total time / dt) + 1).
TrajPtr applyMotionProfile(const TrajPtr& path,
                           const TrajConstraints& limits,
                           double dt = kControlDt);

// Robot heading as a function of distance s along the path (total length totalLen),
// from a start and end heading angle. generatePath uses it with startHeading and
// endHeading to fill pos.z. Independent of the curve's direction:
//   s in [0, kHeadingHoldLen]                      -> theta = startHeading (constant)
//   s in [totalLen - kHeadingHoldLen, totalLen]    -> theta = endHeading, unwrapped (constant)
//   in between                                     -> smooth blend from start to end
//                                                     (quintic, so omega and alpha are
//                                                     zero where it joins the held parts)
// The turn used is dtheta = unwrapAngle(startHeading, endHeading) - startHeading, i.e. the
// shortest way round, so theta stays continuous. If totalLen < 3 * kHeadingHoldLen the
// hold lengths shrink to totalLen / 3 so the blend section keeps some length.
struct HeadingState {
    double theta;           // rad
    double dthetaDs;        // rad/in
    double d2thetaDs2;      // rad/in^2
};
HeadingState headingAtArcLength(double startHeading, double endHeading,
                                double s, double totalLen);

// Shift raw angle by multiples of 2*pi so it is within pi of prev.
double unwrapAngle(double prev, double raw);