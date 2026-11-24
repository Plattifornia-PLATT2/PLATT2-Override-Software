// path_generation.cpp - Step 1: generatePath and the helpers it needs.
// Build: g++ -std=c++17 -O2 -I /usr/include/eigen3 -c path_generation.cpp
#include "platt2/robot/subsystems/holonomicDrive/autonControl/trajectoryGen.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

// Bernstein basis of degree n at u.
double bernstein(int n, int i, double u) {
    double binom = 1.0;
    for (int k = 1; k <= i; ++k) binom *= static_cast<double>(n - i + k) / k;
    return binom * std::pow(u, i) * std::pow(1.0 - u, n - i);
}

double cross2(const Vec2& a, const Vec2& b) { return a.x() * b.y() - a.y() * b.x(); }

constexpr double kMaxLeadScale = 4.0;   // leadDist never exceeds this x kStraightLen
constexpr double kLeadStepScale = 0.25; // leadDist grows by this x kStraightLen per try

// Builds the Bezier for a given control point distance d.
BezierPath makeBezier(const Vec3& start, const Vec3& end, double d) {
    BezierPath path;
    path.start = start;
    path.end = end;
    path.leadDist = d;

    // theta of start / end is the direction of travel at that end.
    const Vec2 dirStart(std::cos(start.z()), std::sin(start.z()));
    const Vec2 dirEnd(std::cos(end.z()), std::sin(end.z()));

    path.ctrl[0] = start.head<2>();
    path.ctrl[1] = path.ctrl[0] + d * dirStart;
    path.ctrl[4] = end.head<2>();
    path.ctrl[3] = path.ctrl[4] - d * dirEnd;
    path.ctrl[2] = computeMiddleControlPoint(path.ctrl[1], start.z(),
                                             path.ctrl[3], end.z());
    return path;
}

// Worst sideways distance (inches) of the curve from the start heading line, measured
// kStraightLen of arc length from the start, and from the end heading line, measured
// kStraightLen of arc length before the end.
double worstLeadDeviation(const BezierPath& bez, TrajConstraints constraints) {
    const std::vector<double> table = buildArcLengthTable(bez, 200);
    const double L = totalLength(table);
    const double ell = std::min(constraints.kStraightLen, 0.5 * L);

    const Vec2 dirStart(std::cos(bez.start.z()), std::sin(bez.start.z()));
    const Vec2 dirEnd(std::cos(bez.end.z()), std::sin(bez.end.z()));

    const Vec2 a = bezierPoint(bez, uFromArcLength(table, ell));
    const Vec2 b = bezierPoint(bez, uFromArcLength(table, L - ell));

    const double devStart = std::abs(cross2(dirStart, a - bez.ctrl[0]));
    const double devEnd   = std::abs(cross2(dirEnd,   b - bez.ctrl[4]));
    return std::max(devStart, devEnd);
}

}  // namespace

double unwrapAngle(double prev, double raw) {
    // remainder() returns a value in [-pi, pi], so the result is within pi of prev.
    return prev + std::remainder(raw - prev, 2.0 * kPi);
}

Vec2 computeMiddleControlPoint(const Vec2& p1, double startTheta,
                               const Vec2& p3, double endTheta) {
    const Vec2 mid = 0.5 * (p1 + p3);
    const Vec2 d0(std::cos(startTheta), std::sin(startTheta));
    const Vec2 d2(std::cos(endTheta), std::sin(endTheta));
    const Vec2 r = p3 - p1;
    const double chord = r.norm();

    // Solve p1 + a*d0 = p3 - b*d2 for (a, b).
    const double det = d0.x() * d2.y() - d2.x() * d0.y();
    if (std::abs(det) < 1e-6 || chord < 1e-9) return mid;  // parallel or degenerate

    const double a = (r.x() * d2.y() - d2.x() * r.y()) / det;
    const double b = (d0.x() * r.y() - r.x() * d0.y()) / det;
    if (a < 0.0 || b < 0.0) return mid;                    // intersection behind a point
    if (a > 2.0 * chord || b > 2.0 * chord) return mid;    // intersection too far away

    return p1 + a * d0;
}

BezierPath buildBezierPath(const Vec3& start, const Vec3& end, TrajConstraints constraints) {
    // No cap based on the start-to-end distance: a short move that turns sharply needs
    // control points much farther out than the move is long.
    const double maxD = kMaxLeadScale * constraints.kStraightLen;

    // First try: kStraightLen, but no more than a third of the start-to-end distance.
    // Otherwise on a short move the two outer control points pass each other and the
    // curve doubles back on itself, even for a perfectly straight move.
    const double chord = (end.head<2>() - start.head<2>()).norm();
    const double firstD = std::min(constraints.kStraightLen, chord / 3.0);

    BezierPath best = makeBezier(start, end, firstD);
    double bestDev = worstLeadDeviation(best, constraints);

    for (double d = firstD + constraints.kStraightLen * kLeadStepScale;
         bestDev > kStraightTol && d <= maxD + 1e-9;
         d += constraints.kStraightLen * kLeadStepScale) {
        const BezierPath cand = makeBezier(start, end, d);
        const double dev = worstLeadDeviation(cand, constraints);
        if (dev < bestDev) {
            best = cand;
            bestDev = dev;
        }
    }
    return best;
}

Vec2 bezierPoint(const BezierPath& path, double u) {
    Vec2 p = Vec2::Zero();
    for (int i = 0; i <= 4; ++i) p += bernstein(4, i, u) * path.ctrl[i];
    return p;
}

Vec2 bezierFirstDeriv(const BezierPath& path, double u) {
    Vec2 d = Vec2::Zero();
    for (int i = 0; i <= 3; ++i)
        d += bernstein(3, i, u) * (path.ctrl[i + 1] - path.ctrl[i]);
    return 4.0 * d;
}

Vec2 bezierSecondDeriv(const BezierPath& path, double u) {
    Vec2 d = Vec2::Zero();
    for (int i = 0; i <= 2; ++i)
        d += bernstein(2, i, u) *
             (path.ctrl[i + 2] - 2.0 * path.ctrl[i + 1] + path.ctrl[i]);
    return 12.0 * d;
}

std::vector<double> buildArcLengthTable(const BezierPath& path, int n) {
    n = std::max(n, 1);
    std::vector<double> table(n + 1, 0.0);
    Vec2 prev = bezierPoint(path, 0.0);
    for (int i = 1; i <= n; ++i) {
        const Vec2 cur = bezierPoint(path, static_cast<double>(i) / n);
        table[i] = table[i - 1] + (cur - prev).norm();
        prev = cur;
    }
    return table;
}

double totalLength(const std::vector<double>& table) {
    return table.empty() ? 0.0 : table.back();
}

double uFromArcLength(const std::vector<double>& table, double s) {
    const int n = static_cast<int>(table.size()) - 1;
    if (n < 1) return 0.0;
    if (s <= 0.0) return 0.0;
    if (s >= table.back()) return 1.0;

    // First entry strictly greater than s; the segment is [idx-1, idx].
    const auto it = std::upper_bound(table.begin(), table.end(), s);
    const int idx = static_cast<int>(it - table.begin());
    const double s0 = table[idx - 1];
    const double s1 = table[idx];
    const double frac = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;
    return (idx - 1 + frac) / n;
}

HeadingState headingAtArcLength(double startHeading, double endHeading,
                                double s, double totalLen) {
    const double theta0 = startHeading;
    const double dTheta = unwrapAngle(theta0, endHeading) - theta0;

    const double hold = std::min(kHeadingHoldLen, totalLen / 3.0);
    const double mid = totalLen - 2.0 * hold;
    if (mid < 1e-9) {  // zero-length path: nothing to blend
        return {s < 0.5 * totalLen ? theta0 : theta0 + dTheta, 0.0, 0.0};
    }

    // Quintic smoothstep over the middle section. Its first and second derivatives
    // are zero at both ends, so omega and alpha are zero where it meets the held parts.
    const double sigma = std::clamp((s - hold) / mid, 0.0, 1.0);
    const double s2 = sigma * sigma;
    const double s3 = s2 * sigma;
    const double h   = 10.0 * s3 - 15.0 * s2 * s2 + 6.0 * s2 * s3;
    const double hp  = 30.0 * s2 * (1.0 - sigma) * (1.0 - sigma);
    const double hpp = 60.0 * sigma - 180.0 * s2 + 120.0 * s3;

    return {theta0 + dTheta * h, dTheta * hp / mid, dTheta * hpp / (mid * mid)};
}

TrajPtr generatePath(const Vec3& start, const Vec3& end,
                     double startHeading, double endHeading, TrajConstraints constraints, double ds) {
    ds = std::max(ds, 0.01);

    const BezierPath bez = buildBezierPath(start, end, constraints);
    const std::vector<double> table = buildArcLengthTable(bez);
    const double L = totalLength(table);

    const int n = std::max(1, static_cast<int>(std::ceil(L / ds)));
    auto path = std::make_shared<std::vector<TrajPoint>>();
    path->reserve(n + 1);

    for (int i = 0; i <= n; ++i) {
        const double s = L * static_cast<double>(i) / n;
        const Vec2 xy = bezierPoint(bez, uFromArcLength(table, s));

        TrajPoint p;
        p.t = 0.0;
        p.pos = Vec3(xy.x(), xy.y(), headingAtArcLength(startHeading, endHeading, s, L).theta);
        p.vel = Vec3::Zero();
        p.acc = Vec3::Zero();
        path->push_back(p);
    }

    // Land exactly on the requested start and end (table error is tiny but nonzero).
    path->front().pos = Vec3(start.x(), start.y(), startHeading);
    path->back().pos  = Vec3(end.x(), end.y(), unwrapAngle(startHeading, endHeading));
    return path;
}