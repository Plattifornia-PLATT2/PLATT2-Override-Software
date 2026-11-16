// motion_profile.cpp - Step 2: adds t, vel and acc to a path vector from generatePath.
// Build: g++ -std=c++17 -O2 -I /usr/include/eigen3 -c motion_profile.cpp
#include "platt2/robot/subsystems/holonomicDrive/autonControl/trajectoryGen.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kEps = 1e-9;

// Per-point geometry of the path vector, from finite differences along the path.
struct PathGeometry {
    std::vector<Vec2> tangent;     // unit tangent of the xy path
    std::vector<double> kappa;     // signed curvature, 1/in (positive = turning left)
    std::vector<double> dTheta;    // d(theta)/ds, rad/in
    std::vector<double> ddTheta;   // d2(theta)/ds2, rad/in^2
};

// Derivative of y with respect to s at index i (central, one-sided at the ends).
double derivative(const std::vector<double>& y, const std::vector<double>& s, size_t i) {
    const size_t n = y.size();
    const size_t lo = (i == 0) ? 0 : i - 1;
    const size_t hi = (i + 1 >= n) ? n - 1 : i + 1;
    const double ds = s[hi] - s[lo];
    return ds > kEps ? (y[hi] - y[lo]) / ds : 0.0;
}

PathGeometry computeGeometry(const std::vector<TrajPoint>& path,
                             const std::vector<double>& s) {
    const size_t n = path.size();
    PathGeometry g;
    g.tangent.assign(n, Vec2(1.0, 0.0));
    g.kappa.assign(n, 0.0);
    g.dTheta.assign(n, 0.0);
    g.ddTheta.assign(n, 0.0);
    if (n < 2) return g;

    // Tangent direction (unwrapped angle so curvature is a clean derivative).
    std::vector<double> phi(n), theta(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t lo = (i == 0) ? 0 : i - 1;
        const size_t hi = (i + 1 >= n) ? n - 1 : i + 1;
        Vec2 d = path[hi].pos.head<2>() - path[lo].pos.head<2>();
        if (d.norm() < kEps) d = (i > 0) ? Vec2(std::cos(phi[i - 1]), std::sin(phi[i - 1]))
                                         : Vec2(1.0, 0.0);
        phi[i] = std::atan2(d.y(), d.x());
        if (i > 0) phi[i] = unwrapAngle(phi[i - 1], phi[i]);
        g.tangent[i] = Vec2(std::cos(phi[i]), std::sin(phi[i]));
        theta[i] = path[i].pos.z();  // robot heading, set by generatePath
    }
    // Curvature from the x, y path; heading rates from theta along the path.
    for (size_t i = 0; i < n; ++i) {
        g.kappa[i]  = derivative(phi, s, i);
        g.dTheta[i] = derivative(theta, s, i);
    }
    for (size_t i = 0; i < n; ++i) g.ddTheta[i] = derivative(g.dTheta, s, i);
    return g;
}

// Largest acceleration along the path (in/s^2) allowed at speed v at point i:
//   |a_vec| <= maxAccel : a_t^2 + (v^2 kappa)^2 <= maxAccel^2
//   |alpha| <= maxAlpha : alpha = a_t * dTheta + v^2 * ddTheta
// (alpha limit applied conservatively with absolute values).
double availableAccel(const PathGeometry& g, size_t i, double v, const TrajConstraints& lim) {
    const double v2 = v * v;
    const double centripetal = v2 * std::abs(g.kappa[i]);
    double a = std::sqrt(std::max(0.0, lim.maxAccel * lim.maxAccel - centripetal * centripetal));

    const double dth = std::abs(g.dTheta[i]);
    if (dth > kEps) {
        const double aAlpha = (lim.maxAlpha - v2 * std::abs(g.ddTheta[i])) / dth;
        a = std::min(a, std::max(0.0, aAlpha));
    }
    return a;
}

}  // namespace

std::vector<double> cumulativeArcLength(const std::vector<TrajPoint>& path) {
    std::vector<double> s(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i)
        s[i] = s[i - 1] + (path[i].pos.head<2>() - path[i - 1].pos.head<2>()).norm();
    return s;
}

std::vector<double> buildSpeedProfile(const std::vector<TrajPoint>& path,
                                      const std::vector<double>& arcLength,
                                      const TrajConstraints& limits) {
    const size_t n = path.size();
    std::vector<double> v(n, 0.0);
    if (n < 2) return v;

    const PathGeometry g = computeGeometry(path, arcLength);

    // Speed cap at each point from the velocity limits alone.
    std::vector<double> cap(n, limits.maxVel);
    for (size_t i = 0; i < n; ++i) {
        const double k = std::abs(g.kappa[i]);
        if (k > kEps) cap[i] = std::min(cap[i], std::sqrt(limits.maxAccel / k));
        const double dth = std::abs(g.dTheta[i]);
        if (dth > kEps) cap[i] = std::min(cap[i], limits.maxOmega / dth);
        const double ddth = std::abs(g.ddTheta[i]);
        if (ddth > kEps) cap[i] = std::min(cap[i], std::sqrt(limits.maxAlpha / ddth));
    }
    cap.front() = 0.0;  // start and end at rest
    cap.back() = 0.0;

    // Acceleration inside a segment is constant, but speed (and so centripetal and
    // alpha demand) is larger at the far end. Take the smaller of the two ends' limits.
    auto segmentAccel = [&](size_t i, double vNear, size_t farIdx, double vFar) {
        return std::min(availableAccel(g, i, vNear, limits),
                        availableAccel(g, farIdx, vFar, limits));
    };

    // Forward pass: accelerate as hard as allowed.
    v[0] = 0.0;
    for (size_t i = 0; i + 1 < n; ++i) {
        const double ds = arcLength[i + 1] - arcLength[i];
        double vNext = std::min(cap[i + 1], std::sqrt(v[i] * v[i] + 2.0 * availableAccel(g, i, v[i], limits) * ds));
        const double a = segmentAccel(i, v[i], i + 1, vNext);
        v[i + 1] = std::min(cap[i + 1], std::sqrt(v[i] * v[i] + 2.0 * a * ds));
    }
    // Backward pass: make sure we can still stop in time.
    v[n - 1] = 0.0;
    for (size_t i = n - 1; i-- > 0;) {
        const double ds = arcLength[i + 1] - arcLength[i];
        double vPrev = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * availableAccel(g, i + 1, v[i + 1], limits) * ds));
        const double a = segmentAccel(i + 1, v[i + 1], i, vPrev);
        v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0 * a * ds));
    }
    return v;
}

std::vector<double> timeAtPoints(const std::vector<double>& arcLength,
                                 const std::vector<double>& speed) {
    std::vector<double> t(arcLength.size(), 0.0);
    for (size_t i = 0; i + 1 < arcLength.size(); ++i) {
        const double ds = arcLength[i + 1] - arcLength[i];
        const double vsum = speed[i] + speed[i + 1];
        // Constant acceleration across the segment: time = 2 ds / (v0 + v1).
        t[i + 1] = t[i] + (vsum > kEps ? 2.0 * ds / vsum : 0.0);
    }
    return t;
}

TrajPtr applyMotionProfile(const TrajPtr& pathPtr,
                           const TrajConstraints& limits,
                           double dt) {
    auto out = std::make_shared<std::vector<TrajPoint>>();
    if (!pathPtr || pathPtr->empty()) return out;
    dt = std::max(dt, 1e-4);

    const std::vector<TrajPoint>& path = *pathPtr;
    const size_t n = path.size();

    auto restPoint = [&](double t, const Vec3& pos) {
        TrajPoint p;
        p.t = t;
        p.pos = pos;
        p.vel = Vec3::Zero();
        p.acc = Vec3::Zero();
        return p;
    };
    if (n < 2) {
        out->push_back(restPoint(0.0, path.front().pos));
        return out;
    }

    const std::vector<double> s = cumulativeArcLength(path);
    const std::vector<double> v = buildSpeedProfile(path, s, limits);
    const std::vector<double> tk = timeAtPoints(s, v);
    const PathGeometry g = computeGeometry(path, s);
    const double total = tk.back();

    const size_t steps = static_cast<size_t>(std::ceil(total / dt - 1e-9));
    out->reserve(steps + 1);

    size_t j = 0;  // segment index: tk[j] <= t < tk[j+1]
    for (size_t k = 0; k <= steps; ++k) {
        const double t = static_cast<double>(k) * dt;
        if (t >= total - 1e-9) {  // at or past the end: sit at the end pose, at rest
            out->push_back(restPoint(t, path.back().pos));
            continue;
        }
        while (j + 2 < n && tk[j + 1] <= t) ++j;

        const double segLen = std::max(s[j + 1] - s[j], kEps);
        const double segTime = tk[j + 1] - tk[j];
        if (segTime < kEps) {  // zero-time segment, shouldn't happen; skip safely
            out->push_back(restPoint(t, path[j].pos));
            continue;
        }

        // Constant tangential acceleration inside the segment.
        const double aT = (v[j + 1] * v[j + 1] - v[j] * v[j]) / (2.0 * segLen);
        const double local = t - tk[j];
        const double speed = v[j] + aT * local;
        const double travelled = v[j] * local + 0.5 * aT * local * local;
        const double f = std::clamp(travelled / segLen, 0.0, 1.0);

        const Vec2 xy = (1.0 - f) * path[j].pos.head<2>() + f * path[j + 1].pos.head<2>();
        const double theta = (1.0 - f) * path[j].pos.z() + f * path[j + 1].pos.z();
        Vec2 tan = (1.0 - f) * g.tangent[j] + f * g.tangent[j + 1];
        tan = tan.norm() > kEps ? Vec2(tan.normalized()) : g.tangent[j];
        const Vec2 nrm(-tan.y(), tan.x());
        const double kappa  = (1.0 - f) * g.kappa[j]   + f * g.kappa[j + 1];
        const double dth   = (1.0 - f) * g.dTheta[j]  + f * g.dTheta[j + 1];
        const double ddth  = (1.0 - f) * g.ddTheta[j] + f * g.ddTheta[j + 1];

        TrajPoint p;
        p.t = t;
        p.pos = Vec3(xy.x(), xy.y(), theta);
        p.vel = Vec3(speed * tan.x(), speed * tan.y(), speed * dth);
        const Vec2 aXY = aT * tan + speed * speed * kappa * nrm;
        p.acc = Vec3(aXY.x(), aXY.y(), aT * dth + speed * speed * ddth);
        out->push_back(p);
    }
    return out;
}