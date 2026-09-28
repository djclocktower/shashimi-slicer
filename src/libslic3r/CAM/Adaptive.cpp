// SPDX-License-Identifier: LGPL-2.1-or-later

/**************************************************************************
 *   Copyright (c) 2018 Kresimir Tusek <kresimir.tusek@gmail.com>          *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

// Ported to Shashimi Slicer from FreeCAD src/Mod/CAM/libarea/Adaptive.cpp (2026).
// Changes from the original:
//  - ClipperLib 6.4.2 (use_xyz) -> libslic3r's ClipperLib_Z (Eigen points: .x()/.y()/.z();
//    note Eigen's == also compares z, so xy comparisons use SameXY()); FreeCAD's Clipper2
//    calls -> the vendored Clipper2Lib_Z.
//  - Python/progress-preview, DEV_MODE, SVG debugging, perf counters and console output removed.
//    Progress/cancel through Params::cancel.
//  - Profiling op types dropped; clearing-inside/outside are expressed by the API
//    (region / keep_out / outside_is_air). Input de-duplication/wire joining dropped (the input
//    is always clean closed rings from ExPolygons). Stock-to-leave is applied at the API
//    boundary (round joins, like the original ApplyStockToLeave).
//  - The wall-clock time limit in ResolveLinkPath was removed so output is deterministic (the
//    10000-iteration limit remains).
//  - Paths stay in integer algorithm units until the API boundary (the original round-tripped
//    through doubles in mm).
//  - Fix: CalcCutArea treated each cleared polygon separately, so the inside of a hole counted as
//    cleared. Outside clearing (cleared = a ring around the stock) then saw no material at all and
//    crawled along the stock edge for ~10000 failed engagements (~100x slower, half left uncut).
//  - Conventional milling: the algorithm only produces climb cuts (CW spindle, material on the
//    right of travel); conventional is produced by mirroring the input in X and mirroring back.

#include "clipper/clipper_z.hpp"
#include <clipper2/clipper2_z.hpp>

#include "Adaptive.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <map>
#include <optional>
#include <tuple>

namespace {
namespace AP {

using namespace ClipperLib_Z;
// Slic3r::ClipperLib is visible here too (via the precompiled header); pin the Z variant.
using ClipperLib_Z::cInt;
using ClipperLib_Z::Clipper;
// ClipperLib_Z::ClipperOffset's constructor leaves m_lowest uninitialized (m_lowest(-1, 0) on an
// Eigen 3-vector); Clear() initializes it.
// ArcTolerance: see Adaptive2d::arcTolerance.
struct ClipperOffset : ClipperLib_Z::ClipperOffset
{
    explicit ClipperOffset(double arcTolerance)
    {
        Clear();
        ArcTolerance = arcTolerance;
    }
};
using ClipperLib_Z::DoublePoint;
using ClipperLib_Z::IntPoint;
using ClipperLib_Z::IntRect;
using ClipperLib_Z::Path;
using ClipperLib_Z::Paths;
using ClipperLib_Z::PolyTree;
using ClipperLib_Z::ctDifference;
using ClipperLib_Z::ctIntersection;
using ClipperLib_Z::ctUnion;
using ClipperLib_Z::etClosedPolygon;
using ClipperLib_Z::etOpenRound;
using ClipperLib_Z::jtRound;
using ClipperLib_Z::jtSquare;
using ClipperLib_Z::pftEvenOdd;
using ClipperLib_Z::pftNonZero;
using ClipperLib_Z::ptClip;
using ClipperLib_Z::ptSubject;

constexpr double NTOL                      = 1.0e-7;
constexpr double SAME_POINT_TOL_SQRD_SCALED = 4.0;
constexpr double PI                         = 3.14159265358979323846;

enum MotionType { mtCutting = 0, mtLinkClear = 1, mtLinkNotClear = 2 };

using TPath  = std::pair<MotionType, Path>;
using TPaths = std::vector<TPath>;

struct AdaptiveOutput
{
    IntPoint   HelixCenterPoint{0, 0, 0};
    IntPoint   StartPoint{0, 0, 0};
    TPaths     AdaptivePaths;
    MotionType ReturnMotionType = mtCutting;
    Paths      Cleared; // cleared area at the end of this region (includes the initial cleared area)

    bool StartPointNotFound         = false;
    bool UnclearedAreaRemains       = false;
    bool FailedToSetUpFinishingPass = false;
    bool FinishingLeadInFailed      = false;
};

//*****************************************
// Utils - inline
//*****************************************

inline IntPoint IP(double x, double y) { return IntPoint(cInt(x), cInt(y), 0); }

inline bool SameXY(const IntPoint& a, const IntPoint& b) { return a.x() == b.x() && a.y() == b.y(); }

inline double DistanceSqrd(const IntPoint& pt1, const IntPoint& pt2)
{
    double Dx = double(pt1.x() - pt2.x());
    double dy = double(pt1.y() - pt2.y());
    return (Dx * Dx + dy * dy);
}

inline double averageDV(const std::vector<double>& vec)
{
    if (vec.empty())
        return 0;
    double s = 0;
    for (double v : vec)
        s += v;
    return s / double(vec.size());
}

inline DoublePoint rotate(const DoublePoint& in, double rad)
{
    double c = cos(rad);
    double s = sin(rad);
    return DoublePoint(c * in.x() - s * in.y(), s * in.x() + c * in.y());
}

// calculates path length for open path
inline double PathLength(const Path& path)
{
    double len = 0;
    for (size_t i = 1; i < path.size(); i++)
        len += sqrt(DistanceSqrd(path[i - 1], path[i]));
    return len;
}

inline DoublePoint DirectionV(const IntPoint& pt1, const IntPoint& pt2)
{
    double DX = double(pt2.x() - pt1.x());
    double DY = double(pt2.y() - pt1.y());
    double l  = sqrt(DX * DX + DY * DY);
    if (l < NTOL)
        return DoublePoint(0, 0);
    return DoublePoint(DX / l, DY / l);
}

inline void NormalizeV(DoublePoint& pt)
{
    double len = sqrt(pt.x() * pt.x() + pt.y() * pt.y());
    if (len > NTOL) {
        pt.x() /= len;
        pt.y() /= len;
    }
}

inline DoublePoint GetPathDirectionV(const Path& pth, size_t pointIndex)
{
    if (pth.size() < 2)
        return DoublePoint(0, 0);
    const IntPoint& p1 = pth.at(pointIndex > 0 ? pointIndex - 1 : pth.size() - 1);
    const IntPoint& p2 = pth.at(pointIndex);
    return DirectionV(p1, p2);
}

// Returns true if points 'a' and 'b' are coincident or nearly so.
bool isClose(const IntPoint& a, const IntPoint& b) { return std::abs(a.x() - b.x()) <= 1 && std::abs(a.y() - b.y()) <= 1; }

// Remove coincident and almost-coincident points from Paths.
void filterCloseValues(Paths& ppg)
{
    for (auto& pth : ppg) {
        while (true) {
            auto i = std::adjacent_find(pth.begin(), pth.end(), isClose);
            if (i == pth.end())
                break;
            pth.erase(i);
        }
        while (pth.size() > 1 && isClose(pth.front(), pth.back()))
            pth.pop_back();
    }
}

void TranslatePath(const Path& input, Path& output, const IntPoint& delta)
{
    output.clear();
    output.reserve(input.size());
    for (const IntPoint& p : input)
        output.emplace_back(p.x() + delta.x(), p.y() + delta.y(), p.z());
}

//*****************************************
// Utils
//*****************************************

class BoundBox
{
public:
    BoundBox() = default;
    explicit BoundBox(const IntPoint& p1) { SetFirstPoint(p1); }
    void SetFirstPoint(const IntPoint& p1)
    {
        minX = maxX = p1.x();
        minY = maxY = p1.y();
    }
    void AddPoint(const IntPoint& pt)
    {
        minX = std::min(pt.x(), minX);
        maxX = std::max(pt.x(), maxX);
        minY = std::min(pt.y(), minY);
        maxY = std::max(pt.y(), maxY);
    }
    // line segment: two points
    BoundBox(const IntPoint& p1, const IntPoint& p2)
    {
        minX = std::min(p1.x(), p2.x());
        maxX = std::max(p1.x(), p2.x());
        minY = std::min(p1.y(), p2.y());
        maxY = std::max(p1.y(), p2.y());
    }
    // for circle: center and radius
    BoundBox(const IntPoint& center, cInt radius)
    {
        minX = center.x() - radius;
        maxX = center.x() + radius;
        minY = center.y() - radius;
        maxY = center.y() + radius;
    }
    bool CollidesWith(const BoundBox& bb2) const
    {
        return minX <= bb2.maxX && maxX >= bb2.minX && minY <= bb2.maxY && maxY >= bb2.minY;
    }
    bool Contains(const BoundBox& bb2) const
    {
        return minX <= bb2.minX && maxX >= bb2.maxX && minY <= bb2.minY && maxY >= bb2.maxY;
    }

    cInt minX = 0, maxX = 0, minY = 0, maxY = 0;
};

int getPathNestingLevel(const Path& path, const Paths& paths)
{
    int nesting = 0;
    for (const auto& other : paths)
        if (!path.empty() && PointInPolygon(path.front(), other) != 0)
            nesting++;
    return nesting;
}

void AverageDirection(const std::vector<DoublePoint>& unityVectors, DoublePoint& output)
{
    output = DoublePoint(0, 0);
    for (const DoublePoint& v : unityVectors)
        output += v;
    double magnitude = output.norm();
    output /= magnitude;
}

double DistancePointToLineSegSquared(const IntPoint& p1, const IntPoint& p2, const IntPoint& pt, IntPoint& closestPoint,
                                     double& ptParameter, bool clamp = true)
{
    double D21X        = double(p2.x() - p1.x());
    double D21Y        = double(p2.y() - p1.y());
    double DP1X        = double(pt.x() - p1.x());
    double DP1Y        = double(pt.y() - p1.y());
    double lsegLenSqr  = D21X * D21X + D21Y * D21Y;
    if (lsegLenSqr == 0) { // segment is zero length, return point to point distance
        closestPoint = p1;
        ptParameter  = 0;
        return DP1X * DP1X + DP1Y * DP1Y;
    }
    double parameter = DP1X * D21X + DP1Y * D21Y;
    if (clamp)
        parameter = std::clamp(parameter, 0., lsegLenSqr);
    ptParameter  = parameter / lsegLenSqr;
    closestPoint = IP(p1.x() + ptParameter * D21X, p1.y() + ptParameter * D21Y);
    double DX    = double(pt.x() - closestPoint.x());
    double DY    = double(pt.y() - closestPoint.y());
    return DX * DX + DY * DY;
}

void ScaleUpPaths(Paths& paths, cInt scaleFactor)
{
    for (auto& pth : paths)
        for (auto& pt : pth) {
            pt.x() *= scaleFactor;
            pt.y() *= scaleFactor;
        }
}

void ScaleDownPaths(Paths& paths, cInt scaleFactor)
{
    for (auto& pth : paths)
        for (auto& pt : pth) {
            pt.x() /= scaleFactor;
            pt.y() /= scaleFactor;
        }
}

double DistancePointToPathsSqrd(const Paths& paths, const IntPoint& pt, IntPoint& closestPointOnPath, size_t& clpPathIndex,
                                size_t& clpSegmentIndex, double& clpParameter)
{
    double   minDistSq = DBL_MAX;
    IntPoint clp;
    for (size_t i = 0; i < paths.size(); i++) {
        const Path& path = paths[i];
        size_t      size = path.size();
        for (size_t j = 0; j < size; j++) {
            double ptPar;
            double distSq = DistancePointToLineSegSquared(path[j > 0 ? j - 1 : size - 1], path[j], pt, clp, ptPar);
            if (distSq < minDistSq) {
                clpPathIndex       = i;
                clpSegmentIndex    = j;
                clpParameter       = ptPar;
                closestPointOnPath = clp;
                minDistSq          = distSq;
            }
        }
    }
    return minDistSq;
}

// joins collinear segments (within the tolerance)
void CleanPath(const Path& inp, Path& outpt, double tolerance)
{
    if (inp.size() < 3) {
        outpt = inp;
        return;
    }
    outpt.clear();
    Path tmp;
    CleanPolygon(inp, tmp, tolerance);
    long size = long(tmp.size());

    // CleanPolygon will have empty result if all points are collinear,
    // need to add first and last point to the output
    if (size <= 2) {
        outpt.push_back(inp.front());
        outpt.push_back(inp.back());
        return;
    }

    // restore starting point
    double   clpPar          = 0;
    size_t   clpSegmentIndex = 0;
    size_t   clpPathIndex    = 0;
    Paths    tmpPaths{tmp};
    IntPoint clp;
    // find point on cleaned poly that is closest to original starting point
    DistancePointToPathsSqrd(tmpPaths, inp.front(), clp, clpPathIndex, clpSegmentIndex, clpPar);

    // if closes point is not one of the polygon points, add it as separate first point
    if (DistanceSqrd(clp, tmp.at(clpSegmentIndex)) > 0 &&
        DistanceSqrd(clp, tmp.at(clpSegmentIndex > 0 ? clpSegmentIndex - 1 : size - 1)) > 0)
        outpt.push_back(clp);

    // add remaining points starting from closest
    for (long i = 0; i < size; i++) {
        long index = long(clpSegmentIndex) + i;
        if (index >= size)
            index -= size;
        outpt.push_back(tmp.at(index));
    }

    if (DistanceSqrd(outpt.front(), inp.front()) > SAME_POINT_TOL_SQRD_SCALED)
        outpt.insert(outpt.begin(), inp.front());

    if (DistanceSqrd(outpt.back(), inp.back()) > SAME_POINT_TOL_SQRD_SCALED)
        outpt.push_back(inp.back());
}

bool Circle2CircleIntersect(const DoublePoint& c1, const DoublePoint& c2, double radius, std::pair<DoublePoint, DoublePoint>& intersections)
{
    double DX = c2.x() - c1.x();
    double DY = c2.y() - c1.y();
    double d  = sqrt(DX * DX + DY * DY);
    if (d < NTOL)
        return false; // same center
    if (d >= radius)
        return false; // do not intersect, or intersect in one point (this case not relevant here)
    double a_2 = sqrt(4 * radius * radius - d * d) / 2.0;
    intersections.first  = DoublePoint(0.5 * (c1.x() + c2.x()) - DY * a_2 / d, 0.5 * (c1.y() + c2.y()) + DX * a_2 / d);
    intersections.second = DoublePoint(0.5 * (c1.x() + c2.x()) + DY * a_2 / d, 0.5 * (c1.y() + c2.y()) - DX * a_2 / d);
    return true;
}

bool Line2CircleIntersect(const DoublePoint& c, double radius, const DoublePoint& p1, const DoublePoint& p2,
                          std::vector<DoublePoint>& result, bool clamp = true)
{
    // if more intersections returned, first is closer to p1
    double dx  = p2.x() - p1.x();
    double dy  = p2.y() - p1.y();
    double lcx = p1.x() - c.x();
    double lcy = p1.y() - c.y();
    double a   = dx * dx + dy * dy;
    double b   = 2 * dx * lcx + 2 * dy * lcy;
    double C   = lcx * lcx + lcy * lcy - radius * radius;
    double sq  = b * b - 4 * a * C;
    if (sq < 0)
        return false; // no solution
    sq        = sqrt(sq);
    double t1 = (-b - sq) / (2 * a);
    double t2 = (-b + sq) / (2 * a);
    result.clear();
    if ((t1 >= 0.0 && t1 <= 1.0) || !clamp)
        result.emplace_back(p1.x() + t1 * dx, p1.y() + t1 * dy);
    if ((t2 >= 0.0 && t2 <= 1.0) || !clamp)
        result.emplace_back(p1.x() + t2 * dx, p1.y() + t2 * dy);
    return !result.empty();
}

// calculate center point of polygon
IntPoint Compute2DPolygonCentroid(const Path& vertices)
{
    double cx = 0, cy = 0, signedArea = 0.0;
    size_t size = vertices.size();
    for (size_t i = 0; i < size; ++i) {
        double x0 = double(vertices[i].x());
        double y0 = double(vertices[i].y());
        double x1 = double(vertices[(i + 1) % size].x());
        double y1 = double(vertices[(i + 1) % size].y());
        double a  = x0 * y1 - x1 * y0;
        signedArea += a;
        cx += (x0 + x1) * a;
        cy += (y0 + y1) * a;
    }
    signedArea *= 0.5;
    return IP(cx / (6.0 * signedArea), cy / (6.0 * signedArea));
}

// point must be within first path (boundary) and must not be within all other paths (holes)
bool IsPointWithinCutRegion(const Paths& toolBoundPaths, const IntPoint& point)
{
    bool inside = false;
    for (const Path& p : toolBoundPaths)
        if (PointInPolygon(point, p) != 0)
            inside = !inside;
    return inside;
}

/* finds intersection of line segment with line segment */
bool IntersectionPoint(const IntPoint& s1p1, const IntPoint& s1p2, const IntPoint& s2p1, const IntPoint& s2p2, IntPoint& intersection)
{
    double S1DX = double(s1p2.x() - s1p1.x());
    double S1DY = double(s1p2.y() - s1p1.y());
    double S2DX = double(s2p2.x() - s2p1.x());
    double S2DY = double(s2p2.y() - s2p1.y());
    double d    = S1DY * S2DX - S2DY * S1DX;
    if (fabs(d) < NTOL)
        return false; // lines are parallel

    double LPDX = double(s1p1.x() - s2p1.x());
    double LPDY = double(s1p1.y() - s2p1.y());
    double p1d  = S2DY * LPDX - S2DX * LPDY;
    double p2d  = S1DY * LPDX - S1DX * LPDY;
    if ((d < 0) && (p1d < d || p1d > 0 || p2d < d || p2d > 0))
        return false; // intersection not within segment1
    if ((d > 0) && (p1d < 0 || p1d > d || p2d < 0 || p2d > d))
        return false; // intersection not within segment2
    double t     = p1d / d;
    intersection = IP(s1p1.x() + S1DX * t, s1p1.y() + S1DY * t);
    return true;
}

double SegmentsDistanceSqrd(const IntPoint& a1, const IntPoint& a2, const IntPoint& b1, const IntPoint& b2)
{
    IntPoint tmp;
    double   par;
    if (IntersectionPoint(a1, a2, b1, b2, tmp))
        return 0;
    return std::min(std::min(DistancePointToLineSegSquared(a1, a2, b1, tmp, par), DistancePointToLineSegSquared(a1, a2, b2, tmp, par)),
                    std::min(DistancePointToLineSegSquared(b1, b2, a1, tmp, par), DistancePointToLineSegSquared(b1, b2, a2, tmp, par)));
}

// True if `path` swept by a disk of radius `dist` lies inside `area` (even-odd paths): the path
// starts inside and stays at least `dist` from the area boundary. Equivalent to "offset(path,
// dist) - area is empty" (and to "point in offset(area, -dist)" for a single point), without
// running a boolean over the whole area - O(edges) with a bounding box reject.
bool SweptInside(const Paths& area, const Path& path, double dist)
{
    if (path.empty() || !IsPointWithinCutRegion(area, path.front()))
        return false;
    BoundBox pbb(path.front());
    for (const IntPoint& p : path)
        pbb.AddPoint(p);
    const cInt d = cInt(std::ceil(dist)) + 1;
    pbb.minX -= d;
    pbb.minY -= d;
    pbb.maxX += d;
    pbb.maxY += d;
    const double d2 = dist * dist;
    for (const Path& poly : area) {
        for (size_t j = 0; j < poly.size(); ++j) {
            const IntPoint& e1 = poly[j > 0 ? j - 1 : poly.size() - 1];
            const IntPoint& e2 = poly[j];
            if (!BoundBox(e1, e2).CollidesWith(pbb))
                continue;
            if (path.size() == 1) {
                IntPoint tmp;
                double   par;
                if (DistancePointToLineSegSquared(e1, e2, path.front(), tmp, par) < d2)
                    return false;
            } else
                for (size_t i = 1; i < path.size(); ++i)
                    if (SegmentsDistanceSqrd(path[i - 1], path[i], e1, e2) < d2)
                        return false;
        }
    }
    return true;
}

void SmoothPaths(Paths& paths, double stepSize, long pointCount, long iterations)
{
    Paths output;
    output.resize(paths.size());
    const long   scale      = 1000;
    const double stepScaled = stepSize * scale;

    ScaleUpPaths(paths, scale);
    std::vector<std::pair<size_t /*path index*/, IntPoint>> points;
    for (size_t i = 0; i < paths.size(); i++) {
        for (const auto& pt : paths[i]) {
            if (points.empty()) {
                points.emplace_back(i, pt);
                continue;
            }
            const auto      back   = points.back();
            const IntPoint& lastPt = back.second;

            const double l = sqrt(DistanceSqrd(lastPt, pt));

            if (l < 0.5 * stepScaled) {
                if (points.size() > 1)
                    points.pop_back();
                points.emplace_back(i, pt);
                continue;
            }
            size_t     lastPathIndex = back.first;
            const long steps         = std::max(long(l / stepScaled), 1L);
            const long left          = pointCount * iterations * 2;
            const long right         = steps - pointCount * iterations * 2;
            for (long idx = 0; idx <= steps; idx++) {
                if (idx > left && idx < right) {
                    idx = right;
                    continue;
                }
                const double   p = double(idx) / steps;
                const IntPoint ptx = IP(lastPt.x() + double(pt.x() - lastPt.x()) * p, lastPt.y() + double(pt.y() - lastPt.y()) * p);

                if (idx == 0 && DistanceSqrd(back.second, ptx) < scale && points.size() > 1)
                    points.pop_back();

                points.emplace_back(p < 0.5 ? lastPathIndex : i, ptx);
            }
        }
    }
    if (points.empty())
        return;
    const long size = long(points.size());
    for (long iter = 0; iter < iterations; iter++) {
        for (long i = 1; i < size - 1; i++) {
            IntPoint& cp       = points[i].second;
            IntPoint  avgPoint = cp;
            long      cnt      = 1;

            long ptsToAverage = pointCount;
            if (i <= ptsToAverage)
                ptsToAverage = std::max(i - 1, 0L);
            else if (i + ptsToAverage >= size - 1)
                ptsToAverage = size - 1 - i;
            for (long j = i - ptsToAverage; j <= i + ptsToAverage; j++) {
                if (j == i)
                    continue;
                long index = std::clamp(j, 0L, size - 1);
                const IntPoint& p = points[index].second;
                avgPoint.x() += p.x();
                avgPoint.y() += p.y();
                cnt++;
            }
            cp.x() = avgPoint.x() / cnt;
            cp.y() = avgPoint.y() / cnt;
        }
    }

    for (const auto& pr : points)
        output[pr.first].push_back(pr.second);
    for (size_t i = 0; i < paths.size(); i++)
        CleanPath(output[i], paths[i], 1.4 * scale);
    ScaleDownPaths(paths, scale);
}

// PopNextFinishingPass: select and remove the next finishing pass to execute (closest to p1).
// Closed paths may start at any vertex (rotated so that execution starts at the closest point,
// advanced by extraDistanceAround); open paths must start at their first vertex.
// Returns true if a closed path was selected, false for an open one.
bool PopNextFinishingPass(Paths& closedFinishingPaths, Paths& openFinishingPaths, IntPoint p1, Path& result,
                          double extraDistanceAround = 0)
{
    if (closedFinishingPaths.empty() && openFinishingPaths.empty()) {
        result.clear();
        return false;
    }

    double minDistSqrd       = DBL_MAX;
    size_t closestPathIndex  = 0;
    size_t closestPointIndex = 0;
    bool   closestIsClosed   = !closedFinishingPaths.empty();

    for (size_t pathIndex = 0; pathIndex < closedFinishingPaths.size(); pathIndex++) {
        const Path& path = closedFinishingPaths[pathIndex];
        for (size_t i = 0; i < path.size(); i++) {
            double dist = DistanceSqrd(p1, path[i]);
            if (dist < minDistSqrd) {
                minDistSqrd       = dist;
                closestPathIndex  = pathIndex;
                closestPointIndex = i;
                closestIsClosed   = true;
            }
        }
    }

    for (size_t pathIndex = 0; pathIndex < openFinishingPaths.size(); pathIndex++) {
        const Path& path = openFinishingPaths[pathIndex];
        if (!path.empty()) {
            double dist = DistanceSqrd(p1, path[0]);
            if (dist < minDistSqrd) {
                minDistSqrd       = dist;
                closestPathIndex  = pathIndex;
                closestPointIndex = 0;
                closestIsClosed   = false;
            }
        }
    }

    if (!closestIsClosed) {
        result = openFinishingPaths[closestPathIndex];
        openFinishingPaths.erase(openFinishingPaths.begin() + closestPathIndex);
        return false;
    }

    Path closestPath = closedFinishingPaths[closestPathIndex];
    closedFinishingPaths.erase(closedFinishingPaths.begin() + closestPathIndex);
    result.clear();

    // Apply extraDistanceAround to advance further along the path before starting
    while (extraDistanceAround > 0) {
        size_t          nexti      = (closestPointIndex + 1) % closestPath.size();
        const IntPoint& a          = closestPath[closestPointIndex];
        const IntPoint& b          = closestPath[nexti];
        double          distToNext = sqrt(DistanceSqrd(a, b));
        closestPointIndex          = nexti;

        if (distToNext <= extraDistanceAround) {
            extraDistanceAround -= distToNext;
        } else {
            double interp = extraDistanceAround / distToNext;
            result.emplace_back(cInt(a.x() * (1 - interp) + b.x() * interp), cInt(a.y() * (1 - interp) + b.y() * interp), a.z());
            extraDistanceAround = 0;
        }
    }

    for (size_t offset = 0; offset < closestPath.size(); offset++)
        result.push_back(closestPath[(closestPointIndex + offset) % closestPath.size()]);

    return true;
}

void ConnectPaths(Paths input, Paths& output)
{
    output.clear();
    bool newPath = true;
    Path joined;
    while (!input.empty()) {
        if (newPath) {
            if (!joined.empty())
                output.push_back(joined);
            joined = input.front();
            input.erase(input.begin());
            newPath = false;
        }
        bool anyMatch = false;
        for (size_t i = 0; i < input.size(); i++) {
            Path& n = input[i];
            if (DistanceSqrd(n.front(), joined.back()) < SAME_POINT_TOL_SQRD_SCALED) {
                joined.insert(joined.end(), n.begin(), n.end());
            } else if (DistanceSqrd(n.back(), joined.back()) < SAME_POINT_TOL_SQRD_SCALED) {
                ReversePath(n);
                joined.insert(joined.end(), n.begin(), n.end());
            } else if (DistanceSqrd(n.front(), joined.front()) < SAME_POINT_TOL_SQRD_SCALED) {
                for (const auto& pt : n)
                    joined.insert(joined.begin(), pt);
            } else if (DistanceSqrd(n.back(), joined.front()) < SAME_POINT_TOL_SQRD_SCALED) {
                ReversePath(n);
                for (const auto& pt : n)
                    joined.insert(joined.begin(), pt);
            } else
                continue;
            input.erase(input.begin() + i);
            anyMatch = true;
            break;
        }
        if (!anyMatch)
            newPath = true;
    }
    if (!joined.empty())
        output.push_back(joined);
}

Clipper2Lib_Z::Paths64 ToC2(const Paths& paths)
{
    Clipper2Lib_Z::Paths64 out;
    out.reserve(paths.size());
    for (const Path& path : paths) {
        Clipper2Lib_Z::Path64 p;
        p.reserve(path.size());
        for (const IntPoint& pt : path)
            p.emplace_back(pt.x(), pt.y(), pt.z());
        out.push_back(std::move(p));
    }
    return out;
}

Paths FromC2(const Clipper2Lib_Z::Paths64& paths)
{
    Paths out;
    out.reserve(paths.size());
    for (const auto& path : paths) {
        Path p;
        p.reserve(path.size());
        for (const auto& pt : path)
            p.emplace_back(pt.x, pt.y, pt.z);
        out.push_back(std::move(p));
    }
    return out;
}

//***********************************
// Cleared area bounding support
//***********************************
class ClearedArea
{
public:
    ClearedArea(cInt p_toolRadiusScaled, double arcTolerance) : clipof(arcTolerance), toolRadiusScaled(p_toolRadiusScaled) {}

    void SetClearedPaths(const Paths& paths)
    {
        clearedPaths = paths;
        pending.clear();
        focusInvalid = windowInvalid = true;
    }

    void AddClearedPaths(const Paths& paths)
    {
        GetCleared(); // flush pending covers first (uses `clip`)
        clip.Clear();
        clip.AddPaths(clearedPaths, ptSubject, true);
        clip.AddPaths(paths, ptClip, true);
        clip.Execute(ctUnion, clearedPaths);
        CleanPolygons(clearedPaths);
        focusInvalid = windowInvalid = true;
    }

    // Adds the area swept by the tool along the path. The union with the whole cleared area is
    // deferred until GetCleared() (several covers are merged in one boolean); the local window
    // used by cut-area queries is updated right away.
    void ExpandCleared(const Path& toClearToolPath)
    {
        if (toClearToolPath.empty())
            return;
        clipof.Clear();
        clipof.AddPath(toClearToolPath, jtRound, etOpenRound);
        Paths toolCoverPoly;
        clipof.Execute(toolCoverPoly, double(toolRadiusScaled + 1));
        if (!windowInvalid) {
            // (cleared + cover) & window == (cleared & window) + (cover & window); cover parts
            // outside the window are harmless, queries clip to their own box
            clip.Clear();
            clip.AddPaths(clearedBoundedWindow, ptSubject, true);
            clip.AddPaths(toolCoverPoly, ptClip, true);
            clip.Execute(ctUnion, clearedBoundedWindow);
        }
        pending.insert(pending.end(), toolCoverPoly.begin(), toolCoverPoly.end());
        focusInvalid = true;
    }

    // get cleared area/poly bounded to toolbox
    // (The original clipped the whole cleared area with Clipper booleans; here a small window
    // around the tool is kept up to date and queries are cut from it with Clipper2's linear-time
    // RectClip - this runs several times on every adaptive step.)
    const Paths& GetBoundedClearedAreaClipped(const IntPoint& toolPos, cInt delta)
    {
        // first, attempt to serve this query from cache
        BoundBox toolBB(toolPos, delta);
        if (!focusInvalid && clearedBBClippedInFocus.Contains(toolBB))
            return clearedBoundedClipped;

        // second, check if the window needs to be recomputed
        if (windowInvalid || !clearedBBWindow.Contains(toolBB)) {
            clearedBBWindow      = BoundBox(toolPos, delta * clearedBoundedWindowScale);
            clearedBoundedWindow = FromC2(Clipper2Lib_Z::RectClip(rect64(clearedBBWindow), ToC2(GetCleared())));
            windowInvalid        = false;
        }

        // finally, perform the query using data from the window
        clearedBBClippedInFocus = toolBB;
        clearedBoundedClipped   = FromC2(Clipper2Lib_Z::RectClip(rect64(toolBB), ToC2(clearedBoundedWindow)));
        focusInvalid            = false;
        return clearedBoundedClipped;
    }

    const Paths& GetCleared()
    {
        if (!pending.empty()) {
            clip.Clear();
            clip.AddPaths(clearedPaths, ptSubject, true);
            clip.AddPaths(pending, ptClip, true);
            clip.Execute(ctUnion, clearedPaths, pftNonZero);
            CleanPolygons(clearedPaths);
            pending.clear();
        }
        return clearedPaths;
    }

private:
    static Clipper2Lib_Z::Rect64 rect64(const BoundBox& bb) { return {bb.minX, bb.minY, bb.maxX, bb.maxY}; }

    Clipper       clip;
    ClipperOffset clipof;
    Paths         clearedPaths;
    Paths         pending; // tool covers not yet merged into clearedPaths
    Paths         clearedBoundedWindow;
    Paths         clearedBoundedClipped;

    cInt     toolRadiusScaled;
    BoundBox clearedBBWindow;
    BoundBox clearedBBClippedInFocus;

    bool focusInvalid              = true;
    bool windowInvalid             = true;
    int  clearedBoundedWindowScale = 3; // original: 10 (a window that large is the whole part)
};

//***************************************
// Linear Interpolation - area vs angle
//***************************************
struct InterpItem
{
    std::pair<double, IntPoint> angle;
    double                      error;
    bool                        isConventional;
};

class Interpolation
{
public:
    static constexpr double MIN_ANGLE = -PI / 4;
    static constexpr double MAX_ANGLE = PI / 4;

    void clear()
    {
        m_min.reset();
        m_max.reset();
    }
    bool bothSides() const
    {
        return m_min && m_max && m_min->error < 0 && m_max->error >= 0 && (!m_min->isConventional || !m_max->isConventional);
    }
    // adds point keeping the incremental order of areas for interpolation to work correctly
    void addPoint(double error, std::pair<double, IntPoint> angle, bool allowSkip, bool isConventional)
    {
        const InterpItem newItem = {angle, error, isConventional};

        if (!m_min) {
            m_min = newItem;
        } else if (!m_max) {
            m_max = newItem;
            if (m_min->error > m_max->error)
                std::swap(m_min, m_max);
        } else if (isConventional && (m_min->isConventional ^ m_max->isConventional)) {
            if (!allowSkip) {
                if (m_min->isConventional)
                    m_min.reset();
                else
                    m_max.reset();
                addPoint(error, angle, false, isConventional);
            }
        } else if (bothSides()) {
            if (error < 0)
                m_min = newItem;
            else
                m_max = newItem;
        } else {
            if (allowSkip && std::abs(error) > std::abs(m_min->error) && std::abs(error) > std::abs(m_max->error) &&
                (isConventional || !m_min->isConventional || !m_max->isConventional))
                return;

            if (m_min->isConventional ^ m_max->isConventional) {
                if (m_min->isConventional)
                    m_min.reset();
                else
                    m_max.reset();
            } else if (std::abs(m_min->error) > std::abs(m_max->error)) {
                m_min.reset();
            } else {
                m_max.reset();
            }
            addPoint(error, angle, false, isConventional);
        }
    }

    double interpolateAngle() const
    {
        if (!m_min)
            return MIN_ANGLE;
        if (!m_max)
            return MAX_ANGLE;
        double p = (0 - m_min->error) / (m_max->error - m_min->error);
        // compromise between binary search (p = 0.5) and following linear interpolation completely
        const double minInterp = .2;
        p = std::max(std::min(p, 1 - minInterp), minInterp);
        return m_min->angle.first * (1 - p) + m_max->angle.first * p;
    }

    static double clampAngle(double angle) { return std::max(std::min(angle, MAX_ANGLE), MIN_ANGLE); }

    std::optional<InterpItem> m_min;
    std::optional<InterpItem> m_max;
};

// performs the intersection of the path (subject) and the area (obj), preserving
// orientation and (closed-path) connectivity
Paths PathIntersectArea(Clipper& clip, Path& subject, Paths& obj, bool isClosed = true)
{
    if (isClosed)
        subject.push_back(subject[0]); // close path explicitly before treating it as open

    // init z-data: p[i].z = 2 * i + 1, and new points are the average of their neighbors
    // this ensures new points have unique z but come between the points they're made from
    for (size_t i = 0; i < subject.size(); i++)
        subject[i].z() = cInt(i * 2 + 1);
    for (Path& path : obj)
        for (IntPoint& p : path)
            p.z() = 0;
    clip.ZFillFunction([](const IntPoint& e1b, const IntPoint& e1t, const IntPoint& e2b, const IntPoint& e2t, IntPoint& p) {
        if (e1b.z() != 0 && e1t.z() != 0)
            p.z() = (e1b.z() + e1t.z()) / 2;
        else if (e2b.z() != 0 && e2t.z() != 0)
            p.z() = (e2b.z() + e2t.z()) / 2;
    });

    PolyTree diffTree;
    Paths    diff;
    clip.Clear();
    clip.AddPath(subject, ptSubject, false);
    clip.AddPaths(obj, ptClip, true);
    clip.Execute(ctIntersection, diffTree);
    clip.ZFillFunction(nullptr);
    OpenPathsFromPolyTree(diffTree, diff);

    // restore orientation
    for (Path& p : diff) {
        for (size_t i = 0; i + 1 < p.size(); i++) {
            if (p[i].z() != 0 && p[i + 1].z() != 0) {
                if (p[i].z() + 1 != p[i + 1].z() && p[i].z() + 2 != p[i + 1].z())
                    ReversePath(p);
                break;
            }
        }
    }

    // collect result, joining any path that goes through the end point
    const cInt          zstart = 1;
    const cInt          zend   = cInt(subject.size() * 2 - 1);
    std::optional<Path> start, end;
    Paths               result;
    for (Path& p : diff) {
        if (p.empty())
            continue;
        if (p[0].z() == zstart)
            start = p;
        else if (p.back().z() == zend)
            end = p;
        else
            result.push_back(p);
    }
    if (start && end) {
        Path joined = *end;
        // append points from start, skipping the first, which is a repeat
        joined.insert(joined.end(), start->begin() + 1, start->end());
        result.push_back(joined);
    } else {
        if (start)
            result.push_back(*start);
        if (end)
            result.push_back(*end);
    }

    return result;
}

double NestedArea(const Paths& paths)
{
    double a = 0;
    for (const Path& p : paths)
        a += (getPathNestingLevel(p, paths) % 2 == 1 ? 1 : -1) * fabs(Area(p));
    return a;
}

struct IterateNextStepOutput
{
    std::optional<double> iterationAngle;
    bool                  failed            = false;
    double                area              = 0;
    double                errorFraction     = 1;
    IntPoint              newToolPos{0, 0, 0};
    DoublePoint           newToolDir{0, 0};
};

//***************************************
// Adaptive2d main class
//***************************************

class Adaptive2d
{
public:
    double toolDiameter            = 5;
    double helixRampTargetDiameter = 0;
    double helixRampMinDiameter    = 0;
    double stepOverFactor          = 0.2;
    double tolerance               = 0.1; // FreeCAD "accuracy" (dimensionless, 0.01..1)
    bool   forceInsideOut          = true;
    bool   finishingProfile        = true;
    double keepToolDownDistRatio   = 3.0;
    std::function<bool(double)> cancel;

    // Must be called before converting input to algorithm units.
    void Init()
    {
        // keep the tolerance in workable range
        tolerance = std::clamp(tolerance, 0.01, 1.0);
        // 1/"tolerance" = number of min-size adaptive steps per stepover
        scaleFactor = MIN_STEP_CLIPPER / tolerance / std::min(1.0, stepOverFactor * toolDiameter);
        // Round-join chord error of all ClipperLib offsets: half the geometric tolerance (mm,
        // tolerance / 10 - see clear()). The original used Clipper's default 0.25 units (0.5 um
        // at the default accuracy), which makes the cleared-area polygons ~3x denser for no
        // visible gain; every boolean and every cut-area evaluation scales with that density.
        arcTolerance = std::max(0.25, 0.05 * tolerance * scaleFactor);
    }
    double ScaleFactor() const { return scaleFactor; }

    // stockPaths: stock outline (region), paths: area to clear (closed rings), clearedPaths: already
    // cleared area. All in algorithm units. allowOutsideStock = FreeCAD !forceInsideOut for
    // outside clearing; restInput = clearedPaths was supplied by the caller.
    std::vector<AdaptiveOutput> Execute(const Paths& stockPaths, const Paths& paths, const Paths& clearedPaths, bool allowOutsideStock,
                                        bool restInput);

    bool Cancelled() const { return stopProcessing; }

private:
    std::vector<AdaptiveOutput> results;
    Paths                       inputPaths;
    Paths                       stockInputPaths;
    double                      scaleFactor              = 100;
    double                      arcTolerance             = 0.25;
    double                      stepOverScaled           = 1;
    cInt                        toolRadiusScaled         = 10;
    cInt                        finishPassOffsetScaled   = 0;
    cInt                        helixRampMaxRadiusScaled = 0;
    cInt                        helixRampMinRadiusScaled = 0;
    double                      referenceCutArea         = 0;
    double                      optimalCutAreaPD         = 0;
    bool                        stopProcessing           = false;
    double                      totalAreaToCut           = 1;
    double                      areaCut                  = 0;
    long                        pollCounter              = 0;

    Path toolGeometry; // tool geometry at coord 0,0, should not be modified

    void Poll(bool force = false)
    {
        if (!cancel || stopProcessing)
            return;
        if (!force && (++pollCounter & 63) != 0)
            return;
        if (cancel(std::min(1.0, areaCut / totalAreaToCut)))
            stopProcessing = true;
    }

    void ProcessPolyNode(Paths boundPaths, Paths toolBoundPaths, Paths finishingPaths, const Paths& initialClearedPaths);
    bool FindEntryPoint(const Paths& toolBoundPaths, const Paths& bound, ClearedArea& cleared, IntPoint& entryPoint, IntPoint& toolPos,
                        DoublePoint& toolDir, cInt& helixRadiusScaled, AdaptiveOutput& adaptiveOutput);
    std::pair<double, double>  CalcCutArea(IntPoint toolPos, IntPoint newToolPos, ClearedArea& clearedArea);
    std::optional<TPaths>      FindLinkPath(const std::optional<IntPoint>& prevPoint, const IntPoint& pathStart, const DoublePoint& pathDir,
                                            ClearedArea& cleared, const Paths& toolBoundPaths);
    std::optional<std::pair<IntPoint, DoublePoint>> AppendToolPath(AdaptiveOutput& output, const Path& passToolPath, TPaths& linkPath,
                                                                   ClearedArea& cleared, const Paths& toolBoundPaths);
    bool IsClearPath(const Path& path, ClearedArea& clearedArea, double safetyDistanceScaled = 0);
    bool IsAllowedToCutTrough(const IntPoint& p1, const IntPoint& p2, ClearedArea& clearedArea, const Paths& toolBoundPaths,
                              double areaFactor = 1.5, bool skipBoundsCheck = false);
    bool MakeLeadPath(bool leadIn, const IntPoint& startPoint, const DoublePoint& startDir, IntPoint beaconPoint, ClearedArea& clearedArea,
                      const Paths& toolBoundPaths, Path& output);
    bool ResolveLinkPath(const IntPoint& startPoint, const IntPoint& endPoint, ClearedArea& clearedArea, Path& output);
    void ApplyStockToLeave(Paths& inputPaths);

    // See the original for the derivation: guarantees a positive cut area is never evaluated as
    // zero down to a 1 % stepover.
    static constexpr double MIN_STEP_CLIPPER               = 16.0 * 3;
    static constexpr int    MAX_ITERATIONS                 = 30;
    static constexpr double AREA_ERROR_FACTOR              = 0.05; // how precise to match the cut area to optimal
    static constexpr size_t ANGLE_HISTORY_POINTS           = 3;    // used for angle prediction
    static constexpr int    DIRECTION_SMOOTHING_BUFLEN     = 3;    // gyro points - used for angle smoothing
    static constexpr double CLEAN_PATH_TOLERANCE           = 1.415; // should be >sqrt(2)
    static constexpr double FINISHING_CLEAN_PATH_TOLERANCE = 1.415; // should be >sqrt(2)
    static constexpr double FINISHING_THICKNESS_SCALE      = 1 / 10.; // finish at this fraction of stepover
};

// Area inside circle c2 but outside circle c1 and the cleared polygons, by a vertical sweep over
// all x-coordinates of interest (polygon vertices, polygon/circle and circle/circle intersections,
// circle tangents) after rotating so that c1->c2 points up (+y). Also returns the part of that
// area on the left side of travel (x < c2.x), used to detect conventional cutting.
std::pair<double, double> Adaptive2d::CalcCutArea(IntPoint c1i, IntPoint c2i, ClearedArea& clearedArea)
{
    double dist = sqrt(DistanceSqrd(c1i, c2i));
    if (dist < NTOL)
        return {0, 0};

    const double R = double(toolRadiusScaled);

    // 0) Extract from clearedArea a set of polygons close enough to potentially affect the bounded area
    std::vector<std::vector<DoublePoint>> polygons;
    std::vector<DoublePoint>              inters; // temporary, to hold intersection results
    const BoundBox                        c2BB(c2i, toolRadiusScaled);
    // get curves from slightly enlarged region that will cover all points tested in this iteration
    const bool   useC2          = dist > 2 * R;
    const Paths& clearedBounded = clearedArea.GetBoundedClearedAreaClipped(useC2 ? c2i : c1i,
                                                                           toolRadiusScaled + (useC2 ? 0 : cInt(dist)) + 4);

    // 0.5) Rotate all geometry so the vector from c1 to c2 points up (y+)
    const double angle = PI / 2 - atan2(double(c2i.y() - c1i.y()), double(c2i.x() - c1i.x()));
    const double ca    = cos(angle);
    const double sa    = sin(angle);
    const auto   rot   = [ca, sa](double x, double y) { return DoublePoint(ca * x - sa * y, sa * x + ca * y); };

    for (const Path& path : clearedBounded) {
        if (path.empty())
            continue;
        BoundBox pathBB(path.front());
        for (const auto& pt : path)
            pathBB.AddPoint(pt);
        if (!pathBB.CollidesWith(c2BB))
            continue; // this path cannot colide with tool
        std::vector<DoublePoint> polygon;
        polygon.reserve(path.size());
        for (const auto& p : path)
            polygon.push_back(rot(double(p.x()), double(p.y())));
        polygons.push_back(std::move(polygon));
    }
    // The original truncated the rotated centers to integers; kept for identical results.
    const DoublePoint c1r = rot(double(c1i.x()), double(c1i.y()));
    const DoublePoint c2r = rot(double(c2i.x()), double(c2i.y()));
    const DoublePoint c1(double((long long) c1r.x()), double((long long) c1r.y()));
    const DoublePoint c2(double((long long) c2r.x()), double((long long) c2r.y()));

    // 1) Find all x-coordinates of interest:
    std::vector<double> xs;
    for (const auto& polygon : polygons) {
        // 1.a) All polygon vertices
        for (const auto& p : polygon)
            xs.push_back(p.x());
        // 1.b/c) Intersection of all polygons with c1 and c2
        for (size_t i = 0; i < polygon.size(); i++) {
            const auto& p0 = polygon[i];
            const auto& p1 = polygon[(i + 1) % polygon.size()];
            if (Line2CircleIntersect(c1, R, p0, p1, inters))
                for (const auto& p : inters)
                    xs.push_back(p.x());
            if (Line2CircleIntersect(c2, R, p0, p1, inters))
                for (const auto& p : inters)
                    xs.push_back(p.x());
        }
    }

    // 1.e) Compute intersection points between c1 and c2
    {
        std::pair<DoublePoint, DoublePoint> res;
        if (Circle2CircleIntersect(c1, c2, R, res)) {
            xs.push_back(res.first.x());
            xs.push_back(res.second.x());
        }
    }

    // 1.f) Add c1's and c2's vertical tangents to the list
    xs.push_back(c1.x() - R);
    xs.push_back(c1.x() + R);
    const double xmin = c2.x() - R;
    const double xmax = c2.x() + R;
    xs.push_back(xmin);
    xs.push_back(xmax);
    // 1.g) x=c2.X
    xs.push_back(c2.x());

    // 2) Sort these x-coordinates. Discard all values before c2-r or after c2+r
    xs.erase(std::remove_if(xs.begin(), xs.end(), [xmin, xmax](double x) { return !(xmin <= x && x <= xmax); }), xs.end());
    std::sort(xs.begin(), xs.end());

    const auto interpX = [](const DoublePoint& p0, const DoublePoint& p1, double x) {
        const double interp = (x - p0.x()) / (p1.x() - p0.x());
        return (p1.y() * interp) + (p0.y() * (1 - interp));
    };

    // 3) For each non-empty range in x, construct a vertical line through its midpoint
    const DoublePoint circles[2] = {c2, c1};
    double            area             = 0;
    double            conventionalArea = 0;
    std::vector<std::tuple<double, size_t, size_t>> ys;
    std::vector<bool>                               outside;
    for (size_t ix = 0; ix + 1 < xs.size(); ix++) {
        const double x0 = xs[ix];
        const double x1 = xs[ix + 1];
        if (x0 == x1)
            continue;
        const double xtest = (x0 + x1) / 2;

        // 3.a) intersections of the line with each polygon and circle:
        // y, polygon index (or polygons.size() + circle index), edge index (or 0/1 for top/bottom half)
        ys.clear();
        for (size_t ipolygon = 0; ipolygon < polygons.size(); ipolygon++) {
            const auto& polygon = polygons[ipolygon];
            for (size_t iedge = 0; iedge < polygon.size(); iedge++) {
                const auto& p0 = polygon[iedge];
                const auto& p1 = polygon[(iedge + 1) % polygon.size()];
                // note: we skip if the edge is vertical, p0.X == p1.X == xtest
                if (std::min(p0.x(), p1.x()) < xtest && std::max(p0.x(), p1.x()) > xtest)
                    ys.emplace_back(interpX(p0, p1, xtest), ipolygon, iedge);
            }
        }
        for (size_t icircle = 0; icircle < 2; icircle++) {
            const DoublePoint& c  = circles[icircle];
            const double       dx = std::abs(xtest - c.x());
            if (dx < R) { // skip tangent; xtest can't be a tangent anyway
                const double dy = sqrt((R * R) - (dx * dx));
                ys.emplace_back(c.y() + dy, polygons.size() + icircle, 0);
                ys.emplace_back(c.y() - dy, polygons.size() + icircle, 1);
            }
        }

        // 3.b) Sort these intersection on their y-coordinate.
        std::sort(ys.begin(), ys.end(), [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });

        // 3.c) Loop over y-coordinates; init (y=-inf): outsideCount = 1 (outside c2, inside all others)
        // The cleared polygons are one even-odd area (outer boundaries and holes), so they share
        // one state slot: crossing any of their edges toggles it. (The original kept one slot
        // per polygon, which counts the inside of a hole as cleared - with a cleared ring around
        // the stock, as in outside clearing, no material was ever seen.) Slots: cleared, c2, c1.
        outside.assign(3, false);
        outside[1]       = true;
        int outsideCount = 1;
        for (const auto& [_, ishape, ipart] : ys) {
            const size_t slot        = ishape < polygons.size() ? 0 : ishape - polygons.size() + 1;
            const bool   prevOutside = outside[slot];
            const int    prevCount   = outsideCount;
            outside[slot]            = !outside[slot];
            outsideCount += prevOutside ? -1 : 1;

            // We compute integral(exitY - entranceY) as -integral(entranceY - 0) + integral(exitY - 0)
            const double entranceExitSign = prevOutside ? -1 : 1;

            if (outsideCount == 0 || prevCount == 0) {
                double newArea;
                if (ishape < polygons.size()) {
                    // crossed a polygon
                    const auto& polygon = polygons[ishape];
                    const auto& p0      = polygon[ipart];
                    const auto& p1      = polygon[(ipart + 1) % polygon.size()];
                    newArea             = (interpX(p0, p1, x0) + interpX(p0, p1, x1)) / 2 * (x1 - x0);
                } else {
                    // crossed a circle
                    const DoublePoint& c          = circles[ishape - polygons.size()];
                    const double       circleSign = ipart == 0 ? 1 : -1;
                    // area of sector - area of triangle = area of segment
                    const auto   clamp1     = [](double a) { return std::max(-1.0, std::min(1.0, a)); };
                    const double phi0       = acos(clamp1((x0 - c.x()) / R)) * circleSign;
                    const double phi1       = acos(clamp1((x1 - c.x()) / R)) * circleSign;
                    const double areaSector = R * R / 2 * std::abs(phi1 - phi0);
                    const double y0 = c.y() + circleSign * sqrt(std::max(0., R * R - (x0 - c.x()) * (x0 - c.x())));
                    const double y1 = c.y() + circleSign * sqrt(std::max(0., R * R - (x1 - c.x()) * (x1 - c.x())));
                    const double tbase        = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
                    const double tmidx        = (x0 + x1) / 2;
                    const double tmidy        = (y0 + y1) / 2;
                    const double th           = sqrt(((tmidx - c.x()) * (tmidx - c.x())) + ((tmidy - c.y()) * (tmidy - c.y())));
                    const double areaSegment  = areaSector - tbase * th / 2;
                    // then add on trapezoid between the segment and 0 (segment area is negative
                    // for the bottom half of the circle, positive for the top half)
                    const double areaTrapezoid = (x1 - x0) * (y0 + y1) / 2;
                    newArea                    = (circleSign * areaSegment) + areaTrapezoid;
                }
                area += entranceExitSign * newArea;
                if (xtest < c2.x())
                    conventionalArea += entranceExitSign * newArea;
            }
        }
    }

    return {area, conventionalArea};
}

void Adaptive2d::ApplyStockToLeave(Paths& paths)
{
    // Stock to leave is applied by the caller; only the "fix for clipper glitches" remains.
    ClipperOffset clipof(arcTolerance);
    clipof.AddPaths(paths, jtRound, etClosedPolygon);
    clipof.Execute(paths, -1);
    filterCloseValues(paths);
    clipof.Clear();
    clipof.AddPaths(paths, jtRound, etClosedPolygon);
    clipof.Execute(paths, 1);
    filterCloseValues(paths);
}

//********************************************
// Adaptive2d - Execute
//********************************************

std::vector<AdaptiveOutput> Adaptive2d::Execute(const Paths& stockPaths, const Paths& paths, const Paths& clearedPaths,
                                                bool allowOutsideStock, bool restInput)
{
    //**********************************
    // Initializations
    //**********************************
    results.clear();
    toolRadiusScaled = cInt(toolDiameter * scaleFactor / 2);
    stepOverScaled   = toolRadiusScaled * stepOverFactor;
    stopProcessing   = false;

    if (helixRampTargetDiameter < NTOL)
        helixRampTargetDiameter = toolDiameter;
    helixRampTargetDiameter = std::min(helixRampTargetDiameter, toolDiameter);
    helixRampMinDiameter    = std::max(helixRampMinDiameter, toolDiameter / 8);
    helixRampTargetDiameter = std::max(helixRampTargetDiameter, helixRampMinDiameter);

    helixRampMaxRadiusScaled = cInt(helixRampTargetDiameter * scaleFactor / 2);
    helixRampMinRadiusScaled = cInt(helixRampMinDiameter * scaleFactor / 2);
    finishPassOffsetScaled   = finishingProfile ? cInt(stepOverScaled * FINISHING_THICKNESS_SCALE) : 0;

    ClipperOffset clipof(arcTolerance);
    Clipper       clip;

    // generate tool shape
    {
        Paths toolGeometryPaths;
        clipof.AddPath(Path{IntPoint(0, 0, 0)}, jtRound, etOpenRound);
        clipof.Execute(toolGeometryPaths, double(toolRadiusScaled));
        toolGeometry = toolGeometryPaths[0];
    }
    // calculate reference area
    {
        Path slotCut;
        TranslatePath(toolGeometry, slotCut, IntPoint(toolRadiusScaled / 2, 0, 0));
        clip.AddPath(toolGeometry, ptSubject, true);
        clip.AddPath(slotCut, ptClip, true);
        Paths crossing;
        clip.Execute(ctDifference, crossing);
        referenceCutArea = fabs(Area(crossing[0]));
        optimalCutAreaPD = 2 * stepOverFactor * referenceCutArea / toolRadiusScaled;
    }

    //******************************
    // Input paths
    //******************************
    inputPaths.clear();
    for (const Path& p : paths) {
        Path cleaned;
        CleanPolygon(p, cleaned, FINISHING_CLEAN_PATH_TOLERANCE);
        if (cleaned.size() >= 3)
            inputPaths.push_back(std::move(cleaned));
    }
    inputPaths = SimplifyPolygons(inputPaths, pftEvenOdd);
    ApplyStockToLeave(inputPaths);

    stockInputPaths               = SimplifyPolygons(stockPaths, pftEvenOdd);
    Paths initialClearedPaths     = SimplifyPolygons(clearedPaths, pftEvenOdd);

    totalAreaToCut = std::max(1.0, NestedArea(inputPaths));
    areaCut        = 0;

    // 2) Fix input path orientation (nesting count includes self, so odd is positive orientation)
    for (Path& p : inputPaths) {
        int nesting = getPathNestingLevel(p, inputPaths);
        if ((nesting % 2 == 1) ^ Orientation(p))
            ReversePath(p);
    }

    // 3) Set Z=1 on all input paths to tag them as needing a finishing pass. Finishing passes are
    // generated only for edges with z=1 on both end vertices.
    for (Path& path : inputPaths)
        for (IntPoint& p : path)
            p.z() = 1;

    // 5) If going outside the stock is allowed, add a region outside the stock to both inputPaths
    // (where the tool center may go) and the cleared area (so it is known to be free). New paths
    // carry Z=0: the stock boundary needs no finishing pass.
    if (allowOutsideStock) {
        // 5a) Shrink the stock boundary to ensure overlap with input paths that hit the boundary
        Paths stockRev;
        clipof.Clear();
        clipof.AddPaths(stockInputPaths, jtRound, etClosedPolygon);
        clipof.Execute(stockRev, -2);
        ReversePaths(stockRev);

        // 5b) Create outside-of-stock region
        Paths outsideOfStock;
        clipof.Clear();
        clipof.AddPaths(stockInputPaths, jtSquare, etClosedPolygon);
        clipof.Execute(outsideOfStock, 4.0 * toolRadiusScaled);

        // 5c) Union input paths with stock regions; new vertices get Z=1 if both vertices of either
        // input edge have Z=1
        clip.Clear();
        clip.AddPaths(inputPaths, ptSubject, true);
        clip.AddPaths(stockRev, ptClip, true);
        clip.AddPaths(outsideOfStock, ptClip, true);
        clip.ZFillFunction([](const IntPoint& e1bot, const IntPoint& e1top, const IntPoint& e2bot, const IntPoint& e2top, IntPoint& pt) {
            bool edge1HasZ1 = (e1bot.z() == 1 && e1top.z() == 1);
            bool edge2HasZ1 = (e2bot.z() == 1 && e2top.z() == 1);
            pt.z()          = (edge1HasZ1 || edge2HasZ1) ? 1 : 0;
        });
        clip.Execute(ctUnion, inputPaths);
        clip.ZFillFunction(nullptr);

        // 5d) Update cleared area
        clipof.Clear();
        clipof.AddPaths(stockInputPaths, jtSquare, etClosedPolygon);
        clipof.Execute(outsideOfStock, 100.0 * toolRadiusScaled);

        clip.Clear();
        clip.AddPaths(initialClearedPaths, ptSubject, true);
        clip.AddPaths(stockRev, ptClip, true);
        clip.AddPaths(outsideOfStock, ptClip, true);
        clip.Execute(ctUnion, initialClearedPaths);
    }

    // 5e) Rest machining: parts of the input boundary that lie in already-cleared area need no
    // finishing pass. Intersect each input boundary (as an open path, z = vertex index + 1) with
    // the non-cleared area and reassemble it with z=0/1 tags. Without caller-supplied cleared
    // area this is a no-op (every input edge lies inside the stock), so it is skipped.
    if (restInput) {
        Clipper2Lib_Z::Clipper64 clipDiff;
        clipDiff.AddSubject(ToC2(stockInputPaths));
        clipDiff.AddClip(ToC2(initialClearedPaths));
        Clipper2Lib_Z::Paths64 nonclearedPaths;
        clipDiff.Execute(Clipper2Lib_Z::ClipType::Difference, Clipper2Lib_Z::FillRule::EvenOdd, nonclearedPaths);

        Paths updatedPaths;
        for (const auto& path : inputPaths) {
            Clipper2Lib_Z::Path64 indexedPath;
            for (size_t i = 0; i < path.size(); i++)
                indexedPath.emplace_back(path[i].x(), path[i].y(), int64_t(i + 1));
            // explicitly close the path before treating it as open
            indexedPath.push_back(indexedPath[0]);

            const int64_t                 firstNewZ = int64_t(path.size()) + 2;
            int64_t                       nextNewZ  = firstNewZ;
            std::map<int64_t, double>     newZPosition;
            std::map<int64_t, int64_t>    newZNeedsFinishing;

            Clipper2Lib_Z::Clipper64 clip2;
            clip2.SetZCallback([&nextNewZ, &newZPosition, &newZNeedsFinishing, &path](
                                   const Clipper2Lib_Z::Point64& e1bot, const Clipper2Lib_Z::Point64& e1top,
                                   const Clipper2Lib_Z::Point64&, const Clipper2Lib_Z::Point64&, Clipper2Lib_Z::Point64& pt) {
                if (pt.x == e1bot.x && pt.y == e1bot.y) {
                    pt.z = e1bot.z;
                    return;
                }
                if (pt.x == e1top.x && pt.y == e1top.y) {
                    pt.z = e1top.z;
                    return;
                }
                pt.z = nextNewZ++;

                // position of the new point: interpolate the z-values (indices) of the parent input
                // edge (e1); the closing edge path.size() -> 1 interpolates to path.size() + 1
                double dx_total   = double(e1top.x - e1bot.x);
                double dy_total   = double(e1top.y - e1bot.y);
                double dist_total = sqrt(dx_total * dx_total + dy_total * dy_total);
                double dx_pt      = double(pt.x - e1bot.x);
                double dy_pt      = double(pt.y - e1bot.y);
                double dist_pt    = sqrt(dx_pt * dx_pt + dy_pt * dy_pt);
                const size_t n    = path.size();
                double e1bot_eff  = (size_t(e1bot.z) == 1 && size_t(e1top.z) == n) ? double(n + 1) : double(e1bot.z);
                double e1top_eff  = (e1top.z == 1 && size_t(e1bot.z) == n) ? double(n + 1) : double(e1top.z);
                double interp     = dist_total > 0 ? dist_pt / dist_total : 0;
                newZPosition[pt.z] = e1bot_eff + interp * (e1top_eff - e1bot_eff);

                int64_t e1bot_finishing = (e1bot.z >= 1 && size_t(e1bot.z) <= n) ? path[e1bot.z - 1].z() : newZNeedsFinishing[e1bot.z];
                int64_t e1top_finishing = (e1top.z >= 1 && size_t(e1top.z) <= n) ? path[e1top.z - 1].z() : newZNeedsFinishing[e1top.z];
                newZNeedsFinishing[pt.z] = (e1bot_finishing && e1top_finishing) ? 1 : 0;
            });

            clip2.AddOpenSubject({indexedPath});
            clip2.AddClip(nonclearedPaths);
            Clipper2Lib_Z::Paths64 unusedClosedPaths, nonclearedInputs;
            clip2.Execute(Clipper2Lib_Z::ClipType::Intersection, Clipper2Lib_Z::FillRule::EvenOdd, unusedClosedPaths, nonclearedInputs);

            // collect points of the intersection, dropping duplicate representations of the start
            std::vector<IntPoint> nonclearedPoints;
            bool                  hasZ1 = false;
            for (const auto& nonclearedPath : nonclearedInputs)
                for (const auto& pt : nonclearedPath) {
                    if (pt.z != 1 || !hasZ1)
                        nonclearedPoints.emplace_back(pt.x, pt.y, pt.z);
                    hasZ1 |= pt.z == 1;
                }
            indexedPath.pop_back();

            const auto position = [&](cInt z) { return z >= firstNewZ ? newZPosition.at(z) : double(z); };
            std::sort(nonclearedPoints.begin(), nonclearedPoints.end(),
                      [&](const IntPoint& a, const IntPoint& b) { return position(a.z()) < position(b.z()); });

            // Reassemble: original-only points are in cleared area (z=0); intersection-only points
            // are split points (finish if the parent edge wanted it); points in both keep their tag.
            Path updatedPath;
            auto nonclearedIt = nonclearedPoints.begin();
            auto indexedIt    = indexedPath.begin();
            while (indexedIt != indexedPath.end() || nonclearedIt != nonclearedPoints.end()) {
                double nextNonclearedPos = nonclearedIt != nonclearedPoints.end() ? position(nonclearedIt->z()) : DBL_MAX;
                double nextPathPos       = indexedIt != indexedPath.end() ? double(indexedIt->z) : DBL_MAX;
                if (nextPathPos < nextNonclearedPos) {
                    IntPoint newPt = path[indexedIt->z - 1];
                    newPt.z()      = 0;
                    updatedPath.push_back(newPt);
                    ++indexedIt;
                } else if (nextNonclearedPos < nextPathPos) {
                    IntPoint newPt = *nonclearedIt;
                    newPt.z()      = newZNeedsFinishing.at(newPt.z());
                    updatedPath.push_back(newPt);
                    ++nonclearedIt;
                } else {
                    updatedPath.push_back(path[indexedIt->z - 1]);
                    ++indexedIt;
                    ++nonclearedIt;
                }
            }
            updatedPaths.push_back(std::move(updatedPath));
        }
        inputPaths = std::move(updatedPaths);
    }

    // 6) toolBounds = offset(input paths, -(toolRadius + finishingThickness)): where the tool
    // center may be during the main adaptive pass. Clipper2 offset keeps the Z tags.
    Clipper2Lib_Z::ClipperOffset clipof2;
    clipof2.PreserveCollinear(true);
    clipof2.AddPaths(ToC2(inputPaths), Clipper2Lib_Z::JoinType::Round, Clipper2Lib_Z::EndType::Polygon);
    clipof2.SetZCallback([](const Clipper2Lib_Z::Point64& e1bot, const Clipper2Lib_Z::Point64& e1top, const Clipper2Lib_Z::Point64& e2bot,
                            const Clipper2Lib_Z::Point64& e2top, Clipper2Lib_Z::Point64& pt) {
        bool edge1HasZ1 = (e1bot.z == 1 && e1top.z == 1);
        bool edge2HasZ1 = (e2bot.z == 1 && e2top.z == 1);
        pt.z            = (edge1HasZ1 || edge2HasZ1) ? 1 : 0;
    });
    Clipper2Lib_Z::Paths64 toolBounds2;
    clipof2.Execute(-double(toolRadiusScaled + finishPassOffsetScaled), toolBounds2);
    const Paths toolBounds = FromC2(toolBounds2);

    // 7) Loop over connected components of toolBounds (outer boundary + its direct holes), so a
    // helix entry is considered only for the initial engagement of each component.
    for (const auto& current : toolBounds) {
        if (stopProcessing)
            break;
        int nesting = getPathNestingLevel(current, toolBounds);
        if (nesting % 2 == 0)
            continue; // a hole
        Paths currentTBP{current};
        for (const auto& other : toolBounds)
            if (PointInPolygon(other.front(), current) != 0 && getPathNestingLevel(other, toolBounds) == nesting + 1)
                currentTBP.push_back(other);

        // 8) finishingPass = offset(currentTBP, finishingThickness): offsetting back out from the
        // tool bounds guarantees every finishing point is a small-stepover cut from cleared area.
        clipof2.Clear();
        clipof2.AddPaths(ToC2(currentTBP), Clipper2Lib_Z::JoinType::Round, Clipper2Lib_Z::EndType::Polygon);
        Clipper2Lib_Z::Paths64 finishingPass2;
        clipof2.Execute(double(finishPassOffsetScaled), finishingPass2);
        Paths finishingPass = FromC2(finishingPass2);

        // 9) bounds = offset(currentTBP, toolRadius), 3 units smaller than nominal to absorb the
        // integer rounding of the three offsets involved (see the original for details).
        Paths boundPath;
        clipof.Clear();
        clipof.AddPaths(currentTBP, jtRound, etClosedPolygon);
        clipof.Execute(boundPath, double(toolRadiusScaled - 3));

        // Skip path generation if bounds are fully cleared
        {
            Paths boundsToClear;
            clip.Clear();
            clip.AddPaths(boundPath, ptSubject, true);
            clip.AddPaths(initialClearedPaths, ptClip, true);
            clip.Execute(ctDifference, boundsToClear);
            if (boundsToClear.empty())
                continue;
        }

        // 10) Run core algorithm on (bounds, toolBounds, finishingPass, clearedArea)
        ProcessPolyNode(boundPath, currentTBP, finishingPass, initialClearedPaths);
    }

    return std::move(results);
}

bool Adaptive2d::FindEntryPoint(const Paths& toolBoundPaths, const Paths& boundPaths, ClearedArea& clearedArea, IntPoint& entryPoint,
                                IntPoint& toolPos, DoublePoint& toolDir, cInt& helixRadiusScaled, AdaptiveOutput& adaptiveOutput)
{
    Paths         incOffset;
    Paths         lastValidOffset;
    Clipper       clip;
    ClipperOffset clipof(arcTolerance);
    bool          found = false;
    Paths         clearedPaths;

    Paths checkPaths;
    clip.AddPaths(toolBoundPaths, ptSubject, true);
    clip.AddPaths(clearedArea.GetCleared(), ptClip, true);
    clip.Execute(ctDifference, checkPaths);

    // check if helix fits, and make the area cleared by it
    const auto checkHelixFit = [&](cInt testHelixRadiusScaled) {
        clipof.Clear();
        clipof.AddPath(Path{entryPoint}, jtRound, etOpenRound);
        clipof.Execute(clearedPaths, double(testHelixRadiusScaled + toolRadiusScaled));
        CleanPolygons(clearedPaths);
        // check if it is crossing the boundary
        clip.Clear();
        clip.AddPaths(clearedPaths, ptSubject, true);
        clip.AddPaths(boundPaths, ptClip, true);
        Paths crossing;
        clip.Execute(ctDifference, crossing);
        return crossing.empty();
    };

    for (int iter = 0; iter < 10; iter++) {
        // find the deepest point by incremental inward offsets
        clipof.Clear();
        clipof.AddPaths(checkPaths, jtSquare, etClosedPolygon);
        double step         = MIN_STEP_CLIPPER;
        double currentDelta = -1;
        clipof.Execute(incOffset, currentDelta);
        while (!incOffset.empty()) {
            clipof.Execute(incOffset, currentDelta);
            if (!incOffset.empty())
                lastValidOffset = incOffset;
            currentDelta -= step;
        }
        for (const Path& p : lastValidOffset)
            if (!p.empty()) {
                entryPoint = Compute2DPolygonCentroid(p);
                found      = true;
                break;
            }
        // check if the start point is in any of the holes; this may happen when toolBoundPaths
        // are symmetric (boundary + holes) - we need to break symmetry and try again
        for (size_t j = 0; j < checkPaths.size(); j++) {
            int pip = PointInPolygon(entryPoint, checkPaths[j]);
            if ((j == 0 && pip == 0) || (j > 0 && pip != 0)) {
                found = false;
                break;
            }
        }

        if (found) {
            if (!checkHelixFit(helixRampMinRadiusScaled)) {
                found = false; // min-size helix does not fit
            } else {
                // find the largest helix that fits
                cInt minSize = helixRampMinRadiusScaled;
                cInt maxSize = helixRampMaxRadiusScaled;
                while (minSize < maxSize) {
                    cInt testSize = (minSize + maxSize + 1) / 2;
                    if (checkHelixFit(testSize))
                        minSize = testSize;
                    else
                        maxSize = testSize - 1;
                }
                helixRadiusScaled = minSize;
                checkHelixFit(helixRadiusScaled); // set clearedPaths for final size
                clearedArea.AddClearedPaths(clearedPaths);
            }
        }

        if (found)
            break;

        // break symmetry and try again
        clip.Clear();
        clip.AddPaths(checkPaths, ptSubject, true);
        IntRect bounds = clip.GetBounds();
        clip.Clear();
        Path rect{IntPoint(bounds.left, bounds.bottom, 0), IntPoint(bounds.left, (bounds.top + bounds.bottom) / 2, 0),
                  IntPoint((bounds.left + bounds.right) / 2, (bounds.top + bounds.bottom) / 2, 0),
                  IntPoint((bounds.left + bounds.right) / 2, bounds.bottom, 0)};
        clip.AddPath(rect, ptSubject, true);
        clip.AddPaths(checkPaths, ptClip, true);
        clip.Execute(ctIntersection, checkPaths);
    }

    if (!found) {
        adaptiveOutput.StartPointNotFound = true;
        return false;
    }
    toolPos = IntPoint(entryPoint.x(), entryPoint.y() - helixRadiusScaled, 0);
    toolDir = DoublePoint(1.0, 0.0);
    return true;
}

//************************************************************
//  IsClearPath - returns true if path is clear from obstacles
//***********************************************************
bool Adaptive2d::IsClearPath(const Path& tp, ClearedArea& cleared, double safetyClearance)
{
    // The original offset the path by the tool radius and required "tool shape - cleared" to
    // have area < 1; the distance test is equivalent (to rounding) and O(edges).
    return SweptInside(cleared.GetCleared(), tp, toolRadiusScaled + safetyClearance - 1);
}

bool Adaptive2d::IsAllowedToCutTrough(const IntPoint& p1, const IntPoint& p2, ClearedArea& cleared, const Paths& toolBoundPaths,
                                      double areaFactor, bool skipBoundsCheck)
{
    if (!skipBoundsCheck && (!IsPointWithinCutRegion(toolBoundPaths, p2) || !IsPointWithinCutRegion(toolBoundPaths, p1)))
        return false; // first or last point outside boundary

    double distance = sqrt(DistanceSqrd(p1, p2));
    double stepSize = std::min(0.5 * stepOverScaled, 8 * MIN_STEP_CLIPPER);
    if (distance < stepSize / 2)
        return true; // not significant cut
    if (distance < stepSize)
        areaFactor *= 2; // adjust for numeric instability with small distances

    IntPoint toolPos1 = p1;
    long     steps    = long(distance / stepSize) + 1;
    stepSize          = distance / steps;
    for (long i = 1; i <= steps; i++) {
        double   p        = double(i) / steps;
        IntPoint toolPos2 = IP(p1.x() + double(p2.x() - p1.x()) * p, p1.y() + double(p2.y() - p1.y()) * p);
        double   area     = CalcCutArea(toolPos1, toolPos2, cleared).first;
        // if we are cutting above optimal -> not clear to cut
        if (area > areaFactor * stepSize * optimalCutAreaPD)
            return false;
        // if tool is outside boundary -> its not clear to cut
        if (!skipBoundsCheck && !IsPointWithinCutRegion(toolBoundPaths, toolPos2))
            return false;
        toolPos1 = toolPos2;
    }
    return true;
}

bool Adaptive2d::ResolveLinkPath(const IntPoint& startPoint, const IntPoint& endPoint, ClearedArea& clearedArea, Path& output)
{
    std::vector<std::pair<IntPoint, IntPoint>> queue;
    queue.emplace_back(startPoint, endPoint);
    Path   checkPath;
    double totalLength    = 0;
    double directDistance = sqrt(DistanceSqrd(startPoint, endPoint));
    Paths  linkPaths;

    double scanStep = std::clamp(2 * MIN_STEP_CLIPPER, scaleFactor * 0.01, scaleFactor * 0.1);
    const long limit = 10000;

    double clearance    = stepOverScaled;
    double offClearance = 2 * stepOverScaled;
    if (offClearance > directDistance / 2) {
        offClearance = directDistance / 2;
        clearance    = 0;
    }

    long cnt = 0;

    // to hold CLP results
    IntPoint clp;
    size_t   pindex;
    size_t   sindex;
    double   par;

    // The original also had a wall-clock limit here; removed for deterministic output, the
    // iteration limit bounds the worst case.
    while (!queue.empty()) {
        if (stopProcessing)
            return false;
        if (++cnt > limit)
            return false; // unable to resolve tool down linking path
        std::pair<IntPoint, IntPoint> pointPair = queue.back();
        queue.pop_back();

        // check for self intersections - if found discard the link path
        for (const Path& lp : linkPaths)
            if (!SameXY(lp.front(), pointPair.first) && !SameXY(lp.back(), pointPair.first) && !SameXY(lp.front(), pointPair.second) &&
                !SameXY(lp.back(), pointPair.second) && IntersectionPoint(lp.front(), lp.back(), pointPair.first, pointPair.second, clp))
                return false;

        DoublePoint direction = DirectionV(pointPair.first, pointPair.second);
        checkPath.clear();
        if (SameXY(pointPair.first, startPoint))
            checkPath.push_back(IP(pointPair.first.x() + offClearance * direction.x(), pointPair.first.y() + offClearance * direction.y()));
        else
            checkPath.push_back(pointPair.first);
        if (SameXY(pointPair.second, endPoint))
            checkPath.push_back(IP(pointPair.second.x() - offClearance * direction.x(), pointPair.second.y() - offClearance * direction.y()));
        else
            checkPath.push_back(pointPair.second);

        if (IsClearPath(checkPath, clearedArea, clearance)) {
            totalLength += sqrt(DistanceSqrd(pointPair.first, pointPair.second));
            if (totalLength > keepToolDownDistRatio * directDistance)
                return false;
            linkPaths.push_back(Path{pointPair.first, pointPair.second});
        } else {
            if (sqrt(DistanceSqrd(pointPair.first, pointPair.second)) < 4)
                return false; // segment became too short but still not clear
            DoublePoint pDir(-direction.y(), direction.x());
            // find mid point
            IntPoint midPoint = IP(0.5 * double(pointPair.first.x() + pointPair.second.x()), 0.5 * double(pointPair.first.y() + pointPair.second.y()));
            for (long i = 1;; i++) {
                if (stopProcessing)
                    return false;
                double   offset      = i * scanStep;
                IntPoint checkPoint1 = IP(midPoint.x() + offset * pDir.x(), midPoint.y() + offset * pDir.y());
                IntPoint checkPoint2 = IP(midPoint.x() - offset * pDir.x(), midPoint.y() - offset * pDir.y());

                if (DistancePointToPathsSqrd(clearedArea.GetCleared(), checkPoint1, clp, pindex, sindex, par) <
                    DistancePointToPathsSqrd(clearedArea.GetCleared(), checkPoint2, clp, pindex, sindex, par))
                    std::swap(checkPoint1, checkPoint2);

                checkPath = Path{checkPoint1};
                if (IsClearPath(checkPath, clearedArea, clearance + 1)) { // check if point clear
                    queue.emplace_back(pointPair.first, checkPoint1);
                    queue.emplace_back(checkPoint1, pointPair.second);
                    break;
                }
                checkPath = Path{checkPoint2}; // check the other side
                if (IsClearPath(checkPath, clearedArea, clearance + 1)) {
                    queue.emplace_back(pointPair.first, checkPoint2);
                    queue.emplace_back(checkPoint2, pointPair.second);
                    break;
                }
                if (offset > keepToolDownDistRatio * directDistance)
                    return false; // can't find keep tool down link
            }
        }
    }
    if (linkPaths.empty())
        return false;
    ConnectPaths(linkPaths, linkPaths);
    output = linkPaths[0];
    return true;
}

bool Adaptive2d::MakeLeadPath(bool leadIn, const IntPoint& startPoint, const DoublePoint& startDir, IntPoint beaconPoint,
                              ClearedArea& clearedAreaOriginal, const Paths& toolBoundPaths, Path& output)
{
    output.push_back(startPoint);
    double stepSize = std::min(MIN_STEP_CLIPPER * 8, 0.2 * stepOverScaled + 1);

    // make a copy of clearedArea to update as the path progresses (for lead out only)
    ClearedArea clearedArea(toolRadiusScaled, arcTolerance);
    clearedArea.SetClearedPaths(clearedAreaOriginal.GetCleared());

    // acceptable tool end locations: offset(cleared, -(toolRadius + stepSize)). The original
    // recomputed that offset of the whole cleared area on every lead-out step; membership is
    // tested with the equivalent distance test instead, and the offset is only built when the
    // beacon has to be moved.
    const double endClearance = toolRadiusScaled + stepSize;
    // (tested on the cut-area window: the area within endClearance of p is all it needs)
    const auto isEndLocation = [&](const IntPoint& p) {
        return SweptInside(clearedArea.GetBoundedClearedAreaClipped(p, cInt(endClearance) + 8), Path{p}, endClearance);
    };

    // move the beacon to an acceptable location if necessary
    if (!isEndLocation(beaconPoint)) {
        ClipperOffset clipof(arcTolerance);
        Paths         cleared;
        clipof.AddPaths(clearedArea.GetCleared(), jtRound, etClosedPolygon);
        clipof.Execute(cleared, -endClearance);
        if (cleared.empty())
            return false;
        IntPoint clp;
        size_t   clpPathIndex;
        size_t   clpSegmentIndex;
        double   clpParameter;
        DistancePointToPathsSqrd(cleared, beaconPoint, clp, clpPathIndex, clpSegmentIndex, clpParameter);
        beaconPoint = clp;
    }

    IntPoint    currentPoint = startPoint;
    DoublePoint targetDir    = DirectionV(currentPoint, beaconPoint);

    double                distanceToBeacon = sqrt(DistanceSqrd(startPoint, beaconPoint));
    double                minExitLength    = std::min(toolRadiusScaled / 5., std::min(stepOverScaled, distanceToBeacon / 2));
    double                maxLength        = std::max(distanceToBeacon * 2, stepSize * 10);
    std::optional<double> clearedStartLen;
    DoublePoint           nextDir   = startDir;
    IntPoint              nextPoint = IP(currentPoint.x() + nextDir.x() * stepSize, currentPoint.y() + nextDir.y() * stepSize);
    Path                  checkPath{currentPoint};
    const double          adaptFactor = 0.4;
    const double          alfa        = PI / 64;
    double                pathLen     = 0;
    for (int i = 0; i < 10000; i++) {
        if (IsAllowedToCutTrough(currentPoint, nextPoint, clearedArea, toolBoundPaths)) {
            if (!leadIn) {
                // For lead out paths, update/recompute the cleared area
                checkPath.push_back(nextPoint);
                clearedArea.ExpandCleared(checkPath);
                checkPath = Path{nextPoint};
            }

            output.push_back(nextPoint);
            currentPoint = nextPoint;
            pathLen += stepSize;
            targetDir = DirectionV(currentPoint, beaconPoint);
            nextDir   = DoublePoint(nextDir.x() + adaptFactor * targetDir.x(), nextDir.y() + adaptFactor * targetDir.y());
            NormalizeV(nextDir);

            // check if cleared
            if (isEndLocation(currentPoint)) {
                if (!clearedStartLen)
                    clearedStartLen = pathLen;
                // if the path is long enough, exit with success
                if (pathLen > minExitLength && pathLen - *clearedStartLen > MIN_STEP_CLIPPER)
                    return true;
            } else {
                clearedStartLen.reset();
            }

            // if traveled too far without getting to a clear area, exit
            if (pathLen > maxLength)
                return getPathNestingLevel(Path{currentPoint}, clearedArea.GetCleared()) % 2 == 1;
        } else {
            nextDir = rotate(nextDir, leadIn ? -alfa : alfa);
        }
        nextPoint = IP(currentPoint.x() + nextDir.x() * stepSize, currentPoint.y() + nextDir.y() * stepSize);
    }
    return false;
}

std::optional<TPaths> Adaptive2d::FindLinkPath(const std::optional<IntPoint>& prevPoint, const IntPoint& pathStart, const DoublePoint& pathDir,
                                               ClearedArea& cleared, const Paths& toolBoundPaths)
{
    TPaths   result;
    IntPoint endPoint(pathStart);

    // if the link distance is very short, no special linking is required
    double linkDistance = prevPoint ? sqrt(DistanceSqrd(*prevPoint, endPoint)) : stepOverScaled;
    if (linkDistance < NTOL)
        return result;

    size_t   clpPathIndex;
    size_t   clpSegmentIndex;
    double   clpParameter;
    IntPoint clp;

    double beaconOffset = std::max(std::min(stepOverScaled, linkDistance / 2) * 1.5, 8 * MIN_STEP_CLIPPER);

    // plan the lead in, as a reverse-direction lead out
    double eDistToBounds = DistancePointToPathsSqrd(toolBoundPaths, endPoint, clp, clpPathIndex, clpSegmentIndex, clpParameter);

    DoublePoint revEndDir(-pathDir.x(), -pathDir.y());
    DoublePoint endBoundaryDir = GetPathDirectionV(toolBoundPaths[clpPathIndex], clpSegmentIndex);
    if (eDistToBounds > beaconOffset)
        endBoundaryDir = pathDir; // if boundary is far away, use beacon to leave the path
    DoublePoint endBeaconDir(revEndDir.x() - endBoundaryDir.y(), revEndDir.y() + endBoundaryDir.x());
    NormalizeV(endBeaconDir);

    IntPoint endBeacon = IP(endPoint.x() + beaconOffset * endBeaconDir.x(), endPoint.y() + beaconOffset * endBeaconDir.y());

    Path leadInPath;
    bool leadInOk = MakeLeadPath(true, endPoint, revEndDir, endBeacon, cleared, toolBoundPaths, leadInPath);
    ReversePath(leadInPath);
    if (!leadInOk)
        return {};

    // Compute linking path
    Path       linkPath;
    MotionType linkType = mtCutting;

    if (prevPoint) {
        if (ResolveLinkPath(*prevPoint, leadInPath.front(), cleared, linkPath)) {
            linkType                        = mtLinkClear;
            double remainingLeadInExtension = stepOverScaled / 2;
            while (linkPath.size() >= 2 && remainingLeadInExtension > NTOL) {
                IntPoint p1 = linkPath[linkPath.size() - 2];
                IntPoint p2 = linkPath[linkPath.size() - 1];
                double   l  = sqrt(DistanceSqrd(p1, p2));
                if (l >= remainingLeadInExtension) {
                    IntPoint splitPoint = IP(p1.x() + (p2.x() - p1.x()) * (l - remainingLeadInExtension) / l,
                                             p1.y() + (p2.y() - p1.y()) * (l - remainingLeadInExtension) / l);
                    linkPath.pop_back();
                    linkPath.push_back(splitPoint);
                    leadInPath.insert(leadInPath.begin(), splitPoint);
                    remainingLeadInExtension = 0;
                    if (!IsClearPath(Path{p2, splitPoint}, cleared, 0))
                        remainingLeadInExtension = stepOverScaled / 2;
                } else {
                    linkPath.pop_back();
                    leadInPath.insert(leadInPath.begin(), p1);
                    remainingLeadInExtension -= l;
                    if (remainingLeadInExtension < NTOL && !IsClearPath(Path{p2, p1}, cleared, 0))
                        remainingLeadInExtension = stepOverScaled / 2;
                }
            }
        } else {
            linkType    = mtLinkNotClear;
            double dist = sqrt(DistanceSqrd(*prevPoint, leadInPath.front()));
            if (dist < 2 * stepOverScaled &&
                IsAllowedToCutTrough(IP(prevPoint->x() + (leadInPath.front().x() - prevPoint->x()) / dist,
                                        prevPoint->y() + (leadInPath.front().y() - prevPoint->y()) / dist),
                                     IP(leadInPath.front().x() - (leadInPath.front().x() - prevPoint->x()) / dist,
                                        leadInPath.front().y() - (leadInPath.front().y() - prevPoint->y()) / dist),
                                     cleared, toolBoundPaths))
                linkType = mtCutting;
            // add direct linking move at clear height
            linkPath = Path{*prevPoint, leadInPath.front()};
        }
    }

    /* paths smoothing*/
    if (linkType == mtLinkClear) {
        Paths linkPaths{linkPath, leadInPath};
        SmoothPaths(linkPaths, 0.1 * stepOverScaled, 1, 4);
        linkPath   = linkPaths[0];
        leadInPath = linkPaths[1];
    }

    if (prevPoint)
        result.emplace_back(linkType, linkPath);
    result.emplace_back(mtCutting, leadInPath);
    return result;
}

std::optional<std::pair<IntPoint, DoublePoint>> Adaptive2d::AppendToolPath(AdaptiveOutput& output, const Path& passToolPath, TPaths& linkPath,
                                                                           ClearedArea& cleared, const Paths& toolBoundPaths)
{
    std::optional<std::pair<IntPoint, DoublePoint>> result; // new toolPos and toolDir after lead out

    for (TPath& lp : linkPath)
        output.AdaptivePaths.push_back(lp);

    if (passToolPath.empty())
        return result;
    output.AdaptivePaths.emplace_back(mtCutting, passToolPath);

    // plan the lead out
    if (passToolPath.size() < 2)
        return result;
    IntPoint    prevPoint = passToolPath.back();
    DoublePoint prevDir   = GetPathDirectionV(passToolPath, passToolPath.size() - 1);

    size_t   clpPathIndex;
    size_t   clpSegmentIndex;
    double   clpParameter;
    IntPoint clp;
    double   distToBounds = DistancePointToPathsSqrd(toolBoundPaths, prevPoint, clp, clpPathIndex, clpSegmentIndex, clpParameter);

    DoublePoint boundaryDir  = GetPathDirectionV(toolBoundPaths[clpPathIndex], clpSegmentIndex);
    double      beaconOffset = std::max(std::min(stepOverScaled, PathLength(passToolPath) / 2) * 1.5, 8 * MIN_STEP_CLIPPER);
    if (distToBounds > beaconOffset)
        boundaryDir = prevDir; // if boundary is far away, use beacon to leave the path
    DoublePoint beaconDir(prevDir.x() - boundaryDir.y(), prevDir.y() + boundaryDir.x());
    NormalizeV(beaconDir);

    IntPoint beacon = IP(prevPoint.x() + beaconOffset * beaconDir.x(), prevPoint.y() + beaconOffset * beaconDir.y());

    Path leadOutPath;
    bool ok = MakeLeadPath(false, prevPoint, prevDir, beacon, cleared, toolBoundPaths, leadOutPath);

    if (ok && !leadOutPath.empty()) {
        Paths linkPaths{leadOutPath};
        SmoothPaths(linkPaths, 0.1 * stepOverScaled, 1, 4);
        leadOutPath = linkPaths[0];

        output.AdaptivePaths.emplace_back(mtCutting, leadOutPath);
        cleared.ExpandCleared(leadOutPath);

        IntPoint p2 = leadOutPath.back();
        IntPoint p1 = leadOutPath.size() >= 2 ? leadOutPath[leadOutPath.size() - 2] : prevPoint;
        result      = {{p2, DirectionV(p1, p2)}};
    }
    return result;
}

void Adaptive2d::ProcessPolyNode(Paths boundPaths, Paths toolBoundPaths, Paths finishingPaths, const Paths& initialClearedPaths)
{
    // node paths are already constrained to tool boundary path for adaptive path before finishing pass
    Clipper       clip;
    ClipperOffset clipof(arcTolerance);

    IntPoint entryPoint(0, 0, 0);
    cInt     helixRadiusScaled = 0;

    CleanPolygons(toolBoundPaths);
    toolBoundPaths = SimplifyPolygons(toolBoundPaths, pftEvenOdd);

    CleanPolygons(boundPaths);
    boundPaths = SimplifyPolygons(boundPaths, pftEvenOdd);

    Paths tbpMinus; // toolBoundPaths shrunk by some buffer room
    clipof.AddPaths(toolBoundPaths, jtRound, etClosedPolygon);
    clipof.Execute(tbpMinus, -2);
    CleanPolygons(tbpMinus);
    tbpMinus = SimplifyPolygons(tbpMinus, pftEvenOdd);

    IntPoint    toolPos(0, 0, 0);
    DoublePoint toolDir(0, 0);

    // Initialize cleared area from previously cleared paths
    ClearedArea cleared(toolRadiusScaled, arcTolerance);
    cleared.SetClearedPaths(initialClearedPaths);

    cInt stepScaled = cInt(MIN_STEP_CLIPPER);

    Path                     passToolPath; // to store pass toolpath
    Path                     toClearPath;
    IntPoint                 clp; // to store closest point
    size_t                   clpPathIndex;
    size_t                   clpSegmentIndex;
    double                   clpParameter;
    std::vector<DoublePoint> gyro;         // used to average tool direction
    std::vector<double>      angleHistory; // use to predict deflection angle
    double                   angle = PI;
    Interpolation            interp;

    long bad_engage_count = 0;

    AdaptiveOutput output;
    ClearedArea    clearedBeforePass(toolRadiusScaled, arcTolerance);
    clearedBeforePass.SetClearedPaths(cleared.GetCleared());

    DoublePoint lastExpandToolDir = toolDir;

    const auto iterateNextStep = [&](const IntPoint& toolPos, const DoublePoint& toolDir) {
        IterateNextStepOutput out;

        double      distanceToBoundary = sqrt(DistancePointToPathsSqrd(toolBoundPaths, toolPos, clp, clpPathIndex, clpSegmentIndex, clpParameter));
        DoublePoint boundaryDir        = GetPathDirectionV(toolBoundPaths[clpPathIndex], clpSegmentIndex);
        double      distanceToEngage   = sqrt(DistanceSqrd(toolPos, entryPoint));

        double targetAreaPD = optimalCutAreaPD;

        // set the step size: 1x to 8x base size
        double slowDownDistance = std::max(double(toolRadiusScaled) / 4, MIN_STEP_CLIPPER * 8);
        if (distanceToBoundary < slowDownDistance || distanceToEngage < slowDownDistance)
            stepScaled = cInt(MIN_STEP_CLIPPER);
        else if (fabs(angle) > NTOL)
            stepScaled = cInt(MIN_STEP_CLIPPER / fabs(angle));
        else
            stepScaled = cInt(MIN_STEP_CLIPPER * 8);

        // clamp the step size - for stability
        stepScaled = std::min(stepScaled, std::min(cInt(toolRadiusScaled / 4), cInt(MIN_STEP_CLIPPER * 8)));
        stepScaled = std::max(stepScaled, cInt(MIN_STEP_CLIPPER));

        //*****************************
        // ANGLE vs AREA ITERATIONS
        //*****************************
        double       predictedAngle     = averageDV(angleHistory);
        double       maxError           = AREA_ERROR_FACTOR * optimalCutAreaPD;
        double       errorFraction      = 1;
        double       area               = 0;
        bool         isConventional     = false;
        const double conventionalCutoff = 0.51; // allow some room for rounding, but otherwise < 50%
        double       areaPD             = 0;
        interp.clear();
        bool        pointNotInterp = true;
        bool        foundArea      = false;
        IntPoint    newToolPos(0, 0, 0);
        DoublePoint newToolDir(0, 0);
        for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
            if (iteration == 0) {
                angle          = predictedAngle;
                pointNotInterp = true;
            } else if (iteration == 1) {
                angle          = Interpolation::MIN_ANGLE; // max engage
                pointNotInterp = true;
            } else if (iteration == 2) {
                if (interp.bothSides()) {
                    angle          = interp.interpolateAngle();
                    pointNotInterp = false;
                } else {
                    angle          = Interpolation::MAX_ANGLE; // min engage
                    pointNotInterp = true;
                }
            } else if (iteration == 3 && !foundArea) {
                // Expand cleared area
                cleared.ExpandCleared(toClearPath);
                toClearPath.clear();
                lastExpandToolDir = toolDir;

                // Find nearby uncleared area in the forward direction
                // 1.5 > sqrt(2) for the constructed triangle to contain possible steps
                double      dist       = (stepScaled + toolRadiusScaled) * 1.5;
                DoublePoint leftAngle  = rotate(toolDir, -PI / 4);
                DoublePoint rightAngle = rotate(toolDir, PI / 4);
                Path        triangle{toolPos, IP(toolPos.x() + rightAngle.x() * dist, toolPos.y() + rightAngle.y() * dist),
                                     IP(toolPos.x() + leftAngle.x() * dist, toolPos.y() + leftAngle.y() * dist)};

                Paths unclearedArea;
                clip.Clear();
                clip.AddPath(triangle, ptSubject, true);
                clip.AddPaths(cleared.GetCleared(), ptClip, true);
                clip.Execute(ctDifference, unclearedArea);

                if (unclearedArea.empty())
                    continue;

                // Find the closest point on the boundary, and try stepping towards it
                DistancePointToPathsSqrd(unclearedArea, toolPos, clp, clpPathIndex, clpSegmentIndex, clpParameter);
                double dy  = double(clp.y() - toolPos.y());
                double dx  = double(clp.x() - toolPos.x());
                double len = sqrt(dx * dx + dy * dy);
                angle      = asin((dy * toolDir.x() - dx * toolDir.y()) / len);
            } else if (!foundArea) {
                // if the previous iteration didn't cut area then nothing will; exit early
                angle  = 0;
                area   = 0;
                areaPD = 0;
                break;
            } else {
                angle          = interp.interpolateAngle();
                pointNotInterp = false;
            }
            angle = Interpolation::clampAngle(angle);

            newToolDir = rotate(toolDir, angle);
            newToolPos = IP(toolPos.x() + newToolDir.x() * stepScaled, toolPos.y() + newToolDir.y() * stepScaled);

            // Skip iteration if this IntPoint has already been processed
            bool intRepeat = false;
            if (interp.m_min && SameXY(newToolPos, interp.m_min->angle.second)) {
                interp.m_min = InterpItem{{angle, newToolPos}, interp.m_min->error, interp.m_min->isConventional};
                intRepeat    = true;
            }
            if (interp.m_max && SameXY(newToolPos, interp.m_max->angle.second)) {
                interp.m_max = InterpItem{{angle, newToolPos}, interp.m_max->error, interp.m_max->isConventional};
                intRepeat    = true;
            }

            if (intRepeat) {
                if (interp.m_min && interp.m_max && std::abs(interp.m_min->angle.second.x() - interp.m_max->angle.second.x()) <= 1 &&
                    std::abs(interp.m_min->angle.second.y() - interp.m_max->angle.second.y()) <= 1) {
                    if (pointNotInterp)
                        continue; // only exit early if interpolation is down to adjacent integers
                    // exit early, selecting the better of the two adjacent integers
                    const InterpItem* best;
                    if (interp.m_min->isConventional ^ interp.m_max->isConventional)
                        best = !interp.m_min->isConventional ? &*interp.m_min : &*interp.m_max;
                    else
                        best = std::abs(interp.m_min->error) < std::abs(interp.m_max->error) ? &*interp.m_min : &*interp.m_max;
                    newToolDir         = rotate(toolDir, best->angle.first);
                    newToolPos         = best->angle.second;
                    isConventional     = best->isConventional;
                    areaPD             = best->error + targetAreaPD;
                    area               = areaPD * double(stepScaled);
                    out.iterationAngle = angle;
                    break;
                }
                continue;
            }

            const auto caRet            = CalcCutArea(toolPos, newToolPos, cleared);
            area                        = caRet.first;
            double fractionConventional = (area == 0) ? 0 : caRet.second / area;
            isConventional              = fractionConventional >= conventionalCutoff;
            if (area > 0)
                foundArea = true;

            areaPD        = area / double(stepScaled); // area per distance
            double error  = areaPD - targetAreaPD;
            errorFraction = std::abs(error / optimalCutAreaPD);
            interp.addPoint(error, {angle, newToolPos}, pointNotInterp, isConventional);
            if (fabs(error) < maxError && !isConventional) {
                out.iterationAngle = angle;
                break;
            }
        }

        if (area > 0) {
            //**********************************************
            // CHECK AND RECORD NEW TOOL POS
            //**********************************************
            bool   recalcArea = false;
            long   rotateStep = 0;
            double rotateIncrement;
            {
                double boundaryAngle = atan2(boundaryDir.y(), boundaryDir.x());
                double toolAngle     = atan2(newToolDir.y(), newToolDir.x());
                double delta         = boundaryAngle - toolAngle;
                if (delta > PI)
                    delta -= 2 * PI;
                if (delta < -PI)
                    delta += 2 * PI;
                rotateIncrement = (delta > 0 ? 1 : -1) * PI / 90;
            }
            while (!IsPointWithinCutRegion(toolBoundPaths, newToolPos) && rotateStep < 180) {
                rotateStep++;
                // if new tool pos. outside boundary rotate until back in
                recalcArea = true;
                newToolDir = rotate(newToolDir, rotateIncrement);
                newToolPos = IP(toolPos.x() + newToolDir.x() * stepScaled, toolPos.y() + newToolDir.y() * stepScaled);
            }
            if (rotateStep >= 180)
                out.failed = true;

            if (recalcArea) {
                const auto caRet = CalcCutArea(toolPos, newToolPos, cleared);
                area             = caRet.first;
                areaPD           = area / double(stepScaled);
                errorFraction    = std::abs((areaPD - targetAreaPD) / optimalCutAreaPD);
                double fractionConventional = area == 0 ? 0 : caRet.second / area;
                isConventional              = fractionConventional >= conventionalCutoff;
            }

            // safety condition
            if (area > stepScaled * optimalCutAreaPD && areaPD > 2 * optimalCutAreaPD)
                out.failed = true;
        }

        out.area          = area;
        out.failed        = out.failed || isConventional || area < 1;
        out.newToolPos    = newToolPos;
        out.newToolDir    = newToolDir;
        out.errorFraction = errorFraction;
        return out;
    };

    const auto initToolDir = [&](const IntPoint& toolPos, const DoublePoint& baseDir) -> std::optional<DoublePoint> {
        const DoublePoint testDirs[] = {{baseDir.x(), baseDir.y()}, {-baseDir.y(), baseDir.x()}, {-baseDir.x(), -baseDir.y()}, {baseDir.y(), -baseDir.x()}};
        std::optional<std::pair<DoublePoint, double>> bestDir;
        bool                                          allZero = true;
        for (const auto& testDir : testDirs) {
            const auto itResult = iterateNextStep(toolPos, testDir);
            if (itResult.area != 0)
                allZero = false;
            if (!itResult.failed && (!bestDir || itResult.errorFraction < bestDir->second))
                bestDir = {itResult.newToolDir, itResult.errorFraction};
        }

        if (bestDir)
            return bestDir->first;
        if (allZero) {
            const Paths& clearedArea = cleared.GetCleared();
            if (DistancePointToPathsSqrd(clearedArea, toolPos, clp, clpPathIndex, clpSegmentIndex, clpParameter) <
                double(toolRadiusScaled) * double(toolRadiusScaled)) {
                IntPoint p2 = Compute2DPolygonCentroid(clearedArea[clpPathIndex]);
                return DirectionV(toolPos, p2);
            }
        }
        return {};
    };

    using EngageResult = std::optional<std::tuple<IntPoint, DoublePoint, TPaths>>;

    // window > 0: only consider engage points closer than `window` to prevPos, computing the
    // engage offsets on the cleared area clipped around prevPos (see getEngagePoint).
    const auto _getEngagePoint = [&](const std::optional<IntPoint>& prevPos, cInt engagementProtrusion,
                                     double window) -> std::pair<EngageResult, double> {
        // engagePoint, engageDir, heuristicCost
        std::vector<std::tuple<IntPoint, DoublePoint, double>> engagePoints;

        const auto addEngagePoint = [&](const IntPoint& engagePoint, const DoublePoint& engageDir) {
            double cost = prevPos ? sqrt(DistanceSqrd(*prevPos, engagePoint)) : 0;
            engagePoints.emplace_back(engagePoint, engageDir, cost);
        };

        // offset inward to find places the tool can start
        const cInt engageBuffer = 2; // smooths out the integer rounding in the two offsets
        Paths      preEngage;
        clipof.Clear();
        if (window > 0) {
            // erosion + dilation only see geometry within (r + 2) + (2 + protrusion) of a point,
            // so inside box(prevPos, window) the result equals the unclipped one
            const cInt h = cInt(window) + toolRadiusScaled + 2 * engageBuffer + engagementProtrusion + 8;
            const Clipper2Lib_Z::Rect64 box(prevPos->x() - h, prevPos->y() - h, prevPos->x() + h, prevPos->y() + h);
            clipof.AddPaths(FromC2(Clipper2Lib_Z::RectClip(box, ToC2(cleared.GetCleared()))), jtRound, etClosedPolygon);
        } else
            clipof.AddPaths(cleared.GetCleared(), jtRound, etClosedPolygon);
        clipof.Execute(preEngage, -double(toolRadiusScaled + engageBuffer));

        // offset outward to find places that would protrude outside the cleared area
        Paths engagePaths;
        clipof.Clear();
        clipof.AddPaths(preEngage, jtRound, etClosedPolygon);
        clipof.Execute(engagePaths, double(engageBuffer + engagementProtrusion));

        // clip engage candidates with tool bounds
        for (Path& engagePath : engagePaths) {
            // rotate the closed path so it starts with the closest point
            Path rotated;
            if (!prevPos) {
                rotated = engagePath;
            } else {
                size_t iClosest   = 0;
                double dsqClosest = DBL_MAX;
                for (size_t i = 0; i < engagePath.size(); i++) {
                    double dsq = DistanceSqrd(*prevPos, engagePath[i]);
                    if (dsq < dsqClosest) {
                        dsqClosest = dsq;
                        iClosest   = i;
                    }
                }
                for (size_t i = 0; i < engagePath.size(); i++)
                    rotated.push_back(engagePath[(i + iClosest) % engagePath.size()]);
            }

            // clip with tbpMinus instead of toolBoundPaths to ensure that all resulting points are
            // inside toolBoundPaths, and not rounded to an integer coordinate outside of it
            Paths openPaths = PathIntersectArea(clip, rotated, tbpMinus);

            for (Path& open : openPaths) {
                bool   added = false;
                double dToGo = 0; // first step is 0 -- start point
                size_t seg   = 0;
                double segD  = 0;
                while (!added && seg + 1 < open.size()) {
                    // step to next point
                    IntPoint    p(0, 0, 0);
                    DoublePoint segDir(0, 0);
                    do {
                        const IntPoint& p1     = open[seg];
                        const IntPoint& p2     = open[seg + 1];
                        double          segLen = sqrt(DistanceSqrd(p1, p2));
                        segDir                 = DoublePoint((p2.x() - p1.x()) / segLen, (p2.y() - p1.y()) / segLen);
                        if (segLen - segD > dToGo) {
                            // interpolate current segment
                            segD += dToGo;
                            dToGo         = 0;
                            double interp = segD / segLen;
                            p             = IP(p2.x() * interp + p1.x() * (1 - interp), p2.y() * interp + p1.y() * (1 - interp));
                        } else {
                            dToGo -= segLen - segD;
                            segD = 0;
                            seg++;
                            p = p2; // ensures that we try the endpoint too
                        }
                    } while (dToGo > 0 && seg + 1 < open.size());

                    // Attempt to add the point
                    if (window > 0 && DistanceSqrd(*prevPos, p) >= window * window) {
                        // outside the exact zone of a windowed search: skip, keep walking
                    } else if (const auto dir = initToolDir(p, segDir)) {
                        addEngagePoint(p, *dir);
                        added = true;
                    }
                    dToGo = MIN_STEP_CLIPPER; // all subsequent steps are MIN_STEP_CLIPPER
                }
            }
        }

        // sort engagePoints based on connection cost (stable: deterministic ties)
        std::stable_sort(engagePoints.begin(), engagePoints.end(),
                         [](const auto& aa, const auto& bb) { return std::get<double>(aa) < std::get<double>(bb); });

        double      bestCost = DBL_MAX;
        TPaths      bestLink;
        IntPoint    bestPos(0, 0, 0);
        DoublePoint bestDir(0, 0);
        const double notClearPenalty = 10000 * scaleFactor; // prioritize links that don't require retraction

        for (const auto& ep : engagePoints) {
            if (std::get<double>(ep) >= bestCost)
                continue;
            std::optional<TPaths> link = FindLinkPath(prevPos, std::get<IntPoint>(ep), std::get<DoublePoint>(ep), cleared, toolBoundPaths);
            if (!link)
                continue;

            double                  cost = 0;
            std::optional<IntPoint> prev = prevPos;
            for (const TPath& tp : *link) {
                if (tp.first == mtLinkNotClear)
                    cost += notClearPenalty;
                for (const IntPoint& cur : tp.second) {
                    if (prev)
                        cost += sqrt(DistanceSqrd(cur, *prev));
                    prev = cur;
                }
            }

            if (cost < bestCost) {
                bestCost = cost;
                bestLink = *link;
                bestPos  = std::get<IntPoint>(ep);
                bestDir  = std::get<DoublePoint>(ep);
            }
        }

        if (bestCost < DBL_MAX)
            return {std::make_tuple(bestPos, bestDir, bestLink), bestCost};
        return {EngageResult{}, bestCost};
    };

    const auto getEngagePoint = [&](const std::optional<IntPoint>& prevPos) {
        // Compute how far into the material the first engagement should be.
        // Area of a circle segment A = R^2/2 (theta - sin theta) ~= R^2 theta^3 / 12
        const double targetArea = optimalCutAreaPD * MIN_STEP_CLIPPER;
        const double R          = double(toolRadiusScaled);
        const double theta      = std::pow(12 * targetArea / R / R, 1 / 3.);
        const double protrusion = R - cos(theta / 2) * R;
        const cInt   engagementProtrusion = cInt(std::min(protrusion, stepOverScaled * FINISHING_THICKNESS_SCALE));

        // The original offset the whole cleared area twice per pass. Try a window
        // around the previous position first: a link's cost is at least the straight distance
        // to its engage point, so a windowed best cost below the window size cannot be beaten by
        // a point outside it. Otherwise (nothing near, or only a retracting link) search globally.
        EngageResult result;
        const double window = 6. * toolRadiusScaled;
        if (prevPos) {
            auto [res, cost] = _getEngagePoint(prevPos, engagementProtrusion, window);
            if (res && cost < window)
                result = std::move(res);
        }
        if (!result)
            result = _getEngagePoint(prevPos, engagementProtrusion, 0).first;

        // update cleared area
        if (result)
            for (const TPath& linkPath : std::get<TPaths>(*result))
                if (linkPath.first == mtCutting)
                    cleared.ExpandCleared(linkPath.second);
        return result;
    };

    EngageResult engagePoint = getEngagePoint({});
    TPaths       linkPath;
    if (engagePoint) {
        toolPos    = std::get<IntPoint>(*engagePoint);
        toolDir    = std::get<DoublePoint>(*engagePoint);
        linkPath   = std::get<TPaths>(*engagePoint);
        entryPoint = !linkPath.empty() ? linkPath[0].second[0] : toolPos;
        output.StartPoint = entryPoint;
    } else {
        // Engagement failed; instead helix down
        if (!FindEntryPoint(toolBoundPaths, boundPaths, cleared, entryPoint, toolPos, toolDir, helixRadiusScaled, output)) {
            results.push_back(std::move(output));
            return;
        }
        output.StartPoint = toolPos;
    }

    output.HelixCenterPoint = entryPoint;

    //*******************************
    // LOOP - PASSES
    //*******************************
    while (!stopProcessing) {
        passToolPath.clear();
        toClearPath.clear();
        angleHistory.clear();
        angleHistory.push_back(0);

        // include linking path in cleared area
        for (const TPath& lp : linkPath)
            if (lp.first == mtCutting || lp.first == mtLinkClear)
                cleared.ExpandCleared(lp.second);

        angle = PI / 4; // initial pass angle
        // init gyro
        gyro.assign(DIRECTION_SMOOTHING_BUFLEN, toolDir);

        //*******************************
        // LOOP - POINTS
        //*******************************
        while (!stopProcessing) {
            AverageDirection(gyro, toolDir);

            const auto itResult = iterateNextStep(toolPos, toolDir);
            if (itResult.failed)
                break;

            // cut is ok - record it
            if (itResult.iterationAngle) {
                angleHistory.push_back(*itResult.iterationAngle);
                if (angleHistory.size() > ANGLE_HISTORY_POINTS)
                    angleHistory.erase(angleHistory.begin());
            }

            // if the path has changed direction by more than 45 degrees then we need to update
            // cleared paths
            if (lastExpandToolDir.x() * itResult.newToolDir.x() + lastExpandToolDir.y() * itResult.newToolDir.y() < cos(PI / 4)) {
                cleared.ExpandCleared(toClearPath);
                toClearPath.clear();
                lastExpandToolDir = toolDir;
            }

            if (toClearPath.empty())
                toClearPath.push_back(toolPos);
            toClearPath.push_back(itResult.newToolPos);

            // append to toolpaths
            if (passToolPath.empty())
                passToolPath.push_back(toolPos);
            passToolPath.push_back(itResult.newToolPos);
            toolPos = itResult.newToolPos;

            gyro.push_back(itResult.newToolDir);
            gyro.erase(gyro.begin());

            areaCut += itResult.area;
            Poll();
        } /* end of points loop*/

        if (!toClearPath.empty()) {
            cleared.ExpandCleared(toClearPath);
            toClearPath.clear();
        }

        Paths newlyClearedAreas;
        clip.Clear();
        clip.AddPaths(cleared.GetCleared(), ptSubject, true);
        clip.AddPaths(clearedBeforePass.GetCleared(), ptClip, true);
        clip.Execute(ctDifference, newlyClearedAreas);
        double cumulativeCutArea = NestedArea(newlyClearedAreas);

        if (cumulativeCutArea >= 1) {
            Path cleaned;
            CleanPath(passToolPath, cleaned, CLEAN_PATH_TOLERANCE);
            if (auto newPos = AppendToolPath(output, cleaned, linkPath, cleared, toolBoundPaths)) {
                toolPos = newPos->first;
                toolDir = newPos->second;
            }
            bad_engage_count = 0;
        } else if (++bad_engage_count > 10000) {
            break; // next valid engage point not found
        }

        clearedBeforePass.SetClearedPaths(cleared.GetCleared());
        engagePoint = getEngagePoint(toolPos);
        if (engagePoint) {
            toolPos           = std::get<IntPoint>(*engagePoint);
            toolDir           = std::get<DoublePoint>(*engagePoint);
            linkPath          = std::get<TPaths>(*engagePoint);
            lastExpandToolDir = toolDir;
        } else {
            // check if there are any uncleared area left
            bool remaining = false;
            for (const auto& p : cleared.GetCleared())
                if (!p.empty() && IsPointWithinCutRegion(toolBoundPaths, p.front()) &&
                    DistancePointToPathsSqrd(boundPaths, p.front(), clp, clpPathIndex, clpSegmentIndex, clpParameter) >
                        4. * double(toolRadiusScaled) * double(toolRadiusScaled))
                    remaining = true;
            output.UnclearedAreaRemains = remaining;
            break;
        }
    }

    // sanity check for finishing paths - check the area of finishing cut
    {
        Paths clearedLocations;
        clipof.Clear();
        clipof.AddPaths(cleared.GetCleared(), jtRound, etClosedPolygon);
        clipof.Execute(clearedLocations, -double(toolRadiusScaled));

        Paths tbpShrink;
        clipof.Clear();
        clipof.AddPaths(toolBoundPaths, jtRound, etClosedPolygon);
        clipof.Execute(tbpShrink, -(stepOverScaled * FINISHING_THICKNESS_SCALE + MIN_STEP_CLIPPER));

        Paths uncut;
        clip.Clear();
        clip.AddPaths(tbpShrink, ptSubject, true);
        clip.AddPaths(clearedLocations, ptClip, true);
        clip.Execute(ctDifference, uncut);
        output.FailedToSetUpFinishingPass = !uncut.empty();
    }

    //**********************************
    //*  FINISHING PASS                *
    //**********************************
    if (finishingProfile) {
        // update tool bound paths to correspond to the finishing paths instead of the interior
        {
            Paths tbpModified;
            for (const Path& fp : finishingPaths) {
                clipof.Clear();
                clipof.AddPath(fp, jtRound, etClosedPolygon);
                int   offset = (getPathNestingLevel(fp, finishingPaths) % 2 == 1) ? 3 : -3;
                Paths out;
                clipof.Execute(out, offset);
                bool orientation = Orientation(fp);
                for (Path& p : out) {
                    if (Orientation(p) != orientation)
                        ReversePath(p);
                    tbpModified.push_back(p);
                }
            }
            toolBoundPaths = tbpModified;
        }

        // Split finishingPaths into closed (all Z=1) and open (partial Z=1) paths
        Paths closedFinishingPaths;
        Paths openFinishingPaths;
        for (const auto& p : finishingPaths) {
            // Find starting index: first Z=0, or 0 if none found
            size_t startIdx = 0;
            for (size_t i = 0; i < p.size(); i++)
                if (p[i].z() == 0) {
                    startIdx = i;
                    break;
                }
            Path currentPath;
            for (size_t offset = 0; offset < p.size(); offset++) {
                const IntPoint& pt = p[(startIdx + offset) % p.size()];
                if (pt.z() == 1) {
                    currentPath.push_back(pt);
                } else if (!currentPath.empty()) {
                    openFinishingPaths.push_back(currentPath);
                    currentPath.clear();
                }
            }
            if (currentPath.size() == p.size())
                closedFinishingPaths.emplace_back(p);
            else if (!currentPath.empty())
                openFinishingPaths.push_back(currentPath);
        }

        // stock boundary grown by the tool radius: finishing passes outside of it cut nothing
        Paths stockExpandedPaths;
        clipof.Clear();
        clipof.AddPaths(stockInputPaths, jtRound, etClosedPolygon);
        clipof.Execute(stockExpandedPaths, double(toolRadiusScaled));

        Path finShiftedPath;
        while (!stopProcessing && (!closedFinishingPaths.empty() || !openFinishingPaths.empty())) {
            bool isClosedPath = PopNextFinishingPass(closedFinishingPaths, openFinishingPaths, toolPos, finShiftedPath, stepOverScaled);
            if (finShiftedPath.empty())
                continue;
            // skip finishing passes outside the stock boundary that do not cut any stock
            Path finShiftedPathCopy = finShiftedPath;
            if (PathIntersectArea(clip, finShiftedPathCopy, stockExpandedPaths, isClosedPath).empty())
                continue;

            Path finCleaned;
            CleanPath(finShiftedPath, finCleaned, FINISHING_CLEAN_PATH_TOLERANCE);

            // Ensure correct endpoint after cleaning
            IntPoint endPoint = isClosedPath ? finCleaned.front() : finShiftedPath.back();
            if (!SameXY(finCleaned.back(), endPoint)) {
                // don't risk ruining the final direction by making a very short segment
                if (sqrt(DistanceSqrd(endPoint, finCleaned.back())) < FINISHING_CLEAN_PATH_TOLERANCE)
                    finCleaned.pop_back();
                finCleaned.push_back(endPoint);
            }

            std::optional<TPaths> finLink = FindLinkPath(toolPos, finCleaned[0], GetPathDirectionV(finCleaned, 1), cleared, toolBoundPaths);
            if (!finLink) {
                output.FinishingLeadInFailed = true;
                continue;
            }
            if (auto newPos = AppendToolPath(output, finCleaned, *finLink, cleared, toolBoundPaths)) {
                toolPos = newPos->first;
                toolDir = newPos->second;
            } else {
                toolPos = finCleaned.back();
                toolDir = GetPathDirectionV(finCleaned, finCleaned.size() - 1);
            }
            cleared.ExpandCleared(finCleaned);
            for (const TPath& lp : *finLink)
                if (lp.first == mtCutting)
                    cleared.ExpandCleared(lp.second);
            Poll();
        }
    }
    // (the original set this only with a finishing pass and returned "cutting" otherwise)
    output.ReturnMotionType = IsClearPath(Path{toolPos, entryPoint}, cleared) ? mtLinkClear : mtLinkNotClear;

    output.Cleared = cleared.GetCleared();
    results.push_back(std::move(output));
}

} // namespace AP
} // namespace

namespace Slic3r::CAM::Adaptive {

namespace {

MotionType to_public(AP::MotionType t)
{
    switch (t) {
    case AP::mtLinkClear: return MotionType::LinkClear;
    case AP::mtLinkNotClear: return MotionType::LinkNotClear;
    default: return MotionType::Cutting;
    }
}

} // namespace

Result clear(const ExPolygons& region, const ExPolygons& keep_out, const Params& params, const ExPolygons& already_cleared)
{
    Result res;
    if (params.tool_diameter <= 0 || params.stepover_fraction <= 0) {
        res.error = "Tool diameter and stepover must be greater than zero.";
        return res;
    }

    // Stock to leave, applied like the original ApplyStockToLeave (round joins): pockets shrink the
    // whole area to clear; with air outside, only the keep-out grows.
    const float stl = float(scale_(std::max(0., params.stock_to_leave)));
    ExPolygons  area;
    if (params.outside_is_air)
        area = diff_ex(region, stl > 0 ? offset_ex(keep_out, stl, ClipperLib::jtRound) : keep_out);
    else {
        area = diff_ex(region, keep_out);
        if (stl > 0)
            area = offset_ex(area, -stl, ClipperLib::jtRound);
    }
    if (area.empty()) {
        res.error = "Nothing to clear: the selected area is empty.";
        return res;
    }

    AP::Adaptive2d a2d;
    a2d.toolDiameter            = params.tool_diameter;
    a2d.stepOverFactor          = params.stepover_fraction;
    a2d.helixRampTargetDiameter = params.helix_diameter;
    // Params::tolerance (mm) -> FreeCAD's accuracy knob: accuracy = 10 x tolerance, i.e. the
    // integer grid is ~tolerance / 4.8 (for stepovers >= 1 mm) and the minimum step ~10 x tolerance.
    a2d.tolerance             = params.tolerance * 10.;
    a2d.forceInsideOut        = params.force_inside_out;
    a2d.finishingProfile      = params.finishing_profile;
    a2d.keepToolDownDistRatio = params.keep_tool_down_ratio;
    a2d.cancel                = params.cancel;
    a2d.Init();

    // Algorithm units: 1 mm = scaleFactor units (FreeCAD: MIN_STEP_CLIPPER / accuracy / min(1, stepover mm)).
    // Conventional milling mirrors X on the way in and out.
    const double sf = a2d.ScaleFactor();
    const double sx = params.climb ? 1. : -1.;
    const auto   to_algo = [&](const Polygons& polys) {
        AP::Paths out;
        for (const Polygon& poly : polys) {
            AP::Path p;
            p.reserve(poly.points.size());
            for (const Point& pt : poly.points)
                p.emplace_back(AP::cInt(std::llround(sx * unscale<double>(pt.x()) * sf)), AP::cInt(std::llround(unscale<double>(pt.y()) * sf)), 0);
            out.push_back(std::move(p));
        }
        return out;
    };
    const auto to_slic3r = [&](const AP::IntPoint& p) { return Point(scale_(sx * double(p.x()) / sf), scale_(double(p.y()) / sf)); };

    const AP::Paths stock   = to_algo(to_polygons(params.outside_is_air ? region : area));
    const AP::Paths input   = to_algo(to_polygons(area));
    const AP::Paths cleared = to_algo(to_polygons(already_cleared));

    std::vector<AP::AdaptiveOutput> outputs = a2d.Execute(stock, input, cleared, params.outside_is_air && !params.force_inside_out,
                                                          !already_cleared.empty());

    Polygons cleared_polys;
    bool     any_uncleared = false, any_overload = false, any_lead_in_failed = false, any_start_failed = false;
    for (const AP::AdaptiveOutput& out : outputs) {
        any_start_failed |= out.StartPointNotFound;
        if (out.AdaptivePaths.empty())
            continue;
        any_uncleared |= out.UnclearedAreaRemains;
        any_overload |= out.FailedToSetUpFinishingPass;
        any_lead_in_failed |= out.FinishingLeadInFailed;

        Entry e;
        e.center     = to_slic3r(out.HelixCenterPoint);
        e.start      = to_slic3r(out.StartPoint);
        e.path_index = res.paths.size();
        e.return_type = to_public(out.ReturnMotionType);
        res.entries.push_back(e);
        res.helix_radius = std::max(res.helix_radius, sqrt(AP::DistanceSqrd(out.HelixCenterPoint, out.StartPoint)) / sf);

        for (const AP::TPath& tp : out.AdaptivePaths) {
            if (tp.second.empty())
                continue;
            Path path;
            path.type = to_public(tp.first);
            path.pts.points.reserve(tp.second.size());
            for (const AP::IntPoint& p : tp.second)
                path.pts.points.push_back(to_slic3r(p));
            res.paths.push_back(std::move(path));
        }
        for (const AP::Path& p : out.Cleared) {
            Polygon poly;
            poly.points.reserve(p.size());
            for (const AP::IntPoint& pt : p)
                poly.points.push_back(to_slic3r(pt));
            cleared_polys.push_back(std::move(poly));
        }
    }

    if (a2d.Cancelled()) {
        res.error = "Cancelled.";
        return res;
    }
    if (res.paths.empty()) {
        if (any_start_failed || outputs.empty()) {
            char buf[160];
            snprintf(buf, sizeof(buf), "The %g mm tool is too large to enter this area. Use a smaller tool or check the selection.",
                     params.tool_diameter);
            res.error = buf;
        } else
            res.error = "Nothing to clear: the area is already cleared.";
        return res;
    }

    // the swept area, limited to the stock grown by the tool radius (drops the air border)
    const ExPolygons stock_limit = offset_ex(params.outside_is_air ? region : area, float(scale_(params.tool_diameter / 2)) + 10.f);
    res.cleared                  = intersection_ex(union_ex(cleared_polys), stock_limit);

    res.helix_center = res.entries.front().center;
    if (any_start_failed)
        res.warnings.emplace_back("Some areas are too small for the tool to enter and were skipped.");
    if (any_uncleared)
        res.warnings.emplace_back("Some material could not be reached without exceeding the optimal load and was left uncut.");
    if (any_overload)
        res.warnings.emplace_back("Some cuts may be above the optimal load. Try a finer tolerance or a smaller stepover.");
    if (any_lead_in_failed)
        res.warnings.emplace_back("A finishing pass could not be entered safely and was skipped.");
    res.ok = true;
    return res;
}

} // namespace Slic3r::CAM::Adaptive
