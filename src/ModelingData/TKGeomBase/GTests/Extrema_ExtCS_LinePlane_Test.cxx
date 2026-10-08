// Copyright (c) 2026 RobitOverlord
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <Extrema_ExtCS.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

#include <gtest/gtest.h>
#include <array>
#include <cmath>

namespace
{
struct LinePlane
{
  GeomAdaptor_Curve   Curve;
  GeomAdaptor_Surface Surface;
  gp_Pnt              Target;

  LinePlane(const gp_Pnt& theTarget, const gp_Dir& theDirection, const gp_Trsf& thePlacement)
      : Target(theTarget.Transformed(thePlacement))
  {
    const gp_Pnt anOrigin = theTarget.Translated(gp_Vec(theDirection) * -5.0);
    const gp_Lin aLine    = gp_Lin(anOrigin, theDirection).Transformed(thePlacement);
    const gp_Pln aPlane =
      gp_Pln(gp_Ax3(gp_Pnt(), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))).Transformed(thePlacement);
    Curve.Load(new Geom_Line(aLine), 0, 10);
    Surface.Load(new Geom_Plane(aPlane), -8, 8, -6, 6);
  }
};

std::array<gp_Trsf, 3> placements()
{
  std::array<gp_Trsf, 3> aResult;
  aResult[1].SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), 0.6457718232379019);
  aResult[1].SetTranslationPart(gp_Vec(7, -13, 4));
  aResult[2].SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(2, -1, 3)), 1.2740903539558606);
  aResult[2].SetTranslationPart(gp_Vec(1e6, -2e6, 3e6));
  return aResult;
}

void checkIntersection(LinePlane& thePair, const double theTolerance)
{
  Extrema_ExtCS anExtrema(thePair.Curve, thePair.Surface, 1e-9, 1e-9);
  ASSERT_TRUE(anExtrema.IsDone());
  ASSERT_FALSE(anExtrema.IsParallel());
  ASSERT_GE(anExtrema.NbExt(), 1);
  int anIndex = 1;
  for (int i = 2; i <= anExtrema.NbExt(); ++i)
    if (anExtrema.SquareDistance(i) < anExtrema.SquareDistance(anIndex))
      anIndex = i;
  Extrema_POnCurv aCurvePoint;
  Extrema_POnSurf aSurfacePoint;
  anExtrema.Points(anIndex, aCurvePoint, aSurfacePoint);
  EXPECT_NEAR(aCurvePoint.Parameter(), 5.0, theTolerance);
  EXPECT_LT(aCurvePoint.Value().Distance(thePair.Target), theTolerance);
  EXPECT_LT(aSurfacePoint.Value().Distance(thePair.Target), theTolerance);
  EXPECT_LT(aCurvePoint.Value().Distance(thePair.Curve.Value(aCurvePoint.Parameter())), 1e-9);
  EXPECT_LT(anExtrema.SquareDistance(anIndex), 1e-14);
}
} // namespace

TEST(Extrema_ExtCS_LinePlane, InteriorEdgesAndCorners)
{
  for (const auto& aPlacement : placements())
    for (const auto& aDirection : {gp_Dir(0, 0, 1), gp_Dir(1, 2, 3), gp_Dir(1, 1, -2)})
      for (const auto& aTarget : {gp_Pnt(), gp_Pnt(8, 0, 0), gp_Pnt(0, -6, 0), gp_Pnt(-8, 6, 0)})
      {
        LinePlane aPair(aTarget, aDirection, aPlacement);
        checkIntersection(aPair, 2e-8);
      }
}

TEST(Extrema_ExtCS_LinePlane, ShallowTransverseIntersection)
{
  for (const double aSlope : {1e-3, 1e-6, 1e-9, -1e-9})
  {
    SCOPED_TRACE(aSlope);
    LinePlane aPair(gp_Pnt(), gp_Dir(1, 0, aSlope), gp_Trsf());
    checkIntersection(aPair, 1e-7);
  }
}

TEST(Extrema_ExtCS_LinePlane, ParallelDistance)
{
  for (const auto& aPlacement : placements())
    for (const double aHeight : {0.0, 2.0})
    {
      LinePlane     aPair(gp_Pnt(0, 0, aHeight), gp_Dir(1, 0, 0), aPlacement);
      Extrema_ExtCS anExtrema(aPair.Curve, aPair.Surface, 1e-9, 1e-9);
      ASSERT_TRUE(anExtrema.IsDone());
      ASSERT_TRUE(anExtrema.IsParallel());
      EXPECT_NEAR(anExtrema.SquareDistance(1), aHeight * aHeight, 1e-8);
    }
}

TEST(Extrema_ExtCS_LinePlane, ExplicitTrimsExcludeIntersection)
{
  const std::array<std::array<double, 6>, 4> aBounds = {
    {{6, 10, -8, 8, -6, 6}, {0, 4, -8, 8, -6, 6}, {0, 10, 1, 8, -6, 6}, {0, 10, -8, 8, -6, -1}}};
  for (const auto& aB : aBounds)
  {
    LinePlane aPair(gp_Pnt(), gp_Dir(1, 1, 2), gp_Trsf());
    Extrema_ExtCS
      anExtrema(aPair.Curve, aPair.Surface, aB[0], aB[1], aB[2], aB[3], aB[4], aB[5], 1e-9, 1e-9);
    ASSERT_TRUE(anExtrema.IsDone());
    ASSERT_FALSE(anExtrema.IsParallel());
    for (int i = 1; i <= anExtrema.NbExt(); ++i)
    {
      Extrema_POnCurv aCurvePoint;
      Extrema_POnSurf aSurfacePoint;
      anExtrema.Points(i, aCurvePoint, aSurfacePoint);
      double aU, aV;
      aSurfacePoint.Parameter(aU, aV);
      EXPECT_GE(aCurvePoint.Parameter(), aB[0] - 1e-9);
      EXPECT_LE(aCurvePoint.Parameter(), aB[1] + 1e-9);
      EXPECT_GE(aU, aB[2] - 1e-9);
      EXPECT_LE(aU, aB[3] + 1e-9);
      EXPECT_GE(aV, aB[4] - 1e-9);
      EXPECT_LE(aV, aB[5] + 1e-9);
      EXPECT_TRUE(std::isfinite(anExtrema.SquareDistance(i)));
    }
  }
}
