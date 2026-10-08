// Copyright (c) 2025 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#include <BSplCLib.hxx>
#include <gtest/gtest.h>

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

namespace
{
// Reference form of the original divided-difference algorithm. Keep the
// scalar division distinct from the reciprocal multiplication in dimensions
// greater than one, and compare every output bit (not only the requested N).
void referenceBohm(double  theU,
                   int     theDegree,
                   int     theN,
                   double* theKnots,
                   int     theDimension,
                   double* thePoles)
{
  for (int i = 0; i < theDegree; ++i)
  {
    for (int j = theDegree - 1; j >= i; --j)
    {
      const double aLeft  = theKnots[j];
      const double aRight = theKnots[j + theDegree - i];
      const double aCoef  = (aLeft == aRight) ? 0.0 : 1.0 / (aRight - aLeft);
      for (int k = 0; k < theDimension; ++k)
      {
        double& aPole = thePoles[(j + 1) * theDimension + k];
        aPole -= thePoles[j * theDimension + k];
        aPole =
          theDimension == 1 ? ((aLeft == aRight) ? 0.0 : aPole / (aRight - aLeft)) : aPole * aCoef;
      }
    }
  }
  for (int i = 0; i < theDegree; ++i)
  {
    const double aCoef = theU - theKnots[i];
    for (int j = i; j >= 0; --j)
    {
      for (int k = 0; k < theDimension; ++k)
      {
        thePoles[j * theDimension + k] += aCoef * thePoles[(j + 1) * theDimension + k];
      }
    }
  }
  double aCoef      = theDegree;
  int    aRemaining = theDegree;
  for (int i = 1; i <= std::min(theN, theDegree); ++i)
  {
    for (int k = 0; k < theDimension; ++k)
    {
      thePoles[i * theDimension + k] *= aCoef;
    }
    aCoef *= --aRemaining;
  }
}

void compareBohm(const double               theU,
                 const int                  theDegree,
                 const int                  theN,
                 std::vector<double>        theKnots,
                 const int                  theDimension,
                 const std::vector<double>& thePoles)
{
  auto anExpected = thePoles;
  auto anActual   = thePoles;
  referenceBohm(theU, theDegree, theN, theKnots.data(), theDimension, anExpected.data());
  BSplCLib::Bohm(theU, theDegree, theN, theKnots.front(), theDimension, anActual.front());
  ASSERT_EQ(std::memcmp(anExpected.data(), anActual.data(), anActual.size() * sizeof(double)), 0)
    << "degree=" << theDegree << " dimension=" << theDimension << " N=" << theN;
}

void checkMatrix()
{
  std::mt19937 aRandom(1037);
  for (int aDegree = 1; aDegree <= 25; ++aDegree)
  {
    SCOPED_TRACE(aDegree);
    for (const int aDimension : {1, 2, 3, 4, 5, 8})
    {
      std::vector<double> aPoles((aDegree + 1) * aDimension);
      for (double& aPole : aPoles)
      {
        aPole = (static_cast<int>(aRandom() % 2049) - 1024) / 32.0;
      }
      for (int aKind = 0; aKind < 5; ++aKind)
      {
        SCOPED_TRACE(aKind);
        std::vector<double> aKnots(2 * aDegree);
        for (int i = 0; i < 2 * aDegree; ++i)
        {
          switch (aKind)
          {
            case 0:
              aKnots[i] = i / 4.0;
              break;
            case 1:
              aKnots[i] = i < aDegree ? 0.0 : (i == 2 * aDegree - 1 ? 2.0 : 1.0);
              break;
            case 2:
              aKnots[i] = (i ? aKnots[i - 1] : -2.0) + (aRandom() % 4) / 8.0;
              break;
            case 3:
              aKnots[i] = i < aDegree ? -0.0 : 0.0;
              break;
            case 4:
              aKnots[i] = i ? std::nextafter(aKnots[i - 1], 2.0) : 1.0;
              break;
          }
        }
        for (const int anOrder : {0, 1, 2, aDegree, aDegree + 2})
        {
          compareBohm(aKnots[aDegree - 1], aDegree, anOrder, aKnots, aDimension, aPoles);
          compareBohm((aKnots[aDegree - 1] + aKnots[aDegree]) / 2.0,
                      aDegree,
                      anOrder,
                      aKnots,
                      aDimension,
                      aPoles);
          compareBohm(aKnots[aDegree], aDegree, anOrder, aKnots, aDimension, aPoles);
        }
      }
    }
  }
}
} // namespace

TEST(BSplCLib_Bohm_Test, MatchesOriginalArithmetic)
{
  checkMatrix();
}

TEST(BSplCLib_Bohm_Test, RoundingModes)
{
  const int aSavedMode = std::fegetround();
  for (const int aMode : {FE_DOWNWARD, FE_UPWARD, FE_TOWARDZERO})
  {
    ASSERT_EQ(std::fesetround(aMode), 0);
    SCOPED_TRACE(aMode);
    checkMatrix();
  }
  EXPECT_EQ(std::fesetround(aSavedMode), 0);
}

TEST(BSplCLib_Bohm_Test, ExtremeKnotScales)
{
  for (const double aScale : {1e-150, 1e-50, 1.0, 1e50, 1e150})
  {
    for (const int aDimension : {2, 3, 4, 7})
    {
      for (const int aDegree : {1, 3, 8, 11, 25})
      {
        std::vector<double> aKnots(2 * aDegree);
        std::fill(aKnots.begin() + aDegree, aKnots.end(), aScale);
        std::vector<double> aPoles((aDegree + 1) * aDimension);
        for (size_t i = 0; i < aPoles.size(); ++i)
          aPoles[i] = (i % 7) / 8.0;
        compareBohm(aScale * 0.37, aDegree, aDegree, aKnots, aDimension, aPoles);
      }
    }
  }
}
