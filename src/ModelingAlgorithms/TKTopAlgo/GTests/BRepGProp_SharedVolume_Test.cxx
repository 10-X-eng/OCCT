// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Trsf.hxx>
#include <gtest/gtest.h>

namespace
{
TopoDS_Compound compound(std::initializer_list<TopoDS_Shape> theShapes)
{
  BRep_Builder    aBuilder;
  TopoDS_Compound aResult;
  aBuilder.MakeCompound(aResult);
  for (const auto& aShape : theShapes)
    aBuilder.Add(aResult, aShape);
  return aResult;
}

void compareFaceIntegration(const TopoDS_Shape& theShape, bool theSkipShared)
{
  BRep_Builder    aBuilder;
  TopoDS_Compound aFaces;
  aBuilder.MakeCompound(aFaces);
  // A compound containing faces but no solids uses direct face integration.
  for (TopExp_Explorer anExplorer(theShape, TopAbs_FACE); anExplorer.More(); anExplorer.Next())
    aBuilder.Add(aFaces, anExplorer.Current());
  GProp_GProps anActual, anExpected;
  BRepGProp::VolumeProperties(theShape, anActual, false, theSkipShared, false);
  BRepGProp::VolumeProperties(aFaces, anExpected, false, theSkipShared, false);
  EXPECT_NEAR(anActual.Mass(), anExpected.Mass(), 1e-9);
  EXPECT_NEAR(anActual.CentreOfMass().Distance(anExpected.CentreOfMass()), 0., 1e-9);
  const gp_Mat anActualInertia   = anActual.MatrixOfInertia();
  const gp_Mat anExpectedInertia = anExpected.MatrixOfInertia();
  for (int row = 1; row <= 3; ++row)
    for (int col = 1; col <= 3; ++col)
      EXPECT_NEAR(anActualInertia.Value(row, col), anExpectedInertia.Value(row, col), 1e-8);
}
} // namespace

TEST(BRepGProp_SharedVolume_Test, RepeatedNonInitialPlacementIsSkipped)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(2., 3., 4.).Shape();
  gp_Trsf            aTranslation;
  aTranslation.SetTranslation(gp_Vec(10., 0., 0.));
  const TopoDS_Shape aMoved = aBox.Moved(TopLoc_Location(aTranslation));
  for (bool aSkip : {false, true})
    compareFaceIntegration(compound({aBox, aMoved, aMoved}), aSkip);
}

TEST(BRepGProp_SharedVolume_Test, InternalAndExternalSolidsHaveNoVolumeContribution)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(2., 3., 4.).Shape();
  for (TopAbs_Orientation anOrientation : {TopAbs_INTERNAL, TopAbs_EXTERNAL})
  {
    const TopoDS_Shape anIgnored = aBox.Oriented(anOrientation);
    for (bool aSkip : {false, true})
    {
      compareFaceIntegration(compound({anIgnored, aBox}), aSkip);
      compareFaceIntegration(compound({aBox, anIgnored}), aSkip);
    }
  }
}

TEST(BRepGProp_SharedVolume_Test, SharedFacesAcrossDifferentSolidsAreSkipped)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(2., 3., 4.).Shape();
  BRep_Builder       aBuilder;
  TopoDS_Solid       aSecond;
  aBuilder.MakeSolid(aSecond);
  for (TopExp_Explorer anExplorer(aBox, TopAbs_SHELL); anExplorer.More(); anExplorer.Next())
    aBuilder.Add(aSecond, anExplorer.Current());
  compareFaceIntegration(compound({aBox, aBox, aSecond}), true);
}

TEST(BRepGProp_SharedVolume_Test, SharedFreeFacesAreSkipped)
{
  const TopoDS_Shape aBox = BRepPrimAPI_MakeBox(2., 3., 4.).Shape();
  TopExp_Explorer    aFace(aBox, TopAbs_FACE);
  compareFaceIntegration(compound({aBox, aBox, aFace.Current()}), true);
}
