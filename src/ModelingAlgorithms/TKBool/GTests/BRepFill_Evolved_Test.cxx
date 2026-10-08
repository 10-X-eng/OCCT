// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFill_Evolved.hxx>
#include <Standard_ConstructionError.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>

#include <gtest/gtest.h>
#include <cmath>

namespace
{
// Same three-edge profile used by LocOpe_DPrism for a tapered extrusion.
void makeTaper(const double theRadius, TopoDS_Face& theSpine, TopoDS_Wire& theProfile)
{
  const TopoDS_Edge anEdge = BRepBuilderAPI_MakeEdge(gp_Circ(gp::XOY(), theRadius));
  const TopoDS_Wire aWire  = BRepBuilderAPI_MakeWire(anEdge);
  theSpine                 = BRepBuilderAPI_MakeFace(gp_Pln(gp::XOY()), aWire, true);
  const double aHeight     = 4.0 * theRadius;
  const double aY          = -aHeight * std::tan(M_PI / 180.0);
  const double aDelta      = 2.0 * (2.0 * theRadius + std::abs(aY));
  const gp_Pnt aP1(0.0, 0.0, 0.0), aP2(0.0, aY, aHeight);
  const gp_Pnt aP3(0.0, aY + aDelta, aHeight), aP4(0.0, aDelta, 0.0);
  theProfile = BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(aP4, aP1),
                                       BRepBuilderAPI_MakeEdge(aP1, aP2),
                                       BRepBuilderAPI_MakeEdge(aP2, aP3));
}
} // namespace

TEST(BRepFill_Evolved_Test, InconsistentGeneratedEdgesRaiseInsteadOfCrashing)
{
  // CadQuery #1873: the profile is valid, but the offset loses a generated
  // edge at this scale. The two lists used for gluing have sizes one and zero.
  TopoDS_Face aSpine;
  TopoDS_Wire aProfile;
  makeTaper(0.0268 * 0.0268 / 2.0, aSpine, aProfile);
  ASSERT_TRUE(BRepCheck_Analyzer(aSpine).IsValid());
  ASSERT_TRUE(BRepCheck_Analyzer(aProfile).IsValid());
  BRepFill_Evolved anEvolved;
  EXPECT_THROW(anEvolved.Perform(aSpine, aProfile, gp::XOY()), Standard_ConstructionError);
  EXPECT_FALSE(anEvolved.IsDone());
}

TEST(BRepFill_Evolved_Test, FailedReuseClearsDoneAndCanRecover)
{
  TopoDS_Face aSpine, aSmallSpine;
  TopoDS_Wire aProfile, aSmallProfile;
  makeTaper(1.0, aSpine, aProfile);
  makeTaper(0.0268 * 0.0268 / 2.0, aSmallSpine, aSmallProfile);
  BRepFill_Evolved anEvolved;
  anEvolved.Perform(aSpine, aProfile, gp::XOY());
  ASSERT_TRUE(anEvolved.IsDone());
  ASSERT_FALSE(anEvolved.Shape().IsNull());
  ASSERT_TRUE(BRepCheck_Analyzer(anEvolved.Shape()).IsValid());
  EXPECT_THROW(anEvolved.Perform(aSmallSpine, aSmallProfile, gp::XOY()),
               Standard_ConstructionError);
  EXPECT_FALSE(anEvolved.IsDone());
  anEvolved.Perform(aSpine, aProfile, gp::XOY());
  EXPECT_TRUE(anEvolved.IsDone());
  EXPECT_TRUE(BRepCheck_Analyzer(anEvolved.Shape()).IsValid());
}
