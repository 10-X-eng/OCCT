// Copyright (c) 2026 RobitOverlord
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_TriangulationParameters.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax1.hxx>
#include <gp_GTrsf.hxx>

#include <gtest/gtest.h>
#include <cmath>

namespace
{
TopoDS_Face meshOnlyFace()
{
  occ::handle<Poly_Triangulation> aMesh = new Poly_Triangulation(3, 1, false);
  aMesh->SetNode(1, gp_Pnt(0, 0, 0));
  aMesh->SetNode(2, gp_Pnt(2, 0, 1));
  aMesh->SetNode(3, gp_Pnt(0, 3, 1));
  aMesh->SetTriangle(1, Poly_Triangle(1, 2, 3));
  aMesh->AddNormals();
  aMesh->SetNormal(1, gp_Dir(1, 2, 3));
  aMesh->SetNormal(2, gp_Dir(3, 1, 2));
  aMesh->SetNormal(3, gp_Dir(2, 3, 1));
  aMesh->Deflection(0.25);
  TopoDS_Face aFace;
  BRep_Builder().MakeFace(aFace, aMesh);
  return aFace;
}

gp_Trsf placement()
{
  gp_Trsf aTrsf;
  aTrsf.SetRotation(gp_Ax1(gp_Pnt(), gp_Dir(1, 2, 3)), 0.4);
  aTrsf.SetTranslationPart(gp_Vec(7, -4, 2));
  return aTrsf;
}
} // namespace

TEST(BRepTools_AffineMesh, AuthoredNormalsAndLocatedNodes)
{
  // Independent inverse-transpose oracle; do not derive normals from facets.
  for (const double aSign : {1.0, -1.0})
  {
    for (const bool isLocated : {false, true})
    {
      const gp_Mat   aLinear(aSign, 0.5, 0, 0, 2, 0.3, 0, 0, 0.7);
      const gp_GTrsf aTrsf(aLinear, gp_XYZ(10, 20, 30));
      TopoDS_Face    aFace = meshOnlyFace();
      if (isLocated)
        aFace.Move(TopLoc_Location(placement()));
      TopLoc_Location   aBeforeLoc, anAfterLoc;
      const auto        aBefore = BRep_Tool::Triangulation(aFace, aBeforeLoc);
      const TopoDS_Face aResult =
        TopoDS::Face(BRepBuilderAPI_GTransform(aFace, aTrsf, true).Shape());
      const auto anAfter = BRep_Tool::Triangulation(aResult, anAfterLoc);
      ASSERT_FALSE(anAfter.IsNull());
      ASSERT_TRUE(anAfter->HasNormals());
      gp_Mat aNormalMat = aLinear.Inverted().Transposed();
      aNormalMat *= aLinear.Determinant();
      for (int i = 1; i <= 3; ++i)
      {
        gp_XYZ aPoint = aBefore->Node(i).Transformed(aBeforeLoc.Transformation()).XYZ();
        aTrsf.Transforms(aPoint);
        EXPECT_LT(
          anAfter->Node(i).Transformed(anAfterLoc.Transformation()).Distance(gp_Pnt(aPoint)),
          1e-9);
        const gp_Dir aNormal = aBefore->Normal(i).Transformed(aBeforeLoc.Transformation());
        const gp_Dir anExpected(aNormalMat * aNormal.XYZ());
        EXPECT_GT(anAfter->Normal(i).Transformed(anAfterLoc.Transformation()).Dot(anExpected),
                  1 - 1e-6);
        EXPECT_EQ(anAfter->Triangle(1).Value(i), aBefore->Triangle(1).Value(i));
      }
      EXPECT_DOUBLE_EQ(aBefore->Node(2).X(), 2.0);
      EXPECT_DOUBLE_EQ(aBefore->Deflection(), 0.25);
      EXPECT_GT(aBefore->Normal(1).Dot(gp_Dir(1, 2, 3)), 1 - 1e-6);
    }
  }
}

TEST(BRepTools_AffineMesh, MaximumEuclideanStretch)
{
  const gp_Mat
    aRotation(2.0 / 3, -1.0 / 3, 2.0 / 3, 2.0 / 3, 2.0 / 3, -1.0 / 3, -1.0 / 3, 2.0 / 3, 2.0 / 3);
  for (const double aScale : {1.0, 4.0, 1e-100, 1e100})
  {
    const gp_GTrsf    aTrsf(aRotation.Multiplied(aScale), gp_XYZ());
    const TopoDS_Face aFace   = meshOnlyFace();
    const auto        aResult = TopoDS::Face(BRepBuilderAPI_GTransform(aFace, aTrsf, true).Shape());
    TopLoc_Location   aLoc;
    const auto        aMesh = BRep_Tool::Triangulation(aResult, aLoc);
    ASSERT_FALSE(aMesh.IsNull());
    EXPECT_NEAR(aMesh->Deflection() / aScale, 0.25, 5e-15);
    ASSERT_TRUE(aMesh->HasNormals());
    EXPECT_GT(aMesh->Normal(1).Dot(gp_Dir(aRotation * gp_Dir(1, 2, 3).XYZ())), 1 - 1e-6);
  }
  // A unit shear stretches by the golden ratio, although every matrix entry <= 1.
  const gp_GTrsf aShear(gp_Mat(1, 1, 0, 0, 1, 0, 0, 0, 1), gp_XYZ());
  const double   aStretch = (1 + std::sqrt(5.0)) / 2;
  TopoDS_Vertex  aVertex;
  BRep_Builder().MakeVertex(aVertex, gp_Pnt(1, 2, 3), 0.25);
  const auto aResult = TopoDS::Vertex(BRepBuilderAPI_GTransform(aVertex, aShear, true).Shape());
  EXPECT_NEAR(BRep_Tool::Tolerance(aResult), 0.25 * aStretch, 5e-15);
  EXPECT_DOUBLE_EQ(BRep_Tool::Tolerance(aVertex), 0.25);
}

TEST(BRepTools_AffineMesh, CollapsedTangentsDiscardUndefinedNormals)
{
  const TopoDS_Face aFace = meshOnlyFace();
  const gp_GTrsf    aTrsf(gp_Mat(1, 0, 0, 0, 0, 0, 0, 0, 0), gp_XYZ());
  const auto        aResult = TopoDS::Face(BRepBuilderAPI_GTransform(aFace, aTrsf, true).Shape());
  TopLoc_Location   aLoc;
  const auto        aMesh = BRep_Tool::Triangulation(aResult, aLoc);
  ASSERT_FALSE(aMesh.IsNull());
  EXPECT_EQ(aMesh->NbNodes(), 3);
  EXPECT_FALSE(aMesh->HasNormals());
  EXPECT_TRUE(BRep_Tool::Triangulation(aFace, aLoc)->HasNormals());
}

TEST(BRepTools_AffineMesh, LocatedEdgePolygonWithoutParameters)
{
  NCollection_Array1<gp_Pnt> aNodes(1, 3);
  aNodes(1)                            = gp_Pnt(0, 0, 0);
  aNodes(2)                            = gp_Pnt(1, 1.5, 2);
  aNodes(3)                            = gp_Pnt(2, 3, 4);
  TopoDS_Edge                 anEdge   = BRepBuilderAPI_MakeEdge(aNodes(1), aNodes(3));
  occ::handle<Poly_Polygon3D> aPolygon = new Poly_Polygon3D(aNodes);
  aPolygon->Deflection(0.25);
  BRep_Builder().UpdateEdge(anEdge, aPolygon);
  anEdge.Move(TopLoc_Location(placement()));
  const gp_GTrsf  aTrsf(gp_Mat(1, 1, 0, 0, 1, 0, 0, 0, 1), gp_XYZ(10, 20, 30));
  const auto      aResult = TopoDS::Edge(BRepBuilderAPI_GTransform(anEdge, aTrsf, true).Shape());
  TopLoc_Location aLoc;
  const auto      aNewPoly = BRep_Tool::Polygon3D(aResult, aLoc);
  ASSERT_FALSE(aNewPoly.IsNull());
  EXPECT_NEAR(aNewPoly->Deflection(), 0.25 * (1 + std::sqrt(5.0)) / 2, 5e-15);
  for (int i = 1; i <= 3; ++i)
  {
    gp_XYZ aPoint = aNodes(i).Transformed(placement()).XYZ();
    aTrsf.Transforms(aPoint);
    EXPECT_LT(aNewPoly->Nodes()(i).Transformed(aLoc.Transformation()).Distance(gp_Pnt(aPoint)),
              1e-9);
    EXPECT_DOUBLE_EQ(aPolygon->Nodes()(i).Distance(aNodes(i)), 0.0);
  }
}

TEST(BRepTools_AffineMesh, LocatedSphereSurfaceTangentsAndPrecision)
{
  TopoDS_Shape aShape = BRepPrimAPI_MakeSphere(3).Shape();
  aShape.Move(TopLoc_Location(placement()));
  BRepMesh_IncrementalMesh(aShape, 0.05, false, 0.3);
  for (TopExp_Explorer anIt(aShape, TopAbs_FACE); anIt.More(); anIt.Next())
  {
    TopLoc_Location aLoc;
    const auto      aFace = TopoDS::Face(anIt.Current());
    const auto      aMesh = BRep_Tool::Triangulation(aFace, aLoc);
    BRepLib_ToolTriangulatedShape::ComputeNormals(aFace, aMesh);
  }
  const gp_GTrsf aTrsf(gp_Mat(2, 0.5, 0, 0, 1, 0.3, 0, 0, 0.7), gp_XYZ(10, 20, 30));
  const auto     aResult = BRepBuilderAPI_GTransform(aShape, aTrsf, true).Shape();
  ASSERT_TRUE(BRepCheck_Analyzer(aResult).IsValid());
  int aChecked = 0;
  for (TopExp_Explorer anIt(aResult, TopAbs_FACE); anIt.More(); anIt.Next())
  {
    TopLoc_Location aLoc;
    const auto      aFace = TopoDS::Face(anIt.Current());
    const auto      aMesh = BRep_Tool::Triangulation(aFace, aLoc);
    ASSERT_FALSE(aMesh.IsNull());
    ASSERT_TRUE(aMesh->HasNormals());
    ASSERT_TRUE(aMesh->HasUVNodes());
    EXPECT_TRUE(aMesh->Parameters().IsNull());
    BRepAdaptor_Surface aSurface(aFace);
    for (int i = 1; i <= aMesh->NbNodes(); ++i)
    {
      const gp_Pnt2d aUV = aMesh->UVNode(i);
      gp_Pnt         aPoint;
      gp_Vec         aDU, aDV;
      aSurface.D1(aUV.X(), aUV.Y(), aPoint, aDU, aDV);
      const gp_Vec aCross = aDU.Crossed(aDV);
      if (aCross.Magnitude() < 1e-7)
        continue; // Sphere poles have no regular UV tangent basis.
      EXPECT_GT(aMesh->Normal(i).Transformed(aLoc.Transformation()).Dot(gp_Dir(aCross)), 1 - 1e-6);
      ++aChecked;
    }
  }
  EXPECT_GT(aChecked, 20);
}

TEST(BRepTools_AffineMesh, NativeScaleAndExplicitMatrixAgree)
{
  for (const double aScale : {2.0, -2.0})
  {
    const TopoDS_Shape aShape = BRepPrimAPI_MakeBox(2, 3, 4).Shape();
    const gp_Pnt       aPivot(5, -2, 1);
    gp_Trsf            aNative;
    aNative.SetScale(aPivot, aScale);
    const gp_GTrsf anExplicit(gp_Mat(aScale, 0, 0, 0, aScale, 0, 0, 0, aScale),
                              aPivot.XYZ() * (1 - aScale));
    for (const gp_GTrsf& aTrsf : {gp_GTrsf(aNative), anExplicit})
    {
      const auto aResult = BRepBuilderAPI_GTransform(aShape, aTrsf, true).Shape();
      ASSERT_TRUE(BRepCheck_Analyzer(aResult).IsValid());
      GProp_GProps aProps;
      BRepGProp::VolumeProperties(aResult, aProps);
      EXPECT_NEAR(aProps.Mass(), 24 * std::pow(std::abs(aScale), 3), 1e-7);
      const gp_Pnt anExpected(gp_XYZ(1, 1.5, 2) * aScale + aPivot.XYZ() * (1 - aScale));
      EXPECT_LT(aProps.CentreOfMass().Distance(anExpected), 1e-8);
    }
  }
}
