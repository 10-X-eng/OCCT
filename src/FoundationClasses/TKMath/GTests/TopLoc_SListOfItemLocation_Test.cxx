// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <TopLoc_SListOfItemLocation.hxx>
#include <TopLoc_ItemLocation.hxx>
#include <TopLoc_Datum3D.hxx>
#include <gp_Trsf.hxx>
#include <gtest/gtest.h>

namespace
{
TopLoc_SListOfItemLocation makeList(int theLength)
{
  gp_Trsf aTransform;
  aTransform.SetTranslation(gp_Vec(1., 2., 3.));
  occ::handle<TopLoc_Datum3D> aDatum = new TopLoc_Datum3D(aTransform);
  TopLoc_SListOfItemLocation  aList;
  for (int i = 0; i < theLength; ++i)
    aList.Construct(TopLoc_ItemLocation(aDatum, i + 1));
  return aList;
}
} // namespace

TEST(TopLoc_SListOfItemLocation_Test, UniqueListCanAdvanceToItsTail)
{
  auto        aList      = makeList(3);
  const auto* aTailValue = &aList.Tail().Value();
  aList.ToTail();
  ASSERT_TRUE(aList.More());
  EXPECT_EQ(&aList.Value(), aTailValue);
  aList.Next();
  ASSERT_TRUE(aList.More());
  aList.Next();
  EXPECT_FALSE(aList.More());
  aList.Next();
  EXPECT_FALSE(aList.More());
}

TEST(TopLoc_SListOfItemLocation_Test, AssignmentFromOwnedDescendantRetainsIt)
{
  auto        aList       = makeList(4);
  const auto& aDescendant = aList.Tail().Tail();
  const auto* aValue      = &aDescendant.Value();
  aList.Assign(aDescendant);
  ASSERT_TRUE(aList.More());
  EXPECT_EQ(&aList.Value(), aValue);
  EXPECT_TRUE(aList.Tail().More());
  EXPECT_FALSE(aList.Tail().Tail().More());
}

TEST(TopLoc_SListOfItemLocation_Test, EmptySelfAndSharedAssignmentsPreserveValues)
{
  TopLoc_SListOfItemLocation anEmpty;
  anEmpty.Assign(anEmpty);
  anEmpty.ToTail();
  EXPECT_FALSE(anEmpty.More());
  auto        aList  = makeList(3);
  const auto* aFirst = &aList.Value();
  aList.Assign(aList);
  EXPECT_EQ(&aList.Value(), aFirst);
  auto aShared = aList;
  aList.Assign(aShared);
  EXPECT_EQ(&aList.Value(), aFirst);
  aList.ToTail();
  EXPECT_EQ(&aShared.Value(), aFirst);
  EXPECT_EQ(&aList.Value(), &aShared.Tail().Value());
  aList.Assign(anEmpty);
  EXPECT_FALSE(aList.More());
  EXPECT_EQ(&aShared.Value(), aFirst);
}
