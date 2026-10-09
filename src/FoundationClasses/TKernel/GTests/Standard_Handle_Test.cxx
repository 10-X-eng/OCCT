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

#include <Standard_Transient.hxx>
#include <Standard_Handle.hxx>

#include <gtest/gtest.h>
#include <utility>

namespace
{
class OwnedHandleNode : public Standard_Transient
{
public:
  OwnedHandleNode(int& theLiveCount, int theValue)
      : Value(theValue),
        myLiveCount(theLiveCount)
  {
    ++myLiveCount;
  }

  ~OwnedHandleNode() override { --myLiveCount; }

  occ::handle<OwnedHandleNode> Child;
  int                          Value;

private:
  int& myLiveCount;
};
} // namespace

TEST(Standard_Handle_Test, OCC24533_IsNullAndPointerChecks)
{
  occ::handle<Standard_Transient> aHandle;

  // Test IsNull() for default-constructed handle
  EXPECT_TRUE(aHandle.IsNull());

  // Test pointer access for null handle
  const Standard_Transient* p = aHandle.get();
  EXPECT_FALSE(static_cast<bool>(p));
  EXPECT_EQ(nullptr, p);

  // Create non-null handle
  aHandle = new Standard_Transient();

  // Test IsNull() for non-null handle
  EXPECT_FALSE(aHandle.IsNull());

  // Test pointer access for non-null handle
  p = aHandle.get();
  EXPECT_TRUE(static_cast<bool>(p));
  EXPECT_NE(nullptr, p);
}

TEST(Standard_Handle_Test, CopyAssignmentFromOwnedChild)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> anOwner = new OwnedHandleNode(aLiveCount, 1);
    anOwner->Child                       = new OwnedHandleNode(aLiveCount, 2);
    anOwner                              = anOwner->Child;
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(2, anOwner->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, PointerAssignmentFromOwnedChild)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> anOwner = new OwnedHandleNode(aLiveCount, 1);
    anOwner->Child                       = new OwnedHandleNode(aLiveCount, 2);
    anOwner                              = anOwner->Child.get();
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(2, anOwner->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, MoveAssignmentFromOwnedChild)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> anOwner = new OwnedHandleNode(aLiveCount, 1);
    anOwner->Child                       = new OwnedHandleNode(aLiveCount, 2);
    anOwner                              = std::move(anOwner->Child);
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(2, anOwner->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, CopyAssignmentFromOwnedGrandchild)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> anOwner = new OwnedHandleNode(aLiveCount, 1);
    anOwner->Child                       = new OwnedHandleNode(aLiveCount, 2);
    anOwner->Child->Child                = new OwnedHandleNode(aLiveCount, 3);
    anOwner                              = anOwner->Child->Child;
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(3, anOwner->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, MoveAssignmentFromOwnedGrandchild)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> anOwner = new OwnedHandleNode(aLiveCount, 1);
    anOwner->Child                       = new OwnedHandleNode(aLiveCount, 2);
    anOwner->Child->Child                = new OwnedHandleNode(aLiveCount, 3);
    anOwner                              = std::move(anOwner->Child->Child);
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(3, anOwner->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, UpcastCopyAssignmentFromOwnedChild)
{
  int aLiveCount = 0;
  {
    auto                            aNode   = new OwnedHandleNode(aLiveCount, 1);
    occ::handle<Standard_Transient> anOwner = aNode;
    aNode->Child                            = new OwnedHandleNode(aLiveCount, 2);
    anOwner                                 = aNode->Child;
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(2, static_cast<OwnedHandleNode*>(anOwner.get())->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, UpcastMoveAssignmentFromOwnedChild)
{
  int aLiveCount = 0;
  {
    auto                            aNode   = new OwnedHandleNode(aLiveCount, 1);
    occ::handle<Standard_Transient> anOwner = aNode;
    aNode->Child                            = new OwnedHandleNode(aLiveCount, 2);
    anOwner                                 = std::move(aNode->Child);
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(2, static_cast<OwnedHandleNode*>(anOwner.get())->Value);
    EXPECT_EQ(1, anOwner->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, SelfAssignmentPreservesReferenceCount)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> anOwner = new OwnedHandleNode(aLiveCount, 1);
    anOwner                              = anOwner;
    anOwner                              = anOwner.get();
    anOwner                              = std::move(anOwner);
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(1, anOwner->GetRefCount());
    EXPECT_EQ(1, anOwner->Value);
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, MoveAssignmentReleasesPreviousDestination)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> aDestination = new OwnedHandleNode(aLiveCount, 1);
    occ::handle<OwnedHandleNode> aSource      = new OwnedHandleNode(aLiveCount, 2);
    aDestination                              = std::move(aSource);
    EXPECT_TRUE(aSource.IsNull());
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(2, aDestination->Value);
    EXPECT_EQ(1, aDestination->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, MoveAssignmentBetweenAliases)
{
  int aLiveCount = 0;
  {
    occ::handle<OwnedHandleNode> aDestination = new OwnedHandleNode(aLiveCount, 1);
    occ::handle<OwnedHandleNode> aSource      = aDestination;
    EXPECT_EQ(2, aSource->GetRefCount());
    aDestination = std::move(aSource);
    EXPECT_TRUE(aSource.IsNull());
    EXPECT_EQ(1, aLiveCount);
    EXPECT_EQ(1, aDestination->GetRefCount());
  }
  EXPECT_EQ(0, aLiveCount);
}

TEST(Standard_Handle_Test, NullAssignmentsReleasePreviousDestination)
{
  int                          aLiveCount = 0;
  occ::handle<OwnedHandleNode> anOwner    = new OwnedHandleNode(aLiveCount, 1);
  occ::handle<OwnedHandleNode> anEmpty;
  anOwner = anEmpty;
  EXPECT_EQ(0, aLiveCount);
  EXPECT_TRUE(anOwner.IsNull());
  anOwner = new OwnedHandleNode(aLiveCount, 2);
  anOwner = std::move(anEmpty);
  EXPECT_EQ(0, aLiveCount);
  EXPECT_TRUE(anOwner.IsNull());
  EXPECT_TRUE(anEmpty.IsNull());
}
