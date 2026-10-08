// Copyright (c) 2026 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.

#include <NCollection_IncAllocator.hxx>
#include <Standard_OutOfMemory.hxx>

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>
#ifdef __linux__
  #include <sys/resource.h>
  #include <unistd.h>
#endif

TEST(NCollection_IncAllocator_Test, PayloadAlignmentDoesNotDependOnBlockAlignment)
{
  using Block = NCollection_IncAllocator::IBlock;
  alignas(
    std::max_align_t) unsigned char aBuffer[sizeof(Block) + 17 + 2 * alignof(std::max_align_t)];
  for (std::size_t anOffset = 0; anOffset < alignof(std::max_align_t); anOffset += alignof(Block))
  {
    auto* aBlock   = new (aBuffer + anOffset) Block(aBuffer + anOffset, 17);
    auto* aPayload = aBlock->CurPointer.load();
    EXPECT_EQ(reinterpret_cast<std::uintptr_t>(aPayload) % alignof(std::max_align_t), 0u);
    EXPECT_EQ(aBlock->AvailableSize.load(), 17u);
    std::memset(aPayload, 0x5a, 17);
    EXPECT_EQ(aPayload[16], 0x5a);
    aBlock->~Block();
  }
}

TEST(NCollection_IncAllocator_Test, OddSizesStayAlignedAcrossResets)
{
  for (bool isThreadSafe : {false, true})
  {
    NCollection_IncAllocator anAllocator(2049);
    anAllocator.SetThreadSafe(isThreadSafe);
    for (int aPass = 0; aPass < 3; ++aPass)
    {
      for (std::size_t aSize = 1; aSize <= 257; ++aSize)
      {
        void* aPointer =
          aSize % 2 ? anAllocator.Allocate(aSize) : anAllocator.AllocateOptimal(aSize);
        ASSERT_NE(aPointer, nullptr);
        ASSERT_EQ(reinterpret_cast<std::uintptr_t>(aPointer) % alignof(std::max_align_t), 0u)
          << "size=" << aSize << " pass=" << aPass << " threadSafe=" << isThreadSafe;
        std::memset(aPointer, 0x5a, aSize);
      }
      anAllocator.Reset(aPass == 1);
    }
  }
}

TEST(NCollection_IncAllocator_Test, ConcurrentAllocationsAreAlignedAndDisjoint)
{
  struct Allocation
  {
    unsigned char* Pointer;
    std::size_t    Size;
    unsigned char  Pattern;
  };

  NCollection_IncAllocator anAllocator;
  anAllocator.SetThreadSafe(true);
  std::array<std::vector<Allocation>, 8> aResults;
  std::array<std::thread, 8>             aThreads;
  for (std::size_t t = 0; t < aThreads.size(); ++t)
  {
    aThreads[t] = std::thread([&, t] {
      for (std::size_t i = 0; i < 1024; ++i)
      {
        const std::size_t aSize    = 1 + (17 * i + t) % 127;
        const auto        aPattern = static_cast<unsigned char>((i + 31 * t) % 251);
        auto*             aPointer = static_cast<unsigned char*>(anAllocator.Allocate(aSize));
        std::memset(aPointer, aPattern, aSize);
        aResults[t].push_back({aPointer, aSize, aPattern});
      }
    });
  }
  for (auto& aThread : aThreads)
    aThread.join();
  std::vector<Allocation> anOrdered;
  for (const auto& aThreadResults : aResults)
  {
    for (const auto& anAllocation : aThreadResults)
    {
      ASSERT_EQ(reinterpret_cast<std::uintptr_t>(anAllocation.Pointer) % alignof(std::max_align_t),
                0u);
      for (std::size_t i = 0; i < anAllocation.Size; ++i)
        ASSERT_EQ(anAllocation.Pointer[i], anAllocation.Pattern);
      anOrdered.push_back(anAllocation);
    }
  }
  std::sort(anOrdered.begin(), anOrdered.end(), [](const Allocation& a, const Allocation& b) {
    return reinterpret_cast<std::uintptr_t>(a.Pointer)
           < reinterpret_cast<std::uintptr_t>(b.Pointer);
  });
  for (std::size_t i = 1; i < anOrdered.size(); ++i)
  {
    EXPECT_LE(reinterpret_cast<std::uintptr_t>(anOrdered[i - 1].Pointer) + anOrdered[i - 1].Size,
              reinterpret_cast<std::uintptr_t>(anOrdered[i].Pointer));
  }
}

TEST(NCollection_IncAllocator_Test, OversizedRequestsDoNotTruncateBlockSize)
{
  if (sizeof(std::size_t) <= sizeof(unsigned int))
    GTEST_SKIP() << "The truncation reproducer requires a wider size_t";
  const std::size_t aTooLarge = std::size_t(std::numeric_limits<unsigned int>::max()) + 1;
  EXPECT_THROW(NCollection_IncAllocator anAllocator(aTooLarge), Standard_OutOfMemory);
  for (bool isThreadSafe : {false, true})
  {
    NCollection_IncAllocator anAllocator;
    anAllocator.SetThreadSafe(isThreadSafe);
    ASSERT_THROW(anAllocator.Allocate(aTooLarge), Standard_OutOfMemory);
    ASSERT_THROW(anAllocator.AllocateOptimal(aTooLarge), Standard_OutOfMemory);
    ASSERT_THROW(anAllocator.Allocate(std::numeric_limits<std::size_t>::max()),
                 Standard_OutOfMemory);
    // Failed allocations must leave the allocator usable.
    auto* aPointer = static_cast<unsigned char*>(anAllocator.Allocate(17));
    ASSERT_NE(aPointer, nullptr);
    std::memset(aPointer, 0xa5, 17);
    EXPECT_EQ(aPointer[16], 0xa5);
  }
}

#ifdef __linux__
TEST(NCollection_IncAllocator_Test, FailedSystemAllocationRaisesInsteadOfDereferencingNull)
{
  // Limit only the child. No geometry or global allocator behavior is mocked.
  for (bool isThreadSafe : {false, true})
  {
    ASSERT_EXIT(
      {
        NCollection_IncAllocator anAllocator;
        anAllocator.SetThreadSafe(isThreadSafe);
        struct rlimit aLimit;
        if (getrlimit(RLIMIT_AS, &aLimit) != 0)
          _exit(3);
        const struct rlimit anOriginalLimit = aLimit;
        aLimit.rlim_cur                     = 0;
        if (setrlimit(RLIMIT_AS, &aLimit) != 0)
          _exit(3);
        try
        {
          anAllocator.AllocateOptimal(std::size_t(1) << 30);
        }
        catch (const Standard_OutOfMemory&)
        {
          if (setrlimit(RLIMIT_AS, &anOriginalLimit) != 0)
            _exit(3);
          auto* aPointer = static_cast<unsigned char*>(anAllocator.Allocate(17));
          if (aPointer == nullptr)
            _exit(5);
          std::memset(aPointer, 0xa5, 17);
          if (aPointer[16] != 0xa5)
            _exit(6);
          _exit(0);
        }
        catch (...)
        {
          _exit(4);
        }
        _exit(2);
      },
      testing::ExitedWithCode(0),
      "");
  }
}
#endif
