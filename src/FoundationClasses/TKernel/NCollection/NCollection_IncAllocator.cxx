// Copyright (c) 2002-2023 OPEN CASCADE SAS
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

#include <NCollection_IncAllocator.hxx>

#include <Standard_OutOfMemory.hxx>

#include <cmath>
#include <algorithm>
#include <cstddef>
#include <limits>

IMPLEMENT_STANDARD_RTTIEXT(NCollection_IncAllocator, NCollection_BaseAllocator)

namespace
{
constexpr size_t THE_ALIGNMENT = alignof(std::max_align_t);
static_assert((THE_ALIGNMENT & (THE_ALIGNMENT - 1)) == 0);
constexpr size_t THE_BLOCK_OVERHEAD = sizeof(NCollection_IncAllocator::IBlock) + THE_ALIGNMENT - 1;
// myBlockSize is unsigned; neither its conversion nor the allocation including
// the block header may wrap. Round down to leave room for request alignment.
constexpr size_t THE_MAX_BLOCK_SIZE =
  std::min(size_t(std::numeric_limits<unsigned>::max()),
           std::numeric_limits<size_t>::max() - THE_BLOCK_OVERHEAD)
  & ~(THE_ALIGNMENT - 1);

char* alignedBlockStart(void* theBlock)
{
  // Some supported memory managers align the block itself only to a pointer.
  // Keep its original address for Free(), and align the payload independently.
  void*  aStart = static_cast<char*>(theBlock) + sizeof(NCollection_IncAllocator::IBlock);
  size_t aSpace = THE_ALIGNMENT;
  return static_cast<char*>(std::align(THE_ALIGNMENT, 0, aStart, aSpace));
}

// Bounds for checking block size level
// clang-format off
  static constexpr unsigned THE_SMALL_BOUND_BLOCK_SIZE = NCollection_IncAllocator::THE_DEFAULT_BLOCK_SIZE * 16;   // 196 KB
  static constexpr unsigned THE_MEDIUM_BOUND_BLOCK_SIZE = NCollection_IncAllocator::THE_DEFAULT_BLOCK_SIZE * 64;  // 786 KB
  static constexpr unsigned THE_LARGE_BOUND_BLOCK_SIZE = NCollection_IncAllocator::THE_DEFAULT_BLOCK_SIZE * 1024; // 12 MB
// clang-format on

//=================================================================================================

NCollection_IncAllocator::IBlockSizeLevel computeLevel(const unsigned int theSize)
{
  if (theSize < NCollection_IncAllocator::THE_DEFAULT_BLOCK_SIZE)
  {
    return NCollection_IncAllocator::IBlockSizeLevel::Min;
  }
  else if (theSize < THE_SMALL_BOUND_BLOCK_SIZE)
  {
    return NCollection_IncAllocator::IBlockSizeLevel::Small;
  }
  else if (theSize < THE_MEDIUM_BOUND_BLOCK_SIZE)
  {
    return NCollection_IncAllocator::IBlockSizeLevel::Medium;
  }
  else if (theSize < THE_LARGE_BOUND_BLOCK_SIZE)
  {
    return NCollection_IncAllocator::IBlockSizeLevel::Large;
  }
  else
  {
    return NCollection_IncAllocator::IBlockSizeLevel::Max;
  }
}
} // namespace

//=================================================================================================

NCollection_IncAllocator::NCollection_IncAllocator(const size_t theDefaultSize)
    : myBlockSize(static_cast<unsigned>(
        theDefaultSize < THE_MINIMUM_BLOCK_SIZE ? THE_DEFAULT_BLOCK_SIZE : theDefaultSize))
{
  if (theDefaultSize > THE_MAX_BLOCK_SIZE)
  {
    throw Standard_OutOfMemory("NCollection_IncAllocator: block size exceeds supported range");
  }
}

//=================================================================================================

void NCollection_IncAllocator::SetThreadSafe(const bool theIsThreadSafe)
{
  if (theIsThreadSafe)
  {
    if (!myMutex)
    {
      myMutex = std::make_unique<std::shared_mutex>();
    }
  }
  else
  {
    myMutex.reset();
  }
}

//=================================================================================================

NCollection_IncAllocator::~NCollection_IncAllocator()
{
  clean();
}

//=================================================================================================

void* NCollection_IncAllocator::AllocateOptimal(const size_t theRequestedSize)
{
  if (theRequestedSize > THE_MAX_BLOCK_SIZE)
  {
    throw Standard_OutOfMemory("NCollection_IncAllocator: allocation size exceeds supported range");
  }
  const size_t theSize = (theRequestedSize + THE_ALIGNMENT - 1) & ~(THE_ALIGNMENT - 1);
  if (myMutex)
  {
    // Fast path: shared lock allows multiple threads to bump-allocate
    // concurrently via CAS. Block list structure is read-only under shared lock.
    {
      std::shared_lock<std::shared_mutex> aSharedLock(*myMutex);
      IBlock*                             aBlock = myAllocationHeap;
      if (aBlock)
      {
        size_t anAvail = aBlock->AvailableSize.load(std::memory_order_relaxed);
        while (anAvail >= theSize)
        {
          if (aBlock->AvailableSize.compare_exchange_weak(anAvail,
                                                          anAvail - theSize,
                                                          std::memory_order_acquire,
                                                          std::memory_order_relaxed))
          {
            // Won the CAS - claim our disjoint portion of the block.
            char* aRes = aBlock->CurPointer.fetch_add(theSize, std::memory_order_relaxed);
            return aRes;
          }
          // CAS failed (another thread allocated), anAvail updated - retry.
        }
      }
    } // shared lock released before taking exclusive

    // Slow path: exclusive lock for new block allocation and list reordering.
    std::lock_guard<std::shared_mutex> aExclLock(*myMutex);
    return allocateSlow(theSize);
  }

  // Non-thread-safe path: no synchronization overhead at all.
  return allocateSlow(theSize);
}

//=================================================================================================

void* NCollection_IncAllocator::allocateSlow(const size_t theSize)
{
  IBlock* aBlock = nullptr;
  // Use existing head block if it has enough space.
  if (myAllocationHeap
      && myAllocationHeap->AvailableSize.load(std::memory_order_relaxed) >= theSize)
  {
    aBlock = myAllocationHeap;
  }
  else // Allocate new block from OS.
  {
    const unsigned aPreviousBlockSize  = myBlockSize;
    const unsigned aPreviousBlockCount = myBlockCount;
    if (++myBlockCount % 5 == 0)
    {
      increaseBlockSize();
    }
    if (myBlockSize < theSize)
    {
      myBlockSize = static_cast<unsigned>(theSize);
    }
    void* aBufferBlock = nullptr;
    try
    {
      aBufferBlock = Standard::AllocateOptimal(myBlockSize + THE_BLOCK_OVERHEAD);
    }
    catch (...)
    {
      myBlockSize  = aPreviousBlockSize;
      myBlockCount = aPreviousBlockCount;
      throw;
    }
    if (aBufferBlock == nullptr)
    {
      myBlockSize  = aPreviousBlockSize;
      myBlockCount = aPreviousBlockCount;
      throw Standard_OutOfMemory("NCollection_IncAllocator: block allocation failed");
    }
    aBlock                   = new (aBufferBlock) IBlock(aBufferBlock, myBlockSize);
    aBlock->NextBlock        = myAllocationHeap;
    aBlock->NextOrderedBlock = myOrderedBlocks;
    myOrderedBlocks          = aBlock;
    myAllocationHeap         = aBlock;
  }
  // Bump allocate within the selected block.
  const size_t anAvail = aBlock->AvailableSize.load(std::memory_order_relaxed);
  void*        aRes    = aBlock->CurPointer.load(std::memory_order_relaxed);
  aBlock->CurPointer.store(static_cast<char*>(aRes) + theSize, std::memory_order_relaxed);
  aBlock->AvailableSize.store(anAvail - theSize, std::memory_order_relaxed);
  if (anAvail - theSize < 16)
  {
    myAllocationHeap  = aBlock->NextBlock;
    aBlock->NextBlock = myUsedHeap;
    myUsedHeap        = aBlock;
  }
  else
  {
    IBlock* aBlockIter           = aBlock->NextBlock;
    IBlock* aBlockToReplaceAfter = nullptr;
    while (aBlockIter) // Search new sorted position
    {
      if (aBlockIter->AvailableSize.load(std::memory_order_relaxed)
          > aBlock->AvailableSize.load(std::memory_order_relaxed))
      {
        aBlockToReplaceAfter = aBlockIter;
        aBlockIter           = aBlockIter->NextBlock;
        continue;
      }
      break;
    }
    if (aBlockToReplaceAfter) // Update list order
    {
      IBlock* aNext                   = aBlockToReplaceAfter->NextBlock;
      aBlockToReplaceAfter->NextBlock = aBlock;
      myAllocationHeap                = aBlock->NextBlock;
      aBlock->NextBlock               = aNext;
    }
  }
  return aRes;
}

//=================================================================================================

void* NCollection_IncAllocator::Allocate(const size_t theSize)
{
  return AllocateOptimal(theSize);
}

//=================================================================================================

void NCollection_IncAllocator::clean()
{
  // Exclusive lock - no allocations can be in progress.
  if (myMutex)
  {
    myMutex->lock();
  }
  IBlock* aHeapIter = myOrderedBlocks;
  while (aHeapIter)
  {
    IBlock* aCur = aHeapIter;
    aHeapIter    = aHeapIter->NextOrderedBlock;
    Standard::Free(aCur);
  }
  myOrderedBlocks  = nullptr;
  myAllocationHeap = nullptr;
  myUsedHeap       = nullptr;
  myBlockCount     = 0;
  myBlockSize      = THE_DEFAULT_BLOCK_SIZE;
  if (myMutex)
  {
    myMutex->unlock();
  }
}

//=================================================================================================

void NCollection_IncAllocator::increaseBlockSize()
{
  switch (computeLevel(myBlockSize))
  {
    case NCollection_IncAllocator::IBlockSizeLevel::Min:
      myBlockSize *= 8;
      break;
    case NCollection_IncAllocator::IBlockSizeLevel::Small:
      myBlockSize *= 4;
      break;
    case NCollection_IncAllocator::IBlockSizeLevel::Medium:
      myBlockSize *= 2;
      break;
    case NCollection_IncAllocator::IBlockSizeLevel::Large:
      myBlockSize = static_cast<unsigned>(std::lround(myBlockSize * 1.5));
      break;
    case NCollection_IncAllocator::IBlockSizeLevel::Max:
      break;
  }
}

//=================================================================================================

void NCollection_IncAllocator::resetBlock(IBlock* theBlock) const
{
  char*  aBlockStart = alignedBlockStart(theBlock);
  size_t anAvail     = theBlock->AvailableSize.load(std::memory_order_relaxed);
  char*  aCurPtr     = theBlock->CurPointer.load(std::memory_order_relaxed);
  theBlock->AvailableSize.store(anAvail + static_cast<size_t>(aCurPtr - aBlockStart),
                                std::memory_order_relaxed);
  theBlock->CurPointer.store(aBlockStart, std::memory_order_relaxed);
}

//=================================================================================================

void NCollection_IncAllocator::Reset(const bool theReleaseMemory)
{
  if (theReleaseMemory)
  {
    clean();
    return;
  }
  if (myMutex)
  {
    myMutex->lock();
  }
  IBlock* aHeapIter = myOrderedBlocks;
  while (aHeapIter)
  {
    IBlock* aCur    = aHeapIter;
    aHeapIter       = aHeapIter->NextOrderedBlock;
    aCur->NextBlock = aHeapIter;
    resetBlock(aCur); // reset size and pointer
  }
  myAllocationHeap = myOrderedBlocks;
  myUsedHeap       = nullptr;
  if (myMutex)
  {
    myMutex->unlock();
  }
}

//=================================================================================================

NCollection_IncAllocator::IBlock::IBlock(void* thePointer, const size_t theSize)
    : CurPointer(alignedBlockStart(thePointer)),
      AvailableSize(theSize)
{
}
