#ifndef MCPTAM_CPER_H
#define MCPTAM_CPER_H

#include <vector>
#include <cstddef>
#include <algorithm>
#include <mcptam/KeyFrame.h> 
#include <mcptam/Types.h>       
#include <mcptam/MapPoint.h>   
#include <mcptam/Relocaliser.h>
#include <mcptam/GLWindow2.h>
#include <mcptam/TrackerTiming.h>

#define DEBUG_CPER         0
#define EXP_7              1e-7
#define EXP_8              1e-8
#define ENTROPY_THRESHOLD -4.0
#define MKF_BUFFER_SIZE    50

template <class PairItem1, class PairItem2, class Compare = std::less<PairItem1> >
struct SortPair
{
  bool operator()(const std::pair<PairItem1,PairItem2>&left, const std::pair<PairItem1,PairItem2>&right) 
  {
    Compare compare;
    return compare(left.first, right.first);
  }
  
};



template <class Element> 
class StreamBuffer
{
  public:
    StreamBuffer(std::size_t capacity = MKF_BUFFER_SIZE) 
    {
        capacity_ = capacity;
        head_ = 0;
        tail_ = 0;
        buffer_.reserve(capacity);
    }

   ~StreamBuffer() {}

   void Enqueue(Element& element)
   {
     if (size_ < capacity_)
     {
        ++tail_ %= capacity_;
        buffer_.push_back(element);
        size_++;
     }
     else
     {
        ++tail_ %= capacity_;
        buffer_.at(tail_) = element;
     }
   }

   Element& AtIndex(std::size_t index)
   {
     return buffer_.at(index);
   }
   
   Element& Head()
   {
       return buffer_.front();
   }

   void Clear()
   {
       buffer_.clear();
   }

   bool Empty()
   {
     return buffer_.empty();
   }

   std::size_t Size()
   {
      return size_;
   }

   std::size_t Capacity() const
   {
     return capacity_;
   }

   std::vector<Element>& GetBuffer()
   {
     return buffer_;
   }


 private:
   std::size_t          capacity_;
   std::size_t          head_;
   std::size_t          tail_;
   std::size_t          size_;
   std::vector<Element> buffer_;
};



#endif
