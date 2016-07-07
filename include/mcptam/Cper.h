#ifndef MCPTAM_CPER_H
#define MCPTAM_CPER_H

#include <vector>
#include <cstddef>
#include <algorithm>
#include <ros/console.h>

#define DEBUG_CPER         0
#define EXP_7              1e-7
#define EXP_8              1e-8
#define ENTROPY_THRESHOLD -4.0
#define MKF_BUFFER_SIZE    50



template <class PairItem1, class PairItem2, class Compare = std::less<PairItem1> >
struct SortPair
{
  bool operator()(const std::pair<PairItem1,PairItem2>& left, const std::pair<PairItem1,PairItem2>& right) 
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
        size_ = 0;
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
        try
        {
            buffer_.at(tail_) = element;
            ++tail_ %= capacity_;
        }
        catch(const std::out_of_range& error)
        {
            ROS_ERROR_STREAM("Out of Range error in StrAeamBuffer::Enqueue(Element& element) -- Buffer size: " << \
                              size_ << ", Tried accessing: " << tail_ << " \n");
        }
     }
   }

   Element& AtIndex(std::size_t index)
   {
       if (index >= size_)
       {
           char index_str[21], size_str[21];
           sprintf(index_str, "%lu", static_cast<unsigned long>(index));
           sprintf(size_str,  "%lu", static_cast<unsigned long>(size_));
           std::string error = "Out of Range error in StreamBuffer::AtIndex(size_t index) -- Buffer size: " + \
                                std::string(size_str) + ", Tried accessing: " + std::string(index_str) + " \n";
           throw std::out_of_range (error); 
       }

       return buffer_.at(index);
   }
   
   Element& Head()
   {
       if (buffer_.empty())
       {
           throw std::out_of_range ("Out of Range error in StreamBuffer::Head(): Buffer is empty. \n");
       }
       
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

   void PrintBuffer()
   {
        typename std::vector<Element>::iterator it;
        for(it = buffer_.begin(); it != buffer_.end(); ++it) 
        {
            ROS_DEBUG_STREAM( "(" << (*it).first << ", " << (*it).second << ") ");
        }
   }


 private:
   std::size_t          capacity_;
   std::size_t          head_;
   std::size_t          tail_;
   std::size_t          size_;
   std::vector<Element> buffer_;
};



#endif
