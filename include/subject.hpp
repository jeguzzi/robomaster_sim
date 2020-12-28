#ifndef SUBJECT_H
#define SUBJECT_H

#include <vector>
#include <cstdint>

#include "robot.hpp"

struct Subject
{
  virtual std::vector<uint8_t> encode() = 0;
  virtual void update(Robot *) = 0;
  Subject() { };
  virtual ~Subject() {}
};

template <uint64_t _uid>
struct SubjectWithUID : Subject
{
  inline static uint64_t uid = _uid;
  SubjectWithUID() { };
};

#endif /* end of include guard: SUBJECT_H */
