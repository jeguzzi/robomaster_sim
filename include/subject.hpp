#ifndef INCLUDE_SUBJECT_HPP_
#define INCLUDE_SUBJECT_HPP_

#include <cstdint>
#include <string>
#include <vector>

class Robot;

struct Subject {
  virtual std::vector<uint8_t> encode() = 0;
  virtual void update(Robot *) = 0;
  Subject() {}
  virtual ~Subject() {}
  virtual std::string name() = 0;
};

template <uint64_t _uid> struct SubjectWithUID : Subject {
  inline static uint64_t uid = _uid;
  SubjectWithUID() {}
};

#endif  // INCLUDE_SUBJECT_HPP_
