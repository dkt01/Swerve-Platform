/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <array>

template <class T>
struct interpMapPoint {
  T inVal;
  T outVal;

  constexpr interpMapPoint(T in, T out) : inVal(in), outVal(out) {}

  constexpr bool operator<(const interpMapPoint<T>& other) { return inVal < other.inVal; }
  constexpr bool operator==(const interpMapPoint<T>& other) { return inVal == other.inVal; }
};

template <class T>
constexpr bool operator<(const interpMapPoint<T>& a, const T& b) {
  return a.inVal < b;
}

template <class T>
constexpr bool operator<(const T& a, const interpMapPoint<T>& b) {
  return a < b.inVal;
}

template <class T, int size>
class interpolationMap {
 public:
  interpolationMap() = delete;
  constexpr interpolationMap(std::array<interpMapPoint<T>, size> initArray) : m_mapArray(initArray) {
    // assert(("Map must contain at least one value.", !initArray.empty()));
    // assert(("Map values must be sorted.", std::is_sorted(initArray.cbegin(), initArray.cend())));
  }

  constexpr T map(const T inVal) const {
    if (inVal >= m_mapArray.back().inVal) {
      return m_mapArray.back().outVal;
    } else if (inVal <= m_mapArray.front().inVal) {
      return m_mapArray.front().outVal;
    } else {
      auto afterPoint{std::lower_bound(m_mapArray.cbegin(), m_mapArray.cend(), inVal)};
      auto beforePoint{std::prev(afterPoint)};
      const auto lerpPct = (inVal - beforePoint->inVal) / (afterPoint->inVal - beforePoint->inVal);
      return beforePoint->outVal + lerpPct * (afterPoint->outVal - beforePoint->outVal);
    }
  }
  constexpr T operator()(const T inVal) { return map(inVal); }

 private:
  std::array<interpMapPoint<T>, size> m_mapArray;
};
