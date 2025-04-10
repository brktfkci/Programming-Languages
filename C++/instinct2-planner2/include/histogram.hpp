#ifndef HISTOGRAM_HPP
#define HISTOGRAM_HPP

#include "constants.hpp"
#include <vector>

class Histogram {
public:
    Histogram();
    void addPoint(float x, float y, float z, float safe_distance);
    void computeBinaryHistogram(float threshold);
    void smoothHistogram();
    void reset();  // Новый метод
    const std::vector<float>& getPolarHistogram() const;
    const std::vector<int>& getBinaryHistogram() const;

private:
    std::vector<float> polar_histogram_;
    std::vector<int> binary_histogram_;
    static constexpr int HISTOGRAM_SIZE = 72;  // Пример размера
};

#endif  // HISTOGRAM_HPP