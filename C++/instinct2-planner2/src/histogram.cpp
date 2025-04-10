#include "histogram.hpp"
#include <cmath>

Histogram::Histogram() {
    polar_histogram_.resize(HISTOGRAM_SIZE, 0.0f);
    binary_histogram_.resize(HISTOGRAM_SIZE, 0);
}

void Histogram::addPoint(float x, float y, float z, float safe_distance) {
    float angle = std::atan2(y, x) * 180.0 / M_PI;
    if (angle < 0) angle += 360.0;
    int bin = static_cast<int>(angle / (360.0 / HISTOGRAM_SIZE));
    if (bin >= HISTOGRAM_SIZE) bin = HISTOGRAM_SIZE - 1;

    float distance = std::sqrt(x * x + y * y + z * z);
    float weight = 1.0 - (distance / safe_distance);  // Пример веса
    if (weight > 0) {
        polar_histogram_[bin] += weight;
    }
}

void Histogram::computeBinaryHistogram(float threshold) {
    for (int i = 0; i < HISTOGRAM_SIZE; ++i) {
        binary_histogram_[i] = (polar_histogram_[i] > threshold) ? 1 : 0;
    }
}

void Histogram::smoothHistogram() {
    std::vector<float> smoothed(HISTOGRAM_SIZE);
    for (int i = 0; i < HISTOGRAM_SIZE; ++i) {
        int prev = (i - 1 + HISTOGRAM_SIZE) % HISTOGRAM_SIZE;
        int next = (i + 1) % HISTOGRAM_SIZE;
        smoothed[i] = (polar_histogram_[prev] + polar_histogram_[i] + polar_histogram_[next]) / 3.0f;
    }
    polar_histogram_ = smoothed;
}

void Histogram::reset() {
    std::fill(polar_histogram_.begin(), polar_histogram_.end(), 0.0f);
    std::fill(binary_histogram_.begin(), binary_histogram_.end(), 0);
}

const std::vector<float>& Histogram::getPolarHistogram() const {
    return polar_histogram_;
}

const std::vector<int>& Histogram::getBinaryHistogram() const {
    return binary_histogram_;
}