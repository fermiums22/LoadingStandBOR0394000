// CalibFft.h -- compact, self-contained FFT helper for the Calibration tab.
//
// DriveScope did not ship an FFT before this. Test 3 (PI tuning + step
// response) needs a frequency-spectrum view of the captured current signal
// to show noise floor and spectral content. This header provides:
//
//   - calibFftMagnitude(): a radix-2 Cooley-Tukey FFT that takes a real
//     time-domain signal and returns the single-sided magnitude spectrum.
//   - calibNextPow2(): round a sample count up to the next power of two.
//   - a Hann window applied internally to reduce spectral leakage.
//
// Pure header-only, no external deps, no ImGui. Kept small on purpose --
// capture bursts are at most a few thousand samples, so an O(N log N)
// in-place radix-2 transform is more than fast enough and avoids pulling
// in a heavyweight DSP library.

#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace drivescope {

// Smallest power of two >= n (minimum 2). Used to zero-pad a capture burst
// to a radix-2 length before the transform.
inline size_t calibNextPow2(size_t n)
{
    size_t p = 2;
    while (p < n) p <<= 1;
    return p;
}

// In-place iterative radix-2 Cooley-Tukey FFT. re/im must be the same
// length and that length must be a power of two. Forward transform
// (negative exponent sign).
inline void calibFftInPlace(std::vector<float>& re, std::vector<float>& im)
{
    const size_t n = re.size();
    if (n < 2 || im.size() != n) return;

    // Bit-reversal permutation.
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            std::swap(re[i], re[j]);
            std::swap(im[i], im[j]);
        }
    }

    // Butterfly stages.
    for (size_t len = 2; len <= n; len <<= 1) {
        const double ang = -2.0 * 3.14159265358979323846 / static_cast<double>(len);
        const float wlenRe = static_cast<float>(std::cos(ang));
        const float wlenIm = static_cast<float>(std::sin(ang));
        for (size_t i = 0; i < n; i += len) {
            float wRe = 1.0f, wIm = 0.0f;
            for (size_t k = 0; k < len / 2; ++k) {
                const size_t a = i + k;
                const size_t b = i + k + len / 2;
                const float uRe = re[a];
                const float uIm = im[a];
                const float vRe = re[b] * wRe - im[b] * wIm;
                const float vIm = re[b] * wIm + im[b] * wRe;
                re[a] = uRe + vRe;  im[a] = uIm + vIm;
                re[b] = uRe - vRe;  im[b] = uIm - vIm;
                const float nwRe = wRe * wlenRe - wIm * wlenIm;
                wIm = wRe * wlenIm + wIm * wlenRe;
                wRe = nwRe;
            }
        }
    }
}

// Compute the single-sided amplitude spectrum of a real signal.
//
//   in        : time-domain samples
//   sampleHz  : sample rate (used to fill the returned freq[] axis)
//   freqOut   : output bin centre frequencies (Hz), length N/2
//   magOut    : output magnitudes (same units as the input), length N/2
//
// A Hann window is applied first to suppress spectral leakage; the
// amplitude is corrected for the window's coherent gain (0.5) and for the
// single-sided fold (x2 on all bins except DC). The DC bin is returned as
// the mean of the windowed signal.
inline void calibFftMagnitude(const std::vector<float>& in, double sampleHz,
                              std::vector<float>& freqOut,
                              std::vector<float>& magOut)
{
    freqOut.clear();
    magOut.clear();
    if (in.size() < 4 || sampleHz <= 0.0) return;

    const size_t n = calibNextPow2(in.size());
    std::vector<float> re(n, 0.0f), im(n, 0.0f);

    // Hann window over the populated region; zero-pad the rest.
    const size_t valid = in.size();
    for (size_t i = 0; i < valid; ++i) {
        const double w = 0.5 - 0.5 * std::cos(2.0 * 3.14159265358979323846 *
                                              static_cast<double>(i) /
                                              static_cast<double>(valid - 1));
        re[i] = static_cast<float>(in[i] * w);
    }

    calibFftInPlace(re, im);

    const size_t half = n / 2;
    freqOut.resize(half);
    magOut.resize(half);
    const float invN = 1.0f / static_cast<float>(n);
    // Coherent-gain correction for the Hann window (mean of window = 0.5).
    const float winGain = 2.0f;   // 1 / 0.5
    for (size_t k = 0; k < half; ++k) {
        const float mag = std::sqrt(re[k] * re[k] + im[k] * im[k]) * invN;
        // Single-sided: double everything except the DC bin.
        magOut[k] = (k == 0) ? mag * winGain : mag * 2.0f * winGain;
        freqOut[k] = static_cast<float>(static_cast<double>(k) * sampleHz /
                                        static_cast<double>(n));
    }
}

} // namespace drivescope
