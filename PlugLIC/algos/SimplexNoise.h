#pragma once

#include <cstddef>

class SimplexNoise {
 public:
  static float noise(float x, float y);

  /**
   * Constructor of to initialize a fractal noise summation
   *
   * @param[in] frequency    Frequency ("width") of the first octave of noise
   * (default to 1.0)
   * @param[in] amplitude    Amplitude ("height") of the first octave of noise
   * (default to 1.0)
   * @param[in] lacunarity   Lacunarity specifies the frequency multiplier
   * between successive octaves (default to 2.0).
   * @param[in] persistence  Persistence is the loss of amplitude between
   * successive octaves (usually 1/lacunarity)
   */
  explicit SimplexNoise(float frequency = 1.0f, float amplitude = 1.0f, float lacunarity = 2.0f,
                        float persistence = 0.5f)
      : mFrequency(frequency), mAmplitude(amplitude), mLacunarity(lacunarity), mPersistence(persistence) {}

 private:
  // Parameters of Fractional Brownian Motion (fBm) : sum of N "octaves" of
  // noise
  float mFrequency;    ///< Frequency ("width") of the first octave of noise
                       ///< (default to 1.0)
  float mAmplitude;    ///< Amplitude ("height") of the first octave of noise
                       ///< (default to 1.0)
  float mLacunarity;   ///< Lacunarity specifies the frequency multiplier between
                       ///< successive octaves (default to 2.0).
  float mPersistence;  ///< Persistence is the loss of amplitude between
                       ///< successive octaves (usually 1/lacunarity)
};
