#pragma once

double gaussian_noise(double sigma);

double gaussian_noise_at_time(
    const unsigned long satellite_id,
    const double time_s,
    const double sigma_m);
