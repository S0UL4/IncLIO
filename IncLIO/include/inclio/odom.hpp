#ifndef INCLIO_ODOM_H
#define INCLIO_ODOM_H

namespace IncLIO {

struct Odom {
    Odom() {}
    Odom(double timestamp, double left_pulse, double right_pulse)
        : timestamp_(timestamp), left_pulse_(left_pulse), right_pulse_(right_pulse) {}

    double timestamp_ = 0.0;
    double left_pulse_ = 0.0;  // number of pulses per unit time for left and right wheels
    double right_pulse_ = 0.0;
};

}  // namespace IncLIO

#endif  // INCLIO_ODOM_H
