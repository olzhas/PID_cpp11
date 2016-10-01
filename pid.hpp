#ifndef PID_H
#define PID_H

#include <chrono>
#include <cmath>
#include <initializer_list>

class DiscretePID {
 public:
  DiscretePID() { ; }
  DiscretePID(const double& Kp, const double& Ki, const double& Kd)
      : _Kp(Kp), _Ki(Ki), _Kd(Kd) {
    ;
  }

  double getU() const { return _u[2]; }
  double getControlAction() const { return _u[2]; }

  void setReference(const double& ref) { _r = ref; }
  void setY(const double& y) { _y[2] = y; }

  double calculate(const double& ref, const double& y) {
    setReference(ref);
    return calculate(y);
  }

  double calculate(const double& y) {
    setY(y);
    computeE();
    computeU();
    return getU();
  }
  void setCoefficients(const double& Kp, const double& Ki, const double& Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
  }

  void computeU() {
    const double& Ts = dt.count();
    double a = (_Kp + _Ki * Ts / 2.0 + _Kd / Ts);
    double b = (-_Kp + _Ki * Ts / 2.0 - 2 * _Kd / Ts);
    double c = _Kd / Ts;

    _u[2] = _u[1] + a * _e[2] + b * _e[1] + c * _e[0];

    if (fabs(_u[2]) > _saturation) {
      _u[2] = std::copysign(1.0, _u[2]) * _saturation;
    } else {
      // anti windup
      double delta_u = _u[1] - _u[0];
      _u[2] = _u[1] + delta_u;
    }
  }

  void computeE() {
    _e[0] = _e[1];
    _e[1] = _e[2];
    _e[2] = _r - _y[2];
  }

  void setSaturation(const double& saturation) { _saturation = saturation; }

 private:
  double _Kp{0};
  double _Ki{0};
  double _Kd{0};

  double _u[3]{0};
  double _e[3]{0};
  double _r{0};
  double _y[3]{0};

  double _saturation{std::nan("")};
  std::chrono::duration<double> dt{std::chrono::milliseconds(10)};
};

#endif  // PID_H
