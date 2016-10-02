#ifndef PID_H
#define PID_H

#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

class DiscretePID {
 public:
  DiscretePID() = delete;
  DiscretePID(
      const double& Kp, const double& Ki, const double& Kd,
      const std::chrono::nanoseconds& dt = std::chrono::milliseconds(10),
      bool exec_at_set_ref = true)
      : _Kp(Kp),
        _Ki(Ki),
        _Kd(Kd),
        _dt(dt),
        _executeAtSetReference(exec_at_set_ref) {
    ;
  }

  ~DiscretePID() {
    // not sure if it is a correct way
    setRunning(false);
    _pid_thread.join();
  }

  double getU() const { return _u[2]; }
  double getControlAction() const { return _u[2]; }

  void setReference(const double& ref) {
    std::lock_guard<std::mutex> lock(_reference_mutex);
    _r = ref;
    if (_executeAtSetReference && !_pid_thread.joinable()) {
      setRunning(true);
      start();
    };
  }
  void setY(const double& y) { _y[2] = y; }

  void setRunning(const bool& running) {
    std::lock_guard<std::mutex> lock(_running_mutex);
    _isRunning = running;
  }

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
    const double& Ts = _dt.count();
    double a = (_Kp + _Ki * Ts / 2.0 + _Kd / Ts);
    double b = (-_Kp + _Ki * Ts / 2.0 - 2 * _Kd / Ts);
    double c = _Kd / Ts;

    _u[2] = _u[1] + a * _e[2] + b * _e[1] + c * _e[0];

    _u[1] = _u[2];
    _u[0] = _u[1];

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

  void controlLoop() {
    auto initialTimePoint = std::chrono::high_resolution_clock::now();
    std::size_t cnt{0};

    auto terminationCondition = [&]() -> bool {
      std::lock_guard<std::mutex> lock(_running_mutex);
      bool res = _isRunning;
      return !res;
    };

    auto nextTimePoint =
        [&]() -> std::chrono::high_resolution_clock::time_point {
      ++cnt;
      return initialTimePoint + (_dt * cnt);
    };

    while (!terminationCondition()) {
      double systemOutput = read();
      double controlAction = calculate(systemOutput);
      _running_mutex.lock();
      if (_isRunning) write(controlAction);
      _running_mutex.unlock();
      std::this_thread::sleep_until(nextTimePoint());
    }
  }

  void start() {
    if (!pthread_setschedparam(
            _pid_thread.native_handle(), SCHED_RR,
            new sched_param{.sched_priority =
                                sched_get_priority_max(SCHED_RR)})) {
      std::cerr << "Error while setting highest priority" << std::endl;
      std::terminate();
    }

    //    []() -> const struct sched_param* {
    //      struct sched_param* p = new struct sched_param;
    //      p->sched_priority =
    //          sched_get_prioriity_max(SCHED_RR);
    //      return p;
    //    }()

    _pid_thread = std::thread(std::bind(&DiscretePID::controlLoop, this));
  }

  void assignReadFunc(const std::function<double(void)>& readFn) {
    read = readFn;
  }

  void assignWriteFunc(const std::function<void(double)>& writeFn) {
    write = writeFn;
  }

  void setSamplingTime(const std::chrono::nanoseconds& dt) { _dt = dt; }

 private:
  double _Kp{0};
  double _Ki{0};
  double _Kd{0};
  double _u[3]{0};
  double _e[3]{0};
  double _r{0};
  double _y[3]{0};

  double _saturation{std::nan("")};
  std::chrono::nanoseconds _dt{std::chrono::milliseconds(10)};
  std::thread _pid_thread;

  std::mutex _running_mutex;
  std::mutex _reference_mutex;

  bool _isRunning{false};
  bool _executeAtSetReference{false};

  std::function<double(void)> read;
  std::function<void(double)> write;
};

#endif  // PID_H
