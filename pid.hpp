/*
 * Copyright (c) 2016 Olzhas Adiyatov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PID_H
#define PID_H

#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

constexpr double default_sampling_time{10e-3};

class DiscretePID {
 public:
  DiscretePID() = delete;
  explicit DiscretePID(const double& Kp, const double& Ki, const double& Kd,
                       double dt = default_sampling_time,
                       const bool& exec_at_set_ref = true)
      : _Kp(Kp),
        _Ki(Ki),
        _Kd(Kd),
        _dt(std::chrono::nanoseconds{static_cast<long>(dt * 1e9)}),
        _executeAtSetReference(exec_at_set_ref) {
    ;
  }

  ~DiscretePID() {
    setRunning(false);
    if (_pid_thread.joinable()) _pid_thread.join();
  }

  double getU() const { return _u[2]; }
  double getControlAction() const { return _u[2]; }

  void setReference(const double& ref) {
    std::lock_guard<std::mutex> lock(_reference_mutex);
    _r = ref;
    if (_executeAtSetReference && !_isRunning) {
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
    const double& Ts = _dt.count() * 1e-9;
    static double Ki = _Ki;
   
    double a = (_Kp + Ki * Ts / 2.0 + _Kd / Ts);
    double b = (-_Kp + Ki * Ts / 2.0 - 2 * _Kd / Ts);
    double c = _Kd / Ts;

    _u[2] = _u[1] + a * _e[2] + b * _e[1] + c * _e[0];

    _u[1] = _u[2];
    _u[0] = _u[1];

    // anti windup, turning off integration
    // if (fabs(_u[2]) > _saturation) {
    //  Ki = 0;
    // } else {
    //  Ki = _Ki;
    // }
   
    // anti windup
    if (fabs(_u[2]) > _saturation) {
      _u[2] = std::copysign(1.0, _u[2]) * _saturation;
    }
  }

  void computeE() {
    _e[0] = _e[1];
    _e[1] = _e[2];
    _e[2] = _r - _y[2];
  }

  void setSaturation(const double& saturation) { _saturation = saturation; }
  void setExecuteAtSetReference(bool executeAtSetReference) { _executeAtSetReference = executeAtSetReference; }

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
      auto timePoint = initialTimePoint + _dt * (++cnt);
      return timePoint;
    };

    double systemOutput;
    double controlAction;

    while (!terminationCondition()) {
      systemOutput = read();
      controlAction = calculate(systemOutput);
      _running_mutex.lock();
      if (_isRunning) write(controlAction);
      _running_mutex.unlock();
      std::this_thread::sleep_until(nextTimePoint());
    }
    // just making sure that control action is zeroed before we stop
    write(0);
    return;
  }

  void start() {
    setRunning(true);
    _pid_thread = std::thread(std::bind(&DiscretePID::controlLoop, this));

   // TODO
    /*
        if (!pthread_setschedparam(
                _pid_thread.native_handle(), SCHED_RR,
                new sched_param{.sched_priority =
                                    sched_get_priority_max(SCHED_RR)})) {
          std::cerr << "Error while setting highest priority" << std::endl;
        }
    */
  }

  void assignReadFunc(const std::function<double(void)>& readFn) {
    read = readFn;
  }

  void assignWriteFunc(const std::function<void(const double&)>& writeFn) {
    write = writeFn;
  }

  void assignRWFunc(const std::function<double(void)>& readFn,
                    const std::function<void(const double&)>& writeFn) {
    read = readFn;
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
  std::chrono::nanoseconds _dt;
  std::thread _pid_thread;

  std::mutex _running_mutex;
  std::mutex _reference_mutex;

  bool _isRunning{false};
  bool _executeAtSetReference{false};

  std::function<double(void)> read;
  std::function<void(const double&)> write;
};

typedef std::shared_ptr<DiscretePID> DiscretePIDPtr;

#endif  // PID_H
