/*
 * The MIT License
 *
 * Copyright 2017-2018 Norwegian University of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING  FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef FMI4CPP_FMUDRIVER_HPP
#define FMI4CPP_FMUDRIVER_HPP

#include <string>
#include <memory>

#include <fmi4cpp/fmi4cpp.hpp>
#include <fmi4cpp/fmi2/fmi2.hpp>

#include <fmi4cpp/driver/error_types.hpp>
#include <fmi4cpp/driver/driver_options.hpp>

namespace fmi4cpp::driver {

struct State {
  double h;
  double v;
};

class fmu_driver {
public:
  fmu_driver(const std::string &fmuPath, const driver_options &options) : fmuPath_(fmuPath), options_(options), slave_(nullptr) {}

  void init() {
    fmi2::fmu fmu(fmuPath_);

    if (fmu.get_model_description()->as_cs_description()->needs_execution_tool) {
      throw rejection("FMU requires execution tool.");
    }

    if (options_.modelExchange) {
#ifdef FMI4CPP_WITH_ODEINT
      auto solver = fmi4cpp::solver::make_solver<fmi4cpp::solver::euler_solver>(1E-3);
      slave_ = fmu.as_me_fmu()->new_instance(solver);
#else
      const char *msg = "Model Exchange mode selected, but driver has been built without odeint support!";
      throw failure(msg);
#endif
    } else {
      slave_ = fmu.as_cs_fmu()->new_instance();
    }

    slave_->setup_experiment(options_.startTime);
    slave_->enter_initialization_mode();
    slave_->exit_initialization_mode();
  }

  State step() {
    State result{};
    if (options_.stopTime == 0.0 || slave_->get_simulation_time() <= options_.stopTime) {
      if (!slave_->step(options_.stepSize)) {
        slave_->terminate();
        throw failure("Simulation terminated prematurely.");
      }
      slave_->read_real(0, result.h);
      slave_->read_real(0, result.v);
    }
    return result;
  }

private:
  const std::string &fmuPath_;
  const driver_options &options_;
  std::unique_ptr<fmu_slave<fmi2::cs_model_description>> slave_;
};

}  // namespace fmi4cpp::driver

#endif //FMI4CPP_FMUDRIVER_HPP
