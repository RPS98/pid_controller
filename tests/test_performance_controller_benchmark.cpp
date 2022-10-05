#include <benchmark/benchmark.h>
#include <PID_controller.hpp>
#include <exception>

using namespace controller;

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

static void BM_TEST(benchmark::State &state) {
  PIDController controller = PIDController();
  Vector3d input           = Vector3d::Identity();
  bool flag                = true;

  controller.setGains(input, input, input);
  controller.setAntiWindup(input);
  controller.setAlpha(input);
  controller.setResetIntegralSaturationFlag(flag);

  double dt = 0.01;

  Vector3d state_vec = Vector3d::Identity();
  Vector3d ref_vec   = 2.0 * Vector3d::Identity();
  for (auto _ : state) {
    controller.computeControl(dt, state_vec, ref_vec);
  }
}
BENCHMARK(BM_TEST)->Threads(1)->Repetitions(10);

int main(int argc, char **argv) {
  // benchmark::RegisterBenchmark("run Efficiciency", BM_TEST);
  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}