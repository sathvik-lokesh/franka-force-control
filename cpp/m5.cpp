// M5: the M1 force-control task in C++ on the MuJoCo C API, plus timing.
//
//   m5 law    <trace.bin>                 replay recorded controller inputs, compare torques
//   m5 closed <model.mjb> <trace.bin>     own simulation from t = 0, compare torques tick by tick
//   m5 rt     <model.mjb> <seconds> <sleep|spin>   1 kHz loop on an absolute schedule
//
// Each prints one JSON object. Plain Linux under WSL2 is not real-time: rt reports misses, it makes no RT claim.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#include "controller.hpp"

using namespace ffc;

namespace {

constexpr int kRecord = 135;   // doubles per tick, see m5_run.py
constexpr double kVApproach = 0.02, kTareTime = 0.5, kContactThresh = 1.0, kKPos = 500.0;

int64_t now_ns() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

struct Trace {
  int64_t n = 0;
  std::vector<double> d;
  const double* rec(int64_t i) const { return d.data() + i * kRecord; }
};

Trace load_trace(const char* path) {
  FILE* f = std::fopen(path, "rb");
  if (!f) { std::perror(path); std::exit(1); }
  Trace t;
  double header;
  if (std::fread(&header, sizeof(double), 1, f) != 1) std::exit(1);
  t.n = int64_t(header);
  t.d.resize(size_t(t.n) * kRecord);
  if (std::fread(t.d.data(), sizeof(double), t.d.size(), f) != t.d.size()) std::exit(1);
  std::fclose(f);
  return t;
}

mjModel* load_model(const char* path) {
  char err[1000] = "";
  mjModel* m = mj_loadModel(path, nullptr);
  if (!m) { std::fprintf(stderr, "cannot load %s %s\n", path, err); std::exit(1); }
  return m;
}

struct Stats { double p50, p99, p999, max, mean; };

Stats stats(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  double sum = 0;
  for (double x : v) sum += x;
  auto at = [&](double q) { return v[std::min(v.size() - 1, size_t(q * v.size()))]; };
  return {at(0.5), at(0.99), at(0.999), v.back(), sum / v.size()};
}

void print_stats(const char* name, const Stats& s, bool comma = true) {
  std::printf("\"%s\": {\"p50\": %.3f, \"p99\": %.3f, \"p999\": %.3f, \"max\": %.3f, \"mean\": %.3f}%s",
              name, s.p50, s.p99, s.p999, s.max, s.mean, comma ? ", " : "");
}

Vec7 home_q() {
  Vec7 q;
  q << 0, 0, 0, -1.57079, 0, 1.57079, -0.7853;
  return q;
}

// The M1 force-control task. Reads the same data fields in the same order as m1_press.py, so it sees the same
// state (including MuJoCo's kinematics left from before the integrator ran).
class PressTask {
 public:
  explicit PressTask(const mjModel* m) : m_(m), d_(mj_makeData(m)) {
    mj_resetDataKeyframe(m_, d_, mj_name2id(m_, mjOBJ_KEY, "home"));
    for (int i = 0; i < 7; ++i) d_->ctrl[i] = 0.0;
    mj_forward(m_, d_);
    d_->ctrl[7] = 255;
    probe_ = mj_name2id(m_, mjOBJ_SITE, "probe");
    ft_ = mj_name2id(m_, mjOBJ_SITE, "ft");
    ft_adr_ = m_->sensor_adr[mj_name2id(m_, mjOBJ_SENSOR, "ft_force")];
    dt_ = m_->opt.timestep;
    tare_steps_ = int64_t(kTareTime / dt_);
    p0_ = Eigen::Map<const Vec3>(d_->site_xpos + 3 * probe_);
    mju_mat2Quat(quat0_, d_->site_xmat + 9 * probe_);
    jacp_.resize(3 * m_->nv);
    jacr_.resize(3 * m_->nv);
    fullM_.resize(m_->nv * m_->nv);
    q_rest_ = home_q();
  }
  ~PressTask() { mj_deleteData(d_); }

  // Computes the torque for tick `step`, without stepping the physics.
  Vec7 control(int64_t step) {
    const double t = step * dt_;
    if (step == tare_steps_) tare_ = raw_world();
    const double z_des = p0_(2) - kVApproach * std::max(0.0, t - kTareTime);

    Inputs in;
    in.pos = Eigen::Map<const Vec3>(d_->site_xpos + 3 * probe_);
    mju_mat2Quat(in.quat, d_->site_xmat + 9 * probe_);
    in.pos_des << p0_(0), p0_(1), z_des;
    std::memcpy(in.quat_des, quat0_, sizeof(quat0_));

    const double f_est = step >= tare_steps_ ? -(raw_world() - tare_)(2) : 0.0;
    if (!switched_ && step > tare_steps_ && f_est > kContactThresh) {
      switched_ = true;
      pi_.reset(std::max(0.0, kKPos * (in.pos(2) - z_des)), t);
    }
    in.has_override_z = switched_;
    in.override_z = switched_ ? pi_.wrench_z(f_est, t, dt_) : 0.0;

    mj_jacSite(m_, d_, jacp_.data(), jacr_.data(), probe_);
    const int nv = m_->nv;
    for (int c = 0; c < 7; ++c)
      for (int r = 0; r < 3; ++r) {
        in.J(r, c) = jacp_[r * nv + c];
        in.J(r + 3, c) = jacr_[r * nv + c];
      }
    mj_fullM(m_, d_, fullM_.data());
    for (int r = 0; r < 7; ++r)
      for (int c = 0; c < 7; ++c) in.M(r, c) = fullM_[r * nv + c];
    for (int i = 0; i < 7; ++i) {
      in.q(i) = d_->qpos[i];
      in.qd(i) = d_->qvel[i];
      in.bias(i) = d_->qfrc_bias[i];
    }
    return impedance_torque(in, q_rest_, Gains{});
  }

  void apply_and_step(const Vec7& tau) {
    for (int i = 0; i < 7; ++i) d_->ctrl[i] = tau(i);
    mj_step(m_, d_);
  }

 private:
  Vec3 raw_world() const {
    const double* R = d_->site_xmat + 9 * ft_;
    const double* f = d_->sensordata + ft_adr_;
    return {R[0] * f[0] + R[1] * f[1] + R[2] * f[2],
            R[3] * f[0] + R[4] * f[1] + R[5] * f[2],
            R[6] * f[0] + R[7] * f[1] + R[8] * f[2]};
  }

  const mjModel* m_;
  mjData* d_;
  int probe_, ft_, ft_adr_;
  double dt_;
  int64_t tare_steps_;
  Vec3 p0_, tare_ = Vec3::Zero();
  double quat0_[4];
  std::vector<mjtNum> jacp_, jacr_, fullM_;
  Vec7 q_rest_;
  ForcePI pi_;
  bool switched_ = false;
};

int run_law(const char* trace_path) {
  const Trace tr = load_trace(trace_path);
  const Vec7 q_rest = home_q();
  double max_diff = 0.0;
  int64_t worst = 0;
  std::vector<double> us;
  us.reserve(tr.n);
  for (int64_t i = 0; i < tr.n; ++i) {
    const double* r = tr.rec(i);
    Inputs in;
    in.pos = Eigen::Map<const Vec3>(r);
    std::memcpy(in.quat, r + 3, 4 * sizeof(double));
    in.J = Eigen::Map<const Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>(r + 7);
    in.M = Eigen::Map<const Eigen::Matrix<double, 7, 7, Eigen::RowMajor>>(r + 49);
    in.q = Eigen::Map<const Vec7>(r + 98);
    in.qd = Eigen::Map<const Vec7>(r + 105);
    in.bias = Eigen::Map<const Vec7>(r + 112);
    in.pos_des = Eigen::Map<const Vec3>(r + 119);
    std::memcpy(in.quat_des, r + 122, 4 * sizeof(double));
    in.has_override_z = r[126] != 0.0;
    in.override_z = r[127];
    const int64_t a = now_ns();
    const Vec7 tau = impedance_torque(in, q_rest, Gains{});
    const int64_t b = now_ns();
    us.push_back((b - a) * 1e-3);
    const double diff = (tau - Eigen::Map<const Vec7>(r + 128)).cwiseAbs().maxCoeff();
    if (diff > max_diff) { max_diff = diff; worst = i; }
  }
  std::printf("{\"ticks\": %lld, \"max_abs_diff_Nm\": %.6e, \"worst_tick\": %lld, ", (long long)tr.n, max_diff, (long long)worst);
  print_stats("compute_us", stats(us), false);
  std::printf("}\n");
  return 0;
}

int run_closed(const char* model_path, const char* trace_path) {
  const Trace tr = load_trace(trace_path);
  mjModel* m = load_model(model_path);
  PressTask task(m);
  double max_diff = 0.0;
  int64_t worst = 0;
  std::vector<double> ctl_us, step_us, diff_by_second;
  double second_max = 0.0;
  for (int64_t i = 0; i < tr.n; ++i) {
    const int64_t a = now_ns();
    const Vec7 tau = task.control(i);
    const int64_t b = now_ns();
    task.apply_and_step(tau);
    const int64_t c = now_ns();
    ctl_us.push_back((b - a) * 1e-3);
    step_us.push_back((c - b) * 1e-3);
    const double diff = (tau - Eigen::Map<const Vec7>(tr.rec(i) + 128)).cwiseAbs().maxCoeff();
    if (diff > max_diff) { max_diff = diff; worst = i; }
    second_max = std::max(second_max, diff);
    if ((i + 1) % 1000 == 0) { diff_by_second.push_back(second_max); second_max = 0.0; }
  }
  std::printf("{\"ticks\": %lld, \"max_abs_diff_Nm\": %.6e, \"worst_tick\": %lld, \"max_diff_per_second\": [", (long long)tr.n, max_diff, (long long)worst);
  for (size_t i = 0; i < diff_by_second.size(); ++i) std::printf("%s%.3e", i ? ", " : "", diff_by_second[i]);
  std::printf("], ");
  print_stats("control_us", stats(ctl_us));
  print_stats("mj_step_us", stats(step_us), false);
  std::printf("}\n");
  mj_deleteModel(m);
  return 0;
}

int run_rt(const char* model_path, double seconds, const std::string& mode) {
  mjModel* m = load_model(model_path);
  PressTask task(m);
  const int64_t ticks = int64_t(seconds * 1000);
  const int64_t period = 1000000;
  std::vector<double> wake_us, ctl_us, tick_us;
  wake_us.reserve(ticks); ctl_us.reserve(ticks); tick_us.reserve(ticks);
  int64_t misses = 0, longest_run = 0, run = 0;
  int64_t sched = now_ns() + 10 * period;
  for (int64_t i = 0; i < ticks; ++i, sched += period) {
    if (mode == "sleep") {
      timespec ts{time_t(sched / 1000000000LL), long(sched % 1000000000LL)};
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    } else {
      while (now_ns() < sched) {}
    }
    const int64_t start = now_ns();
    const Vec7 tau = task.control(i);
    const int64_t mid = now_ns();
    task.apply_and_step(tau);
    const int64_t end = now_ns();
    wake_us.push_back((start - sched) * 1e-3);
    ctl_us.push_back((mid - start) * 1e-3);
    tick_us.push_back((end - sched) * 1e-3);
    if (end - sched > period) { ++misses; longest_run = std::max(longest_run, ++run); }
    else run = 0;
  }
  std::printf("{\"mode\": \"%s\", \"ticks\": %lld, \"deadline_misses\": %lld, \"longest_miss_run\": %lld, ", mode.c_str(), (long long)ticks, (long long)misses, (long long)longest_run);
  print_stats("wake_latency_us", stats(wake_us));
  print_stats("control_us", stats(ctl_us));
  print_stats("tick_end_us", stats(tick_us), false);
  std::printf("}\n");
  mj_deleteModel(m);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string cmd = argc > 1 ? argv[1] : "";
  if (cmd == "law" && argc == 3) return run_law(argv[2]);
  if (cmd == "closed" && argc == 4) return run_closed(argv[2], argv[3]);
  if (cmd == "rt" && argc == 5) return run_rt(argv[2], std::atof(argv[3]), argv[4]);
  std::fprintf(stderr, "usage: m5 law <trace> | closed <mjb> <trace> | rt <mjb> <seconds> <sleep|spin>\n");
  return 2;
}
