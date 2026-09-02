#include "genisom_l1_control/process_supervisor.hpp"

#include <sys/prctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

namespace genisom_l1_control
{
namespace
{

constexpr auto kPollPeriod = std::chrono::milliseconds(20);
constexpr auto kShutdownGracePeriod = std::chrono::seconds(3);
volatile std::sig_atomic_t shutdown_signal = 0;
volatile std::sig_atomic_t worker_supervisor_pid = 0;

void record_shutdown_signal(int signal_number)
{
  shutdown_signal = signal_number;
}

bool set_signal_action(int signal_number, void (* handler)(int))
{
  struct sigaction action {};
  action.sa_handler = handler;
  sigemptyset(&action.sa_mask);
  return sigaction(signal_number, &action, nullptr) == 0;
}

void wait_for_child(pid_t child_pid, int & child_status)
{
  while (waitpid(child_pid, &child_status, 0) < 0 && errno == EINTR) {
  }
}

int child_exit_code(int child_status, int forwarded_signal)
{
  if (WIFEXITED(child_status)) {
    return WEXITSTATUS(child_status);
  }
  if (WIFSIGNALED(child_status)) {
    const int signal_number = forwarded_signal != 0 ? forwarded_signal : WTERMSIG(child_status);
    return 128 + signal_number;
  }
  return 1;
}

}  // 匿名命名空间

void request_supervised_shutdown()
{
  const auto supervisor_pid = static_cast<pid_t>(worker_supervisor_pid);
  if (supervisor_pid > 1 && getppid() == supervisor_pid) {
    static_cast<void>(kill(supervisor_pid, SIGUSR1));
  }
}

int run_with_shutdown_supervisor(int argc, char * argv[], WorkerMain worker_main)
{
  shutdown_signal = 0;
  if (!set_signal_action(SIGINT, record_shutdown_signal) ||
    !set_signal_action(SIGTERM, record_shutdown_signal) ||
    !set_signal_action(SIGUSR1, record_shutdown_signal))
  {
    std::perror("无法安装监督进程信号处理器");
    return 1;
  }

  const pid_t child_pid = fork();
  if (child_pid < 0) {
    std::perror("无法创建 SDK 工作进程");
    return 1;
  }

  if (child_pid == 0) {
    const pid_t supervisor_pid = getppid();
    worker_supervisor_pid = supervisor_pid;
    set_signal_action(SIGINT, SIG_DFL);
    set_signal_action(SIGTERM, SIG_DFL);
    set_signal_action(SIGUSR1, SIG_DFL);

    // 监督进程异常消失时同步回收工作进程，避免 UDP 端口残留占用。
    if (prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || getppid() != supervisor_pid) {
      _exit(1);
    }
    return worker_main(argc, argv);
  }

  int child_status = 0;
  int forwarded_signal = 0;
  auto force_kill_deadline = std::chrono::steady_clock::time_point::max();

  while (true) {
    const pid_t wait_result = waitpid(child_pid, &child_status, WNOHANG);
    if (wait_result == child_pid) {
      break;
    }
    if (wait_result < 0 && errno != EINTR) {
      std::perror("等待 SDK 工作进程失败");
      return 1;
    }

    if (shutdown_signal != 0 && forwarded_signal == 0) {
      // SIGUSR1 只用于工作进程通知监督进程，实际向 ROS2 子进程转发 SIGTERM。
      forwarded_signal = shutdown_signal == SIGUSR1 ? SIGTERM : shutdown_signal;
      static_cast<void>(kill(child_pid, forwarded_signal));
      force_kill_deadline = std::chrono::steady_clock::now() + kShutdownGracePeriod;
    }

    if (forwarded_signal != 0 && std::chrono::steady_clock::now() >= force_kill_deadline) {
      std::fprintf(
        stderr,
        "[监督进程] SDK 工作进程在退出信号后 3 秒仍未结束，现强制回收。\n");
      static_cast<void>(kill(child_pid, SIGKILL));
      wait_for_child(child_pid, child_status);
      break;
    }

    std::this_thread::sleep_for(kPollPeriod);
  }

  return child_exit_code(child_status, forwarded_signal);
}

}  // 命名空间 genisom_l1_control
