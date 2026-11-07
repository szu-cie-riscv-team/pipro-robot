#pragma once
#include <fcntl.h>
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <vector>
#include <optional>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <map>
#include <set>
#include <sched.h>
extern char **environ;

namespace k1muse_web {
// Fixed owned processes only, polled by the executor; never invokes a shell.
struct Child {
  pid_t pid{-1}; int output_fd{-1}; int exit_code{-1}; std::string output;
  Child() = default;
  Child(const Child &) = delete;
  ~Child() {
    if (pid > 0) { kill(-pid, SIGKILL); waitpid(pid, nullptr, 0); }
    if (output_fd >= 0) close(output_fd);
  }
  void start(std::vector<std::string> args, bool capture = false,
      const std::optional<std::string> &input = std::nullopt) {
    if (pid > 0) throw std::runtime_error("process already running");
    output.clear(); exit_code = -1;
    int fds[2]{-1, -1};
    if (capture && pipe2(fds, O_CLOEXEC) != 0) throw std::runtime_error("pipe failed");
    // Anonymous stdin keeps API keys out of argv and avoids a blocking pipe write.
    int input_fd = -1;
    if (input) {
      input_fd = memfd_create("k1muse-config", MFD_CLOEXEC);
      std::size_t offset = 0;
      while (input_fd >= 0 && offset < input->size()) {
        const auto count = write(input_fd, input->data() + offset, input->size() - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) { close(input_fd); input_fd = -1; break; }
        offset += static_cast<std::size_t>(count);
      }
      if (input_fd < 0 || lseek(input_fd, 0, SEEK_SET) < 0) {
        if (input_fd >= 0) close(input_fd);
        if (capture) { close(fds[0]); close(fds[1]); }
        throw std::runtime_error("helper input failed");
      }
    }
    posix_spawn_file_actions_t actions; posix_spawn_file_actions_init(&actions);
    if (input_fd >= 0) {
      posix_spawn_file_actions_adddup2(&actions, input_fd, STDIN_FILENO);
      posix_spawn_file_actions_addclose(&actions, input_fd);
    }
    if (capture) {
      posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
      posix_spawn_file_actions_addclose(&actions, fds[0]);
      posix_spawn_file_actions_addclose(&actions, fds[1]);
    }
    posix_spawnattr_t attr; posix_spawnattr_init(&attr);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attr, 0);
    std::vector<char *> argv;
    for (auto &arg : args) argv.push_back(arg.data());
    argv.push_back(nullptr);
    int error = posix_spawnp(&pid, argv[0], &actions, &attr, argv.data(), environ);
    posix_spawnattr_destroy(&attr); posix_spawn_file_actions_destroy(&actions);
    if (capture) close(fds[1]);
    if (input_fd >= 0) close(input_fd);
    if (error) {
      pid = -1; if (capture) close(fds[0]);
      throw std::runtime_error("cannot start " + args.front() + ": " + std::to_string(error));
    }
    output_fd = capture ? fds[0] : -1;
    if (capture) fcntl(output_fd, F_SETFL, O_NONBLOCK);
  }
  // Called only at Nav2 phase boundaries, never from a frame/control loop.
  void affinity(bool computing) const {
    if (pid <= 0) return;
    std::map<pid_t,pid_t> parents;
    for (const auto &entry : std::filesystem::directory_iterator("/proc")) {
      const auto name = entry.path().filename().string();
      if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos) continue;
      std::ifstream file(entry.path()/"stat"); std::string line; std::getline(file,line);
      const auto end = line.rfind(')'); if (end == std::string::npos) continue;
      std::istringstream fields(line.substr(end+1)); char state; pid_t parent;
      if (fields >> state >> parent) parents[std::stoi(name)] = parent;
    }
    std::set<pid_t> owned{pid}; bool added;
    do {
      added = false;
      for (const auto &[child,parent] : parents)
        if (owned.count(parent) && owned.insert(child).second) added = true;
    } while (added);
    cpu_set_t mask; CPU_ZERO(&mask);
    for (int cpu = computing ? 0 : 4; cpu < 8; ++cpu) CPU_SET(cpu,&mask);
    for (const auto process : owned) {
      std::error_code error;
      for (const auto &thread : std::filesystem::directory_iterator(
          std::filesystem::path("/proc")/std::to_string(process)/"task",error)) {
        if (sched_setaffinity(std::stoi(thread.path().filename().string()),sizeof(mask),&mask) && errno != ESRCH)
          throw std::runtime_error("Nav2 thread affinity failed: " + std::to_string(errno));
      }
    }
  }
  void drain() {
    if (output_fd < 0) return;
    char data[4096]; ssize_t n;
    while ((n = read(output_fd, data, sizeof(data))) > 0) {
      if (output.size() + static_cast<size_t>(n) > 262144) {
        signal(SIGKILL); throw std::runtime_error("helper output too large");
      }
      output.append(data, static_cast<size_t>(n));
    }
  }
  bool poll() {
    if (pid <= 0) return true;
    drain(); int status = 0;
    const auto waited = waitpid(pid, &status, WNOHANG);
    if (waited == 0 || (waited < 0 && errno == EINTR)) return false;
    if (waited < 0) throw std::runtime_error("waitpid failed");
    drain(); pid = -1;
    exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
    if (output_fd >= 0) { close(output_fd); output_fd = -1; }
    return true;
  }
  void signal(int sig) const {
    // ros2 launch forwards SIGINT and handles its children's shutdown timeout.
    // Sending SIGINT to the group as well interrupts each node twice.
    if (pid > 0) kill(sig == SIGINT ? pid : -pid, sig);
  }
};
}  // namespace k1muse_web
