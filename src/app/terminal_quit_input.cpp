#include "app/terminal_quit_input.hpp"

#include <cerrno>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>

namespace srm::app
{
TerminalQuitInput::TerminalQuitInput(int fd) : fd_(fd)
{
  if (!::isatty(fd_)) return; // Never consume redirected input or a pipeline.
  const auto foreground = ::tcgetpgrp(fd_);
  if (foreground >= 0 && foreground != ::getpgrp()) return;
  if (::tcgetattr(fd_, &original_) != 0) throw std::runtime_error("cannot read debug terminal settings");
  auto settings = original_;
  settings.c_lflag &= ~(ICANON | ECHO);
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (::tcsetattr(fd_, TCSANOW, &settings) != 0) throw std::runtime_error("cannot enable debug terminal keys");
  active_ = true;
}

TerminalQuitInput::~TerminalQuitInput()
{
  if (active_) {
    while (::tcsetattr(fd_, TCSANOW, &original_) != 0 && errno == EINTR) {}
  }
}

bool TerminalQuitInput::quit_requested()
{
  if (!active_ || quit_) return quit_;
  const auto foreground = ::tcgetpgrp(fd_);
  if (foreground >= 0 && foreground != ::getpgrp()) return false;
  pollfd input{fd_, POLLIN, 0};
  if (::poll(&input, 1, 0) <= 0 || !(input.revents & POLLIN)) return false;
  char buffer[64];
  const auto count = ::read(fd_, buffer, sizeof(buffer));
  for (ssize_t i = 0; i < count; ++i) {
    if (buffer[i] == 'q' || buffer[i] == 'Q' || buffer[i] == '\x1b') quit_ = true;
  }
  return quit_;
}
}  // namespace srm::app
