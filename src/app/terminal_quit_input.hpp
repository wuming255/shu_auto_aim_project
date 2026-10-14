#ifndef SHU_AUTO_AIM_TERMINAL_QUIT_INPUT_HPP
#define SHU_AUTO_AIM_TERMINAL_QUIT_INPUT_HPP

#include <termios.h>

namespace srm::app
{
// Only used by the optional debug viewer. Leave signals enabled and restore the TTY on exit.
class TerminalQuitInput final
{
public:
  explicit TerminalQuitInput(int fd = 0);
  ~TerminalQuitInput();
  TerminalQuitInput(const TerminalQuitInput &) = delete;
  TerminalQuitInput & operator=(const TerminalQuitInput &) = delete;
  bool quit_requested();

private:
  int fd_;
  termios original_{};
  bool active_ = false;
  bool quit_ = false;
};
}  // namespace srm::app
#endif
