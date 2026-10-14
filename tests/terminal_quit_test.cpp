#include "app/terminal_quit_input.hpp"

#include <cassert>
#include <pty.h>
#include <stdexcept>
#include <unistd.h>

int main()
{
  int master, slave;
  assert(::openpty(&master, &slave, nullptr, nullptr, nullptr) == 0);
  termios original{};
  assert(::tcgetattr(slave, &original) == 0);
  for (const char key : {'q', 'Q', '\x1b'}) {
    {
      srm::app::TerminalQuitInput input(slave);
      termios current{};
      assert(::tcgetattr(slave, &current) == 0);
      assert(!(current.c_lflag & (ICANON | ECHO)));
      assert((current.c_lflag & ISIG) == (original.c_lflag & ISIG));
      assert(!input.quit_requested());
      assert(::write(master, "n ", 2) == 2);
      assert(!input.quit_requested());
      assert(::write(master, &key, 1) == 1);
      // PTY input can be delivered asynchronously; bounded polling avoids a blocking read.
      bool quit = false;
      for (int i = 0; i < 100 && !quit; ++i) { quit = input.quit_requested(); ::usleep(1000); }
      assert(quit);
      assert(input.quit_requested());
    }
    termios restored{};
    assert(::tcgetattr(slave, &restored) == 0);
    assert(restored.c_lflag == original.c_lflag);
    assert(restored.c_cc[VMIN] == original.c_cc[VMIN]);
    assert(restored.c_cc[VTIME] == original.c_cc[VTIME]);
  }
  try {
    srm::app::TerminalQuitInput input(slave);
    throw std::runtime_error("unwind");
  } catch (const std::runtime_error &) {}
  termios restored{};
  assert(::tcgetattr(slave, &restored) == 0 && restored.c_lflag == original.c_lflag);
  ::close(master); ::close(slave);
  int pipefd[2];
  assert(::pipe(pipefd) == 0);
  assert(::write(pipefd[1], "q", 1) == 1);
  { srm::app::TerminalQuitInput input(pipefd[0]); assert(!input.quit_requested()); }
  char key;
  assert(::read(pipefd[0], &key, 1) == 1 && key == 'q');
  ::close(pipefd[0]); ::close(pipefd[1]);
}
