"""Manual desktop regression: python3 tests/debug_viewer_interaction.py.

Requires DISPLAY, GTK/X11, libX11/libXtst and a built MVS mock. Only focuses
windows created by these subprocesses; refuses to run alongside other SRM windows.
Not registered in headless CTest. Uses a real PTY and real GUI key events.
"""
import ctypes as c
import ctypes.util
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import termios
import time


def main():
    root = Path(__file__).resolve().parents[1]
    x = c.CDLL(ctypes.util.find_library("X11"))
    xtest = c.CDLL(ctypes.util.find_library("Xtst"))
    x.XOpenDisplay.restype = c.c_void_p
    display = x.XOpenDisplay(None)
    assert display, "requires an accessible X11 display"
    # Desktop windows can disappear between QueryTree and FetchName.
    error_callback = c.CFUNCTYPE(c.c_int, c.c_void_p, c.c_void_p)(lambda *_: 0)
    x.XSetErrorHandler.argtypes = [c.c_void_p]
    x.XSetErrorHandler(error_callback)
    x.XDefaultRootWindow.argtypes = [c.c_void_p]
    x.XDefaultRootWindow.restype = c.c_ulong
    x.XQueryTree.argtypes = [c.c_void_p, c.c_ulong, c.POINTER(c.c_ulong),
                            c.POINTER(c.c_ulong), c.POINTER(c.POINTER(c.c_ulong)), c.POINTER(c.c_uint)]
    x.XFetchName.argtypes = [c.c_void_p, c.c_ulong, c.POINTER(c.c_char_p)]
    x.XFree.argtypes = [c.c_void_p]
    x.XSetInputFocus.argtypes = [c.c_void_p, c.c_ulong, c.c_int, c.c_ulong]
    x.XFlush.argtypes = [c.c_void_p]
    x.XKeysymToKeycode.argtypes = [c.c_void_p, c.c_ulong]
    xtest.XTestFakeKeyEvent.argtypes = [c.c_void_p, c.c_uint, c.c_int, c.c_ulong]

    def windows(window=None):
        if window is None:
            window = x.XDefaultRootWindow(display)
        parent, tree_root = c.c_ulong(), c.c_ulong()
        children, count = c.POINTER(c.c_ulong)(), c.c_uint()
        result, name = {}, c.c_char_p()
        if x.XFetchName(display, window, c.byref(name)) and name.value:
            result[window] = name.value.decode(errors="replace")
            x.XFree(name)
        if x.XQueryTree(display, window, c.byref(tree_root), c.byref(parent), c.byref(children), c.byref(count)):
            ids = [children[i] for i in range(count.value)]
            if children:
                x.XFree(children)
            for child in ids:
                result.update(windows(child))
        return result

    def srm_windows():
        return {w: name for w, name in windows().items() if name in ("SRM detection", "SRM brightness mask")}

    assert not srm_windows(), "existing SRM windows; close them before running this test"
    output = Path(tempfile.mkdtemp(prefix="srm-debug-keys-"))
    cases = [(f"gui_{window}_{name}", key, window, False)
             for window in ("detection", "brightness mask")
             for name, key in (("q", ord("q")), ("Q", ord("Q")), ("esc", 0xff1b))]
    cases += [(f"terminal_{name}{'_paused' if paused else ''}", key, None, paused)
              for paused in (False, True)
              for name, key in (("q", ord("q")), ("Q", ord("Q")), ("esc", 27))]
    for name, key, window, paused in cases:
        case = output / name
        case.mkdir()
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        args = [str(root / "build/srm_auto_aim_app"), "--dry-run", "--show-debug",
                "--summary", str(case / "summary.txt"), "--record", str(case / "record.avi"),
                "--record-codec", "FFV1"]
        if paused:
            args += ["--input", "image", "--source", str(root / "tests/data/application_smoke.ppm")]
        else:
            args += ["--input", "mvs", "--mvs-library", str(root / "build/libsrm_mvs_mock.so")]
        with (case / "stderr.txt").open("w") as err:
            process = subprocess.Popen(args, stdin=slave, stdout=err, stderr=err,
                                       env=dict(os.environ, GDK_BACKEND="x11"))
            try:
                deadline = time.monotonic() + 8
                while time.monotonic() < deadline:
                    matches = srm_windows()
                    if len(matches) == 2:
                        break
                    assert process.poll() is None, (case / "stderr.txt").read_text()
                    time.sleep(.05)
                assert len(matches) == 2, "preview windows did not appear"
                time.sleep(.3)
                if window:
                    target = next(w for w, title in matches.items() if title == "SRM " + window)
                    x.XSetInputFocus(display, target, 1, 0)
                    code = x.XKeysymToKeycode(display, key)
                    assert xtest.XTestFakeKeyEvent(display, code, 1, 0)
                    assert xtest.XTestFakeKeyEvent(display, code, 0, 0)
                    x.XFlush(display)
                else:
                    os.write(master, bytes([key]))  # No newline: terminal keys must be immediate.
                assert process.wait(timeout=5) == 0
                summary = dict(line.split("=", 1) for line in (case / "summary.txt").read_text().splitlines())
                assert int(summary["processed_frames"]) > 0
                assert summary["nonzero_fire_request_count"] == "0"
                assert summary["transport_sent_frame_count"] == "0"
                assert summary["error_count"] == "0"
                assert (case / "record.avi").stat().st_size > 0
                assert (case / "record.avi.metadata.yml").exists()
                assert not list(case.glob("*.partial-*.avi"))
                assert termios.tcgetattr(slave) == original, "terminal settings not restored"
                assert not srm_windows(), "preview windows remain after exit"
                print(name, "PASS", "frames=" + summary["processed_frames"], flush=True)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                os.close(slave)
                os.close(master)
    print("Artifacts:", output)


if __name__ == "__main__":
    main()
