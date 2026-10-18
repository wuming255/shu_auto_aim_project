"""Manual desktop regression: python3 tests/debug_viewer_interaction.py.

Requires DISPLAY, GTK/X11, libX11/libXtst and a built MVS mock. Only focuses
windows created by these subprocesses; refuses to run alongside other SRM windows.
Not registered in headless CTest. Uses real PTYs and real GUI key events.
Live cases use synthetic calibration, a mock camera and a CDC PTY only.
"""
import csv
import ctypes as c
import ctypes.util
import os
from pathlib import Path
import pty
import struct
import subprocess
import tempfile
import termios
import threading
import time
import tty


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
    cases = [(f"gui_{window}_{name}", key, window, False, False, "mvs")
             for window in ("detection", "brightness mask")
             for name, key in (("q", ord("q")), ("Q", ord("Q")), ("esc", 0xff1b))]
    cases += [(f"terminal_{name}{'_paused' if paused else ''}", key, None, paused, False,
               "image" if paused else "mvs")
               for paused in (False, True)
               for name, key in (("q", ord("q")), ("Q", ord("Q")), ("esc", 27))]
    cases += [(f"live_{source}_{method}", ord("q"), window, False, True, source)
              for source in ("mvs", "image", "video")
              for method, window in (("gui", "detection"), ("terminal", None))]
    for name, key, window, paused, live, source in cases:
        case = output / name
        case.mkdir()
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        args = [str(root / "build/srm_auto_aim_app"), "--live" if live else "--dry-run",
                "--show-debug", "--summary", str(case / "summary.txt")]
        if source == "image":
            args += ["--input", "image", "--source", str(root / "tests/data/application_smoke.ppm")]
            if live:
                args += ["--loop"]
            width, height = 64, 48
        elif source == "video":
            args += ["--input", "video", "--source", str(output / "terminal_q/record.avi"), "--loop"]
            width, height = 2, 2
        else:
            args += ["--input", "mvs", "--mvs-library", str(root / "build/libsrm_mvs_mock.so")]
            width, height = 2, 2
        serial_master = serial_slave = None
        serial_stop = threading.Event()
        commands, serial_errors = [], []
        serial_worker = None
        if live:
            geometry = f"schema_version: 1\nimage_width: {width}\nimage_height: {height}\norientation_deg: 0\n"
            (case / "camera.yml").write_text("%YAML:1.0\n---\n" + geometry +
                "camera_matrix: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
                f"   data: [80.,0.,{width / 2},0.,80.,{height / 2},0.,0.,1.]\n"
                "distortion_coefficients: [0.,0.,0.,0.,0.]\n")
            (case / "extrinsics.yml").write_text("%YAML:1.0\n---\n" + geometry +
                "camera_frame: opencv_processed\ngimbal_frame: x_forward_y_left_z_up\ntranslation_unit: m\n"
                "rotation_camera_to_gimbal: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n"
                "   data: [0.,0.,1.,-1.,0.,0.,0.,-1.,0.]\ntranslation_camera_to_gimbal: [0.,0.,0.]\n")
            serial_master, serial_slave = pty.openpty()
            tty.setraw(serial_slave)
            os.set_blocking(serial_master, False)
            args += ["--device", os.ttyname(serial_slave), "--calibration", str(case / "camera.yml"),
                     "--extrinsics", str(case / "extrinsics.yml"), "--log", str(case / "run.csv"),
                     "--fixed-bullet-speed", "22", "--io-timeout-ms", "1"]

            def serial_peer():
                pending = bytearray()
                try:
                    while not serial_stop.is_set():
                        os.write(serial_master, struct.pack("<hhfffiihf", 28, 1, 12., -3., 0., 0, 1, 2, 22.))
                        try:
                            pending.extend(os.read(serial_master, 4096))
                        except BlockingIOError:
                            pass
                        while len(pending) >= 18:
                            frame = bytes(pending[:18])
                            del pending[:18]
                            length, id1, yaw, pitch, id2, fire = struct.unpack("<hhffhi", frame)
                            assert (length, id1, id2, fire) == (16, 1, 2, 0), frame.hex()
                            commands.append((yaw, pitch, fire))
                        serial_stop.wait(.005)
                except Exception as exc:
                    serial_errors.append(exc)

            serial_worker = threading.Thread(target=serial_peer, daemon=True)
            serial_worker.start()
        else:
            args += ["--record", str(case / "record.avi"), "--record-codec", "FFV1"]
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
                if live:
                    def completed_frames():
                        with (case / "run.csv").open(newline="") as log:
                            return [row for row in csv.DictReader(log) if row.get("frame_read") == "1"
                                    and None not in row and all(v is not None for v in row.values())]

                    for character in (ord(" "), ord("n")):
                        target = next(w for w, title in matches.items() if title == "SRM detection")
                        x.XSetInputFocus(display, target, 1, 0)
                        code = x.XKeysymToKeycode(display, character)
                        xtest.XTestFakeKeyEvent(display, code, 1, 0)
                        xtest.XTestFakeKeyEvent(display, code, 0, 0)
                        x.XFlush(display)
                        time.sleep(.1)
                        before = len(completed_frames())
                        time.sleep(.2)
                        assert process.poll() is None and len(completed_frames()) > before, "live preview paused"
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
                if live:
                    assert int(summary["transport_sent_frame_count"]) > 0
                    assert int(summary["transport_received_frame_count"]) > 0
                    assert summary["transport_communication_error_count"] == "0"
                    assert commands and not serial_errors, serial_errors
                    assert commands[-1] == (0., 0., 0), "missing safe shutdown command"
                    assert int(summary["gui_display_sample_count"]) > 0
                    assert all(row["fire_flag"] == "0" for row in completed_frames())
                else:
                    assert summary["transport_sent_frame_count"] == "0"
                    assert (case / "record.avi").stat().st_size > 0
                    assert (case / "record.avi.metadata.yml").exists()
                assert summary["error_count"] == "0"
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
                serial_stop.set()
                if serial_worker:
                    serial_worker.join(timeout=1)
                if serial_master is not None:
                    os.close(serial_master)
                    os.close(serial_slave)
    print("Artifacts:", output)


if __name__ == "__main__":
    main()
