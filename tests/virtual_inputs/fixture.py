"""Only PTYs and simulated kinematics. Never opens a real serial port."""

import json, math, os, pty, signal, struct, subprocess, sys, threading, time, tty
from pathlib import Path

if not Path("/.dockerenv").exists():
    raise SystemExit("Docker only")
from robonix_api import Primitive, Ok, Err

primitive = Primitive(
    id=os.environ.get("RBNX_INSTANCE_NAME", "virtual_inputs"),
    namespace="robonix/primitive/lidar",
)
process = None


@primitive.on_init
def init(cfg):
    global process
    try:
        file = Path(cfg["run_dir"]) / "fixture.json"
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(json.dumps(cfg))
        process = subprocess.Popen(
            [sys.executable, __file__, "--worker", str(file)], stdin=subprocess.DEVNULL
        )
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Virtual input worker exited")
            if (file.parent / "dev/g70").exists() and (
                file.parent / "dev/wit"
            ).exists():
                break
            time.sleep(0.05)
        else:
            raise TimeoutError("Virtual PTYs unavailable")
        primitive.declare_ros2_topic(
            "robonix/primitive/lidar/lidar3d", "/stack/front_cloud", qos="reliable"
        )
        return Ok()
    except Exception as e:
        return Err(str(e))


@primitive.on_shutdown
def shutdown():
    if process and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    return Ok()


def worker(file):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    cfg = json.loads(Path(file).read_text())
    run = Path(cfg["run_dir"])
    dev = run / "dev"
    dev.mkdir(exist_ok=True)
    masters = []
    slaves = []
    for name in ("g70", "wit"):
        master, slave = pty.openpty()
        tty.setraw(slave)
        masters.append(master)
        slaves.append(slave)
        os.set_blocking(master, False)
        link = dev / name
        if link.is_symlink():
            link.unlink()
        os.symlink(os.ttyname(slave), link)

    class Plant(Node):
        def __init__(self):
            super().__init__("raw_sensor_simulator")
            self.x = self.y = self.yaw = 0.0
            self.v = self.w = 0.0
            self.target = (0.0, 0.0)
            self.at = 0.0
            self.last = time.monotonic()
            self.steps = 0
            self.commands = 0
            self.faults = {"gnss": False, "heading": False, "gyro": False}
            self.closed_ring = False
            self.pub = self.create_publisher(PointCloud2, "/stack/front_cloud", 10)
            self.create_subscription(Twist, "/stack/virtual_cmd_vel", self.command, 10)
            self.fields = [
                PointField(name=a, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                for i, a in enumerate(("x", "y", "z"))
            ]
            self.create_timer(0.05, self.tick)

        def command(self, m):
            self.target = (m.linear.x, m.angular.z)
            self.at = time.monotonic()
            if abs(m.linear.x) + abs(m.angular.z) > 1e-6:
                self.commands += 1

        def tick(self):
            now = time.monotonic()
            dt = min(0.1, now - self.last)
            self.last = now
            self.steps += 1
            target = self.target if now - self.at < 0.5 else (0.0, 0.0)
            self.v += max(-0.3 * dt, min(0.3 * dt, target[0] - self.v))
            self.w += max(-0.6 * dt, min(0.6 * dt, target[1] - self.w))
            self.yaw = math.atan2(
                math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt)
            )
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt

            def frame(kind, values):
                data = bytes((0x55, kind)) + struct.pack("<hhhh", *values)
                return data + bytes([sum(data) & 255])

            payload = frame(0x51, [0, 0, 2048, 0])
            if not self.faults["gyro"]:
                payload += frame(
                    0x52, [0, 0, round(math.degrees(self.w) / 2000 * 32768), 0]
                )
            if not self.faults["heading"]:
                payload += frame(0x53, [0, 0, round(self.yaw / math.pi * 32768), 0])
            try:
                os.write(masters[1], payload)
            except BlockingIOError:
                pass
            if self.steps % 2 == 0 and not self.faults["gnss"]:
                latitude = cfg["origin"]["latitude"] + math.degrees(
                    self.y / cfg["north_radius"]
                )
                longitude = cfg["origin"]["longitude"] + math.degrees(
                    self.x / cfg["east_radius"]
                )

                def ll(v):
                    v = abs(v)
                    d = int(v)
                    return f"{d * 100 + (v - d) * 60:.7f}"

                epoch = 43200 + self.steps * 0.05
                h = int(epoch // 3600) % 24
                m = int(epoch // 60) % 60
                s = epoch % 60
                utc = f"{h:02d}{m:02d}{s:05.2f}"
                speed = abs(self.v) * 3600 / 1852
                course = (90 - math.degrees(self.yaw)) % 360
                lines = [
                    f"GNGGA,{utc},{ll(latitude)},N,{ll(longitude)},E,4,20,0.8,0,M,0,M,1,0001",
                    f"GNRMC,{utc},A,{ll(latitude)},N,{ll(longitude)},E,{speed:.7f},{course:.5f},051126,,,A",
                    f"GNGST,{utc},1,1,1,0,0.1,0.1,0.15",
                ]
                for body in lines:
                    check = 0
                    for b in body.encode():
                        check ^= b
                    try:
                        os.write(masters[0], f"${body}*{check:02X}\r\n".encode())
                    except BlockingIOError:
                        pass
            distance = 4.0 if self.closed_ring else 7.5
            rays = [
                (
                    distance * math.cos(i * math.pi / 360),
                    distance * math.sin(i * math.pi / 360),
                    0.0,
                )
                for i in range(720)
            ]
            self.pub.publish(
                point_cloud2.create_cloud(
                    Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"),
                    self.fields,
                    rays,
                )
            )
            if self.count_publishers("/cmd_vel") or self.count_publishers("/NAV_CMD"):
                raise RuntimeError("Canonical motion publisher in isolated test")

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    plant = Plant()
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            data = json.dumps(
                {
                    "x": plant.x,
                    "y": plant.y,
                    "yaw": plant.yaw,
                    "v": plant.v,
                    "w": plant.w,
                    "commands": plant.commands,
                    "faults": plant.faults,
                }
            ).encode()
            self.send_response(200)
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            value = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            if self.path == "/fault":
                plant.faults.update(value)
            elif self.path == "/blockage":
                plant.closed_ring = bool(value.get("blocked"))
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.end_headers()

        def log_message(self, *_):
            pass

    http = ThreadingHTTPServer(("127.0.0.1", 19989), Handler)
    threading.Thread(target=http.serve_forever, daemon=True).start()
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stopping:
            rclpy.spin_once(plant, timeout_sec=0.05)
    finally:
        http.shutdown()
        http.server_close()
        plant.destroy_node()
        rclpy.try_shutdown()
        for fd in masters + slaves:
            os.close(fd)


if __name__ == "__main__":
    if "--worker" in sys.argv:
        worker(sys.argv[sys.argv.index("--worker") + 1])
    else:
        primitive.run()
