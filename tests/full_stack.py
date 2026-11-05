"""Real rbnx build/boot + Atlas + two managed raw-sensor primitives + localization + Nav2.

No old outdoorNav repository is mounted or imported. Docker only; PTYs only.
"""

import json, math, os, signal, subprocess, sys, time
from pathlib import Path
from urllib.request import Request, urlopen
import yaml

if not Path("/.dockerenv").exists():
    raise SystemExit("Docker only")
ROOT = Path(__file__).resolve().parents[1]
PROJECTS = ROOT.parent
NAV = PROJECTS / "service-navigation-outdoor-rbnx"
sys.path[:0] = [
    str(NAV),
    str(NAV / "rbnx-build/codegen/proto_gen"),
    str(NAV / "rbnx-build/codegen/robonix_mcp_types"),
]
os.environ.update(
    ROS_DOMAIN_ID="219",
    ROS_LOCALHOST_ONLY="1",
    RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
    ROBONIX_ATLAS="127.0.0.1:51051",
)
RUN = ROOT / "rbnx-build/full-stack" / time.strftime("%Y%m%d-%H%M%S")
RUN.mkdir(parents=True)
PYTHON = "/tmp/stack-venv/bin/python"
origin = {"latitude": 30.22413029, "longitude": 120.25979315833334}
latitude = math.radians(origin["latitude"])
e2 = 6.69437999014e-3
den = math.sqrt(1 - e2 * math.sin(latitude) ** 2)
east = 6378137 / den * math.cos(latitude)
north = 6378137 * (1 - e2) / den**3
fixture = {
    "run_dir": str(RUN),
    "origin": origin,
    "east_radius": east,
    "north_radius": north,
}
localizer = {
    "origin": origin,
    "provider_ids": {"gnss": "g70_gnss", "imu": "wit_imu"},
    "calibrated": True,
    "antenna_offset_confirmed": True,
    "feedback_confirmed": True,
    "velocity_accuracy_mps": 0.02,
    "velocity_accuracy_confirmed": True,
    "imu": {
        "base_from_imu_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "mounting_confirmed": True,
        "reference_confirmed": True,
        "magnetic_declination_rad": 0.0,
        "quality_confirmed": True,
        "yaw_std_deg": 1.0,
    },
    "output_topics": {
        "odom": "/stack/odom",
        "pose": "/stack/pose",
        "status": "/stack/localization",
        "tf": "/stack/tf",
        "tf_static": "/stack/tf_static",
    },
    "run_dir": str(RUN / "localizer"),
    "http_port": 19980,
    "worker_python": PYTHON,
}
nav = {
    "params_file": str(NAV / "config/nav2_params.example.yml"),
    "provider_ids": {"cloud": "virtual_inputs", "localization": "localization_outdoor"},
    "topic_remap": {"tf": "/stack/tf", "tf_static": "/stack/tf_static"},
    "velocity_output_topic": "/stack/virtual_cmd_vel",
    "http_port": 19985,
    "global_grid": {
        "origin": [-10.0, -10.0],
        "width": 40.0,
        "height": 40.0,
        "resolution": 0.1,
    },
    "run_dir": str(RUN / "navigation"),
    "policy": {
        "motion_enabled": True,
        "calibrated": True,
        "antenna_offset_confirmed": True,
        "footprint_confirmed": True,
        "cloud_frame_confirmed": True,
        "feedback_confirmed": True,
        "require_chassis_ready": False,
    },
}
manifest = {
    "name": "independent-outdoor-stack",
    "system": {
        "atlas": {"listen": "127.0.0.1:51051"},
        "soma": {"listen": "127.0.0.1:51091"},
    },
    "primitive": [
        {
            "name": "virtual_inputs",
            "path": str(ROOT / "tests/virtual_inputs"),
            "config": fixture,
        },
        {
            "name": "g70_gnss",
            "path": str(PROJECTS / "primitive-g70-gnss-rbnx"),
            "manifest": "package_manifest.namespace-driver.yaml",
            "config": {
                "serial": {"port": str(RUN / "dev/g70"), "baud": 115200},
                "require_velocity": True,
            },
        },
        {
            "name": "wit_imu",
            "path": str(PROJECTS / "primitive-wit-imu-rbnx"),
            "manifest": "package_manifest.namespace-driver.yaml",
            "config": {"serial": {"port": str(RUN / "dev/wit"), "baud": 9600}},
        },
    ],
    "service": [
        {"name": "localization_outdoor", "path": str(ROOT), "config": localizer},
        {"name": "outdoor_execution", "path": str(NAV), "config": nav},
    ],
}
file = RUN / "robonix_manifest.yaml"
file.write_text(yaml.safe_dump(manifest, sort_keys=False))
(RUN / "robot.urdf").write_text(
    '<robot name="virtual_outdoor"><link name="base_link"/></robot>'
)
(RUN / "soma.yaml").write_text(
    yaml.safe_dump(
        {
            "urdf": {"path": "robot.urdf", "root_link": "base_link"},
            "robot": {
                "id": "virtual_outdoor",
                "display_name": "Virtual outdoor test",
                "footprint": {
                    "base_frame": "base_link",
                    "points": [[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]],
                },
            },
        }
    )
)
env = {
    **os.environ,
    "INDEPENDENT_STACK_RUN": str(RUN),
    "PATH": "/tmp/stack-venv/bin:/usr/local/bin:" + os.environ["PATH"],
    "G70_PYTHON": PYTHON,
    "WIT_PYTHON": PYTHON,
    "LOCALIZATION_PYTHON": PYTHON,
    "OUTDOOR_SERVICE_PYTHON": PYTHON,
    "RBNX_CODEGEN_PYTHON": PYTHON,
    "ROBONIX_DRIVER_INIT_TIMEOUT_S": "75",
}
report = {
    "passed": False,
    "hardware": False,
    "old_repository_imported": False,
    "framework_boot": True,
    "scenarios": [],
}
boot = None
atlas = None


def http(path, payload=None):
    req = Request(
        "http://127.0.0.1:19989" + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urlopen(req, timeout=3) as r:
        return json.load(r) if payload is None else None


def wait(fn, timeout=30):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        if boot is not None and boot.poll() is not None:
            raise RuntimeError("rbnx boot exited")
        try:
            value = fn()
            if value:
                return value
        except Exception as e:
            last = e
        time.sleep(0.1)
    raise TimeoutError(str(last or "Condition not met"))


try:
    with (RUN / "build.log").open("w") as log:
        subprocess.run(
            ["rbnx", "build", "-f", str(file)],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            check=True,
            timeout=180,
        )
    log = (RUN / "boot.log").open("w")
    boot = subprocess.Popen(
        ["rbnx", "boot", "-f", str(file), "--no-update-check", "-v"],
        env=env,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    # Imports after generation use real SDK bindings, never a fake Atlas.
    import robonix_contracts_pb2_grpc
    from robonix_api import ATLAS
    from outdoor_nav.client import ExecutionClient

    record = wait(
        lambda: ATLAS.find_unique_capability(
            contract_id="robonix/service/navigation_outdoor/follow_reference",
            provider_id="outdoor_execution",
            transport="grpc",
        ),
        100,
    )
    channel = ATLAS.connect_capability(
        consumer_id="independent_stack_test",
        provider_id=record.provider_id,
        contract_id="robonix/service/navigation_outdoor/follow_reference",
        transport="grpc",
    )
    client = ExecutionClient(channel.endpoint)
    wait(lambda: client.status()["ready"], 60)
    report["discovered_provider"] = record.provider_id

    def submit(x, tag):
        s = client.status()
        p = s["pose"]
        request = {
            "frame_id": "map",
            "reference_path": [{"x": p["x"], "y": p["y"]}, {"x": x, "y": 0.0}],
            "goal": {"x": x, "y": 0.0},
            "request_id": tag * 32,
            "arrival_tolerance_m": 3.0,
            "localization_grade": "fixed",
        }
        return client.follow_reference(request)

    first = submit(8.0, "a")
    wait(lambda: client.status(first)["state"] == "succeeded", 65)
    state = http("/state")
    assert state["commands"] > 0 and abs(state["v"]) <= 0.03
    report["scenarios"].append(
        {
            "name": "arrival_from_managed_primitives",
            "state": "succeeded",
            "x": state["x"],
            "commands": state["commands"],
        }
    )
    second = submit(state["x"] + 12.0, "b")
    wait(lambda: client.status(second)["state"] == "navigating", 10)
    client.cancel(second)
    wait(lambda: client.status(second)["state"] == "canceled", 20)
    assert abs(http("/state")["v"]) <= 0.03
    report["scenarios"].append({"name": "cancel", "state": "canceled"})
    third = submit(http("/state")["x"] + 12.0, "c")
    wait(lambda: client.status(third)["state"] == "navigating", 10)
    http("/fault", {"heading": True})
    wait(lambda: not client.status()["ready"], 5)
    wait(lambda: abs(http("/state")["v"]) <= 0.03, 5)
    wait(lambda: client.status(third)["state"] == "failed", 25)
    http("/fault", {"heading": False})
    wait(lambda: client.status()["ready"], 15)
    report["scenarios"].append(
        {"name": "heading_loss_measured_stop_and_recovery", "state": "passed"}
    )
    recovered = submit(http("/state")["x"] + 7.0, "d")
    wait(lambda: client.status(recovered)["state"] == "succeeded", 60)
    report["scenarios"].append({"name": "recovered_task", "state": "succeeded"})
    assert all(k not in sys.modules for k in ("outdoornav", "outdoornav_ros"))
    report["last_state"] = http("/state")
    report["passed"] = True
    client.close()
    channel.close()
finally:
    for name, url in (
        ("localization", "http://127.0.0.1:19980/status"),
        ("navigation", "http://127.0.0.1:19985/api/status"),
    ):
        try:
            with urlopen(url, timeout=2) as response:
                report[name + "_snapshot"] = json.load(response)
        except Exception as error:
            report[name + "_snapshot"] = {"error": str(error)}
    if boot is not None and boot.poll() is None:
        os.killpg(boot.pid, signal.SIGINT)
        try:
            boot.wait(timeout=50)
        except subprocess.TimeoutExpired:
            os.killpg(boot.pid, signal.SIGTERM)
            boot.wait(timeout=10)
    if atlas is not None and atlas.poll() is None:
        os.killpg(atlas.pid, signal.SIGTERM)
        atlas.wait(timeout=8)
    # Framework startup errors can leave nested package shells. Only this
    # exact inherited run marker authorizes cleanup; never kill generic ROS names.
    marker = ("INDEPENDENT_STACK_RUN=" + str(RUN)).encode()
    remaining = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (
                marker in (proc / "environ").read_bytes().split(b"\0")
                and (proc / "stat").read_text().split()[2] != "Z"
            ):
                remaining.append(int(proc.name))
                os.kill(int(proc.name), signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass
    deadline = time.monotonic() + 4
    while remaining and time.monotonic() < deadline:
        alive = []
        for pid in remaining:
            try:
                if Path(f"/proc/{pid}/stat").read_text().split()[2] != "Z":
                    alive.append(pid)
            except FileNotFoundError:
                pass
        remaining = alive
        if remaining:
            time.sleep(0.1)
    for pid in remaining:
        try:
            if marker in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0"):
                os.kill(pid, signal.SIGKILL)
        except (OSError, ProcessLookupError):
            pass
    report["marked_processes_requiring_final_kill"] = remaining
    report["boot_returncode"] = boot.returncode if boot else None
    report["system_mode"] = (
        "rbnx boot: real Atlas + Soma + managed primitives + services; no Scene or real hardware"
    )
    (RUN / "report.json").write_text(json.dumps(report, indent=2))
    print("INDEPENDENT_STACK_REPORT", json.dumps(report), str(RUN), flush=True)
