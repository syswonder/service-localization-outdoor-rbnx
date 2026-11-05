"""Robonix localization service; input contracts are owned by GNSS/IMU primitives."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import Request, urlopen
from robonix_api import ATLAS, Service, Ok, Err, Deferred
from .configuration import validate, CONTRACTS

service = Service(
    id=os.environ.get(
        "RBNX_INSTANCE_NAME",
        os.environ.get("ROBONIX_CAPABILITY_ID", "localization_outdoor"),
    ),
    namespace="robonix/service/localization",
)
channels = {}
worker = None
config = None
log = None


def call(path, payload=None):
    if worker is None or worker.poll() is not None:
        raise RuntimeError("Localization worker is not running")
    req = Request(
        f"http://127.0.0.1:{config.get('http_port', 19980)}" + path,
        data=None if payload is None else json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urlopen(req, timeout=3) as r:
        return json.load(r) if path == "/status" else None


def cleanup():
    global worker, log
    if worker is not None and worker.poll() is None:
        os.killpg(worker.pid, signal.SIGTERM)
        try:
            worker.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(worker.pid, signal.SIGKILL)
            worker.wait(timeout=3)
    worker = None
    if log:
        log.close()
        log = None
    for channel in channels.values():
        channel.close()
    channels.clear()


@service.on_init
def init(raw):
    global worker, config, log
    try:
        checked = validate(raw)
        if worker is not None and worker.poll() is None:
            if checked == config:
                return Ok()
            cleanup()
        bindings = dict(checked.get("topic_remap", {}))
        for role, (provider_role, contract) in CONTRACTS.items():
            if role in bindings:
                continue
            provider = checked.get("provider_ids", {}).get(provider_role, "")
            matches = ATLAS.find_capability(
                contract_id=contract, transport="ros2", provider_id=provider
            )
            if not matches:
                cleanup()
                return Deferred("Missing primitive capability: " + contract)
            channel = service.connect_capability(
                matches[0], contract_id=contract, transport="ros2"
            )
            if not channel.endpoint:
                channel.close()
                cleanup()
                return Deferred("Empty primitive endpoint: " + role)
            channels[role] = channel
            bindings[role] = channel.endpoint
        checked["topic_remap"] = bindings
        config = validate(checked)
        run = Path(
            config.get(
                "run_dir", Path(__file__).resolve().parents[1] / "rbnx-build/runtime"
            )
        ).resolve()
        run.mkdir(parents=True, exist_ok=True)
        import socket

        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", config.get("http_port", 19980)))
        file = run / "config.json"
        file.write_text(json.dumps(config, indent=2))
        log = (run / "worker.log").open("a", buffering=1)
        worker = subprocess.Popen(
            [
                config.get("worker_python", sys.executable),
                "-m",
                "localization_outdoor.node",
                "--config",
                str(file),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if worker.poll() is not None:
                raise RuntimeError(
                    "Localization worker exited; inspect " + str(run / "worker.log")
                )
            try:
                call("/status")
                break
            except OSError:
                time.sleep(0.05)
        else:
            raise TimeoutError("Localization worker did not initialize")
        for role in ("odom", "pose", "status"):
            service.declare_ros2_topic(
                "robonix/service/localization/" + role,
                config["output_topics"][role],
                qos="reliable",
            )
        return Ok()
    except Exception as e:
        cleanup()
        return Err(str(e))


@service.on_activate
def activate():
    try:
        call("/active", {"active": True})
        return Ok()
    except Exception as e:
        return Err(str(e))


@service.on_deactivate
def deactivate():
    try:
        call("/active", {"active": False})
        return Ok()
    except Exception as e:
        return Err(str(e))


@service.on_shutdown
def shutdown():
    cleanup()
    return Ok()


if __name__ == "__main__":
    service.run()
