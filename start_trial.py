#!/usr/bin/env python3
"""启动一组已经明确授权的实机实验；不执行自动重启。"""

import argparse
import json
import time
from pathlib import Path


ALLOWED_TRIAL_ID_CHARACTERS = (
    "abcdefghijklmnopqrstuvwxyz"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "0123456789_-"
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--start",
        action="store_true",
        help="explicit authorization to start the hardware controller",
    )
    parser.add_argument("--output", required=True, help="directory for bag recordings")
    args = parser.parse_args()

    if not args.start:
        parser.error("Add --start only after reviewing the experiment procedure.")

    import rosgraph
    import rospy
    from controller_manager_msgs.srv import LoadController, ListControllers, SwitchController
    from tase_hcf_rcbf.msg import TrialSample

    rospy.init_node("tase_trial_start", anonymous=True, disable_signals=True)
    params = rospy.get_param("/tase_comparison")
    commissioning = params.get("commissioning", {})

    # 仅允许加载已经明确授权的实验配置。
    if (
        not params.get("enable_hardware")
        or params.get("mode") not in ("HCF", "RCBF")
        or not commissioning.get("enabled")
        or (
            params.get("injection", {}).get("enabled")
            and not commissioning.get("allow_controlled_injection")
        )
        or params.get("velocity_bias", {}).get("enabled")
        or params.get("certificate", {}).get("enforce")
    ):
        raise RuntimeError("Invalid commissioning configuration; controller not started.")

    trial_id = params.get("trial_id", "")
    if (
        not trial_id
        or "unset" in trial_id
        or any(char not in ALLOWED_TRIAL_ID_CHARACTERS for char in trial_id)
    ):
        raise RuntimeError("Use a unique ASCII trial ID containing letters, digits, _ or -.")

    folder = Path(args.output).expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    param_file = folder / f"{trial_id}_parameters.json"
    with param_file.open("x", encoding="utf-8") as stream:
        json.dump(params, stream, indent=2, ensure_ascii=False)

    master = rosgraph.Master(rospy.get_name())

    def recorder_registered():
        _, subscribers, _ = master.getSystemState()
        registrations = dict(subscribers)
        return all(
            "/tase_record" in registrations.get(topic, [])
            for topic in (
                "/tase_comparison/sample",
                "/tase_comparison/configuration",
            )
        )

    if not recorder_registered():
        raise RuntimeError("The recorder must subscribe before starting the controller.")

    manager = "/controller_manager"
    for service_name in ("list_controllers", "load_controller", "switch_controller"):
        rospy.wait_for_service(f"{manager}/{service_name}", timeout=10)

    list_controllers = rospy.ServiceProxy(f"{manager}/list_controllers", ListControllers)
    controllers = {entry.name: entry.state for entry in list_controllers().controller}
    if controllers.get("tase_comparison") == "running":
        raise RuntimeError("The controller is already running.")

    if "tase_comparison" not in controllers:
        load_controller = rospy.ServiceProxy(f"{manager}/load_controller", LoadController)
        if not load_controller("tase_comparison").ok:
            raise RuntimeError("Controller load failed.")

    latest_sample = [None]

    def observe(message):
        latest_sample[0] = message

    subscriber = rospy.Subscriber(
        "/tase_comparison/sample",
        TrialSample,
        observe,
        queue_size=20,
    )

    time.sleep(3)
    if not recorder_registered():
        raise RuntimeError("Recorder subscription disappeared; controller not started.")

    switch_controller = rospy.ServiceProxy(f"{manager}/switch_controller", SwitchController)
    response = switch_controller(
        start_controllers=["tase_comparison"],
        stop_controllers=[],
        strictness=2,
        start_asap=False,
        timeout=5.0,
    )
    if not response.ok:
        raise RuntimeError("Controller switch failed; inspect the robot state.")

    # 等待运动阶段完成，并继续观察结束后至少两秒的保持状态。
    # 此脚本不是紧急停止装置，也不会自动恢复机器人的初始姿态。
    start_time = time.monotonic()
    duration = float(params["run_duration"])
    while time.monotonic() - start_time < duration + 15:
        sample = latest_sample[0]
        if sample is not None:
            if sample.fault_reason:
                raise RuntimeError(
                    f"Controller fault {sample.fault_reason}; inspect the robot before proceeding."
                )
            if sample.time_s >= duration + 2 and (
                params.get("hold_only") or sample.phase == 2
            ):
                subscriber.unregister()
                return
        time.sleep(0.1)

    raise RuntimeError("A complete trial was not observed; inspect the hardware.")


if __name__ == "__main__":
    try:
        main()
    except (Exception, KeyboardInterrupt) as error:
        raise SystemExit(f"Trial aborted: {error}")
