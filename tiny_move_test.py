import argparse
import socket
import time

from config import load_config


AXES = {
    "x+": [1, 0, 0, 0, 0, 0],
    "x-": [-1, 0, 0, 0, 0, 0],
    "y+": [0, 1, 0, 0, 0, 0],
    "y-": [0, -1, 0, 0, 0, 0],
    "z+": [0, 0, 1, 0, 0, 0],
    "z-": [0, 0, -1, 0, 0, 0],
}


def robot_ip(name):
    cfg = load_config()
    if name == "diagnost":
        return cfg.diagnost_ip
    if name == "surgeon":
        return cfg.surgeon_ip
    return name


def dashboard_status(host, port=29999):
    try:
        with socket.create_connection((host, port), timeout=3) as sock:
            sock.settimeout(3)
            try:
                banner = sock.recv(256).decode(errors="replace").strip()
            except socket.timeout:
                banner = ""
            sock.sendall(b"robotmode\n")
            mode = sock.recv(256).decode(errors="replace").strip()
            return banner, mode
    except Exception as exc:
        return "", f"ERROR: {type(exc).__name__}: {exc}"


def speed_vector(axis, speed):
    return [value * speed for value in AXES[axis]]


def urscript(axis, speed, duration, acceleration, move_back):
    forward = ", ".join(f"{value:.6f}" for value in speed_vector(axis, speed))
    lines = [
        "def tiny_move_test():",
        f"  speedl([{forward}], {acceleration:.6f}, {duration:.6f})",
        "  stopl(0.200000)",
        "  sleep(0.300000)",
    ]
    if move_back:
        backward = ", ".join(f"{-value:.6f}" for value in speed_vector(axis, speed))
        lines.extend([
            f"  speedl([{backward}], {acceleration:.6f}, {duration:.6f})",
            "  stopl(0.200000)",
        ])
    lines.append("end")
    return "\n".join(lines) + "\n"


def send_program(host, port, program):
    with socket.create_connection((host, port), timeout=3) as sock:
        sock.sendall(program.encode("utf-8"))
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Tiny direct URScript movement test without urx or joystick.")
    parser.add_argument("robot", choices=["diagnost", "surgeon"], help="Robot from robot_config.json")
    parser.add_argument("--axis", choices=sorted(AXES), default="z+", help="Base-coordinate movement axis")
    parser.add_argument("--speed", type=float, default=0.005, help="Linear speed in m/s")
    parser.add_argument("--duration", type=float, default=0.5, help="Move duration in seconds")
    parser.add_argument("--acceleration", type=float, default=0.05, help="Acceleration in m/s^2")
    parser.add_argument("--no-return", action="store_true", help="Do not move back after the test move")
    parser.add_argument("--execute", action="store_true", help="Actually send the movement program")
    args = parser.parse_args()

    host = robot_ip(args.robot)
    program = urscript(args.axis, args.speed, args.duration, args.acceleration, not args.no_return)
    distance_mm = args.speed * args.duration * 1000

    print(f"Robot: {args.robot} ({host})")
    print(f"Motion: {args.axis}, about {distance_mm:.1f} mm, return={not args.no_return}")
    banner, mode = dashboard_status(host)
    print(f"Dashboard banner: {banner or '<empty>'}")
    print(f"Robot mode: {mode}")
    print("\nProgram:")
    print(program)

    if not args.execute:
        print("DRY RUN: add --execute to send this program to the robot.")
        return

    send_program(host, 30002, program)
    print("Program sent to port 30002.")


if __name__ == "__main__":
    main()
