import argparse
import socket
import struct

from config import load_config


DEFAULT_PORTS = [
    80,
    443,
    502,
    29999,
    30001,
    30002,
    30003,
    30004,
    30011,
    30012,
    30013,
    8080,
    9093,
]


def resolve_host(name):
    cfg = load_config()
    if name == "diagnost":
        return cfg.diagnost_ip
    if name == "surgeon":
        return cfg.surgeon_ip
    return name


def recv_probe(host, port, timeout):
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)

        if port == 29999:
            try:
                banner = sock.recv(256)
            except socket.timeout:
                banner = b""
            sock.sendall(b"robotmode\n")
            try:
                response = sock.recv(256)
            except socket.timeout:
                response = b""
            return banner + response

        if port == 30004:
            # RTDE usually waits for a client request, so no banner is also normal.
            return b""

        try:
            return sock.recv(512)
        except socket.timeout:
            return b""


def classify(port, data):
    lower = data.lower()
    if port == 29999 and (b"dashboard" in lower or b"robotmode" in lower or b"power" in lower):
        return "UR Dashboard"
    if b"http" in lower or b"<html" in lower or b"bitly" in lower:
        return "HTTP/Web, not UR stream"
    if port in (30001, 30002, 30003, 30011, 30012, 30013):
        if len(data) >= 5:
            packet_size = struct.unpack("!i", data[:4])[0]
            packet_type = data[4]
            if 0 < packet_size < 100000 and packet_type in range(16):
                return f"Possible UR binary stream packet_size={packet_size} type={packet_type}"
            return f"Open, but packet does not look like UR stream packet_size={packet_size} type={packet_type}"
        if not data:
            return "Open, no initial data"
    if port == 30004:
        return "RTDE port open"
    if port == 502:
        return "Modbus TCP open"
    if not data:
        return "Open, no banner"
    return "Open, unknown service"


def printable(data):
    if not data:
        return "<empty>"
    text = data[:160].decode("utf-8", errors="replace")
    text = text.replace("\r", "\\r").replace("\n", "\\n")
    return text


def scan(host, ports, timeout):
    print(f"Scanning {host}")
    for port in ports:
        try:
            data = recv_probe(host, port, timeout)
            service = classify(port, data)
            print(f"{port:5d} OPEN   {service}")
            print(f"      bytes: {data[:32].hex(' ') if data else '<empty>'}")
            print(f"      text : {printable(data)}")
        except Exception as exc:
            print(f"{port:5d} closed {type(exc).__name__}: {exc}")


def main():
    parser = argparse.ArgumentParser(description="Scan robot ports and classify UR-related services.")
    parser.add_argument("host", nargs="?", default="both", help="diagnost, surgeon, both, or direct IP")
    parser.add_argument("--ports", nargs="*", type=int, default=DEFAULT_PORTS)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    if args.host == "both":
        for name in ("diagnost", "surgeon"):
            scan(resolve_host(name), args.ports, args.timeout)
            print()
    else:
        scan(resolve_host(args.host), args.ports, args.timeout)


if __name__ == "__main__":
    main()
