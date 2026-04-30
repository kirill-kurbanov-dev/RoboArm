# Dashboard examples
# CB-series: https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-port-29999-15690/
# E-series:  https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-e-series-port-29999-42728/
import socket
import time
import multiprocessing as mp
from config import load_config, dashboard_command

def main(heartbeat, host=None):
    cfg = load_config()
    host = host or cfg.diagnost_ip
    try:
        dashboard_command(host, "power on", cfg.dashboard_port)
        time.sleep(5)
        dashboard_command(host, "brake release", cfg.dashboard_port)
        heartbeat.put((mp.current_process().name, "FINISHED"))
    except Exception as exc:
        heartbeat.put((mp.current_process().name, "ERROR", f"{type(exc).__name__}: {str(exc)[:200]}", time.time()))
