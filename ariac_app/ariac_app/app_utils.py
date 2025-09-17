import psutil

def _find_gazebo_processes():
    """Yield processes that match Gazebo Sim (gz sim) server or gui."""
    for proc in psutil.process_iter(['pid', 'cmdline']):
        try:
            cmdline = proc.info['cmdline']
            if not cmdline:
                continue
            cmd = ' '.join(cmdline).lower()
            if 'gz sim server' in cmd or 'gz sim gui' in cmd:
                yield proc
        except (psutil.NoSuchProcess, psutil.AccessDenied, IndexError):
            continue

def is_gazebo_running():
    """Return True if Gazebo Sim is running, else False."""
    return any(True for _ in _find_gazebo_processes())

def kill_gazebo():
    """Kill Gazebo Sim processes (server/gui) and return their PIDs."""

    for proc in _find_gazebo_processes():
        try:
            proc.kill()   # or proc.terminate() for a gentler shutdown
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
