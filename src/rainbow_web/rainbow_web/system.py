"""The Pi's health for the phone remote's System tab, read from /proc and /sys.

The robot container shares the host's PID namespace and network, so /proc/stat,
/proc/net/wireless and every process (the Hailo container's too) are the whole
Pi's. Each function takes the text it parses, so the tests can feed it samples.
"""

import os
import subprocess

# get_throttled bits: now (0-3) and since boot (16-19)
THROTTLE_BITS = {0: 'under-voltage', 1: 'clock capped', 2: 'throttled', 3: 'soft temp limit'}


def read(path, default=''):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def cpu_times(stat_text):
    """Return [(idle, total)] for the whole CPU then each core, from /proc/stat."""
    out = []
    for line in stat_text.splitlines():
        if not line.startswith('cpu'):
            break
        v = [int(x) for x in line.split()[1:9]]
        out.append((v[3] + v[4], sum(v)))
    return out


def cpu_percent(before, after):
    """Busy % for each entry of two cpu_times() samples (None where no time passed)."""
    out = []
    for (i0, t0), (i1, t1) in zip(before, after):
        out.append(None if t1 <= t0 else round(100.0 * (1.0 - (i1 - i0) / (t1 - t0))))
    return out


def meminfo(text):
    """Return (used MB, total MB), used = total - available."""
    kb = {}
    for line in text.splitlines():
        name, _, rest = line.partition(':')
        if rest:
            kb[name] = int(rest.split()[0])
    total = kb.get('MemTotal', 0)
    return round((total - kb.get('MemAvailable', total)) / 1024), round(total / 1024)


def disk(path):
    """Return (used GB, total GB) of the filesystem holding path."""
    try:
        s = os.statvfs(path)
    except OSError:
        return None
    total = s.f_blocks * s.f_frsize
    return round((total - s.f_bavail * s.f_frsize) / 1e9, 1), round(total / 1e9, 1)


def throttled(value):
    """Decode the firmware's get_throttled value: (now [...], since boot [...])."""
    now = [n for b, n in THROTTLE_BITS.items() if value & (1 << b)]
    past = [n for b, n in THROTTLE_BITS.items() if value & (1 << (b + 16))]
    return now, past


def wifi(text, iface='wlan0'):
    """Return (link quality 0-70, signal dBm) from /proc/net/wireless, or None."""
    for line in text.splitlines():
        name, _, rest = line.strip().partition(':')
        if name == iface:
            f = rest.split()
            return int(float(f[1])), int(float(f[2]))
    return None


def process_table(proc='/proc'):
    """Return {pid: (ppid, cpu ticks)} for every process."""
    table = {}
    for name in os.listdir(proc):
        if not name.isdigit():
            continue
        text = read(f'{proc}/{name}/stat')
        # the command name is in parentheses and may contain spaces
        rest = text[text.rfind(')') + 2:].split()
        if len(rest) > 13:
            table[int(name)] = (int(rest[1]), int(rest[11]) + int(rest[12]))
    return table


def tree_ticks(table, root):
    """CPU ticks of root and all its descendants."""
    children = {}
    for pid, (ppid, _) in table.items():
        children.setdefault(ppid, []).append(pid)
    total, todo = 0, [root]
    while todo:
        pid = todo.pop()
        if pid in table:
            total += table[pid][1]
        todo.extend(children.get(pid, ()))
    return total


def supervisor_status(text):
    """Parse `supervisorctl status`: [(name, state, pid or None)]."""
    out = []
    for line in text.splitlines():
        f = line.split()
        if len(f) < 2:
            continue
        pid = None
        if len(f) > 3 and f[2] == 'pid':
            pid = int(f[3].rstrip(','))
        out.append((f[0], f[1], pid))
    return out


def supervisorctl(socket=None):
    cmd = ['supervisorctl'] + (['-s', f'unix://{socket}'] if socket else []) + ['status']
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def other_supervisors(proc='/proc'):
    """PIDs of the supervisords in other containers (the Hailo's) with a socket."""
    try:
        own_root = os.stat('/').st_ino
    except OSError:
        return []
    out = []
    for name in os.listdir(proc):
        if not name.isdigit() or 'supervisord' not in read(f'{proc}/{name}/cmdline'):
            continue
        try:
            root_ino = os.stat(f'{proc}/{name}/root/').st_ino
        except OSError:
            continue
        sock = f'{proc}/{name}/root/run/supervisor.sock'
        # inode 2 is the host's own root (its supervisord does not run ours)
        if root_ino not in (own_root, 2) and os.path.exists(sock):
            out.append(int(name))
    return out


def ns_pid_map(table, root, proc='/proc'):
    """Map the PIDs inside another container's namespace to ours, for root's tree."""
    children = {}
    for pid, (ppid, _) in table.items():
        children.setdefault(ppid, []).append(pid)
    out, todo = {}, [root]
    while todo:
        pid = todo.pop()
        for line in read(f'{proc}/{pid}/status').splitlines():
            if line.startswith('NSpid:'):
                out[int(line.split()[-1])] = pid
        todo.extend(children.get(pid, ()))
    return out


class Sampler:
    """Takes the System tab's numbers every couple of seconds (called from a thread)."""

    def __init__(self, data_path='/data/rainbow'):
        self.data_path = data_path if os.path.exists(data_path) else '/'
        self.cpu_before = None
        self.ticks_before = {}
        self.tick_hz = os.sysconf('SC_CLK_TCK')
        self.before_time = None

    def sample(self, now, with_programs=True):
        cpu_now = cpu_times(read('/proc/stat'))
        cpu = cpu_percent(self.cpu_before, cpu_now) if self.cpu_before else []
        self.cpu_before = cpu_now
        freqs = [int(read(f'/sys/devices/system/cpu/cpu{i}/cpufreq/scaling_cur_freq', '0'))
                 for i in range(max(len(cpu_now) - 1, 0))]
        thr = read('/sys/devices/platform/soc/soc:firmware/get_throttled', '').strip()
        now_t, past_t = throttled(int(thr, 0)) if thr else (None, None)
        used, total = meminfo(read('/proc/meminfo'))
        out = {
            't': 'system',
            'cpu': cpu[0] if cpu else None,
            'cores': cpu[1:],
            'mhz': max(freqs) // 1000 if freqs else None,
            'throttled_now': now_t, 'throttled_since_boot': past_t,
            'mem': [used, total],
            'disk': disk(self.data_path),
            'uptime': int(float(read('/proc/uptime', '0').split()[0] or 0)),
            'load': read('/proc/loadavg').split()[:3],
            'wifi': wifi(read('/proc/net/wireless')),
        }
        if with_programs:
            out['programs'] = self.programs(now)
        return out

    def programs(self, now):
        """[(container, program, state, cpu % of one core or None)]."""
        table = process_table()
        rows = []
        for name, st, pid in supervisor_status(supervisorctl()):
            rows.append(['robot', name, st, pid])
        for sup in other_supervisors():
            nsmap = ns_pid_map(table, sup)
            for name, st, pid in supervisor_status(
                    supervisorctl(f'/proc/{sup}/root/run/supervisor.sock')):
                rows.append(['hailo', name, st, nsmap.get(pid)])
        dt = None if self.before_time is None else now - self.before_time
        ticks_now = {}
        out = []
        for container, name, st, pid in rows:
            pct = None
            if pid is not None:
                key = (container, name, pid)
                ticks_now[key] = tree_ticks(table, pid)
                if dt and key in self.ticks_before:
                    pct = round(100.0 * (ticks_now[key] - self.ticks_before[key])
                                / self.tick_hz / dt)
            out.append([container, name, st, pct])
        self.ticks_before, self.before_time = ticks_now, now
        return out
