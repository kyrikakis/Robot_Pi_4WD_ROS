"""The Pi's health for the phone remote's System tab, read from /proc and /sys.

The robot container shares the host's PID namespace and network, so /proc/stat,
/proc/net/wireless and every process (the Hailo container's too) are the whole
Pi's. Each function takes the text it parses, so the tests can feed it samples.

The Hailo's NPU load comes from HailoRT's monitor: the Hailo app runs with
HAILO_MONITOR=1 (set in its container's /etc/supervisor/conf.d/hailo.conf) and
writes one protobuf file per process to its /tmp/hmon_files about once a second
(no measurable cost to the Hailo: its networks stayed at 15 fps).
"""

import http.client
import os
import socket
import struct
import time
import xmlrpc.client

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


class _UnixConnection(http.client.HTTPConnection):

    def __init__(self, path):
        super().__init__('localhost', timeout=3)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(3)
        self.sock.connect(self.path)


class _UnixTransport(xmlrpc.client.Transport):

    def __init__(self, path):
        super().__init__()
        self.path = path

    def make_connection(self, host):
        return _UnixConnection(self.path)


def supervisor_programs(sock='/var/run/supervisor.sock'):
    """Ask a supervisord over its socket: [(name, state, pid or None)]."""
    try:
        proxy = xmlrpc.client.ServerProxy('http://localhost', transport=_UnixTransport(sock))
        info = proxy.supervisor.getAllProcessInfo()
    except (OSError, xmlrpc.client.Error):
        return []
    return [(p['name'], p['statename'], p['pid'] or None) for p in info]


def other_supervisors(proc='/proc'):
    """Return the PIDs of the supervisords in other containers (the Hailo's)."""
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


def proto_fields(buf):
    """Return [(field number, value)] of a protobuf message (varint, double, bytes)."""
    out, i, n = [], 0, len(buf)

    def varint():
        nonlocal i
        value = shift = 0
        while True:
            b = buf[i]
            i += 1
            value |= (b & 0x7f) << shift
            shift += 7
            if not b & 0x80:
                return value
    while i < n:
        key = varint()
        field, wire = key >> 3, key & 7
        if wire == 0:
            out.append((field, varint()))
        elif wire == 1:
            out.append((field, struct.unpack_from('<d', buf, i)[0]))
            i += 8
        elif wire == 2:
            size = varint()
            out.append((field, bytes(buf[i:i + size])))
            i += size
        elif wire == 5:
            out.append((field, struct.unpack_from('<f', buf, i)[0]))
            i += 4
        else:
            raise ValueError(f'protobuf wire type {wire}')
    return out


def hailo_monitor(buf):
    """Parse one HailoRT monitor file (ProtoMon, HailoRT 4.20).

    Returns {'load': chip utilization %, 'models': [[name, utilization %, fps]]}.
    ProtoMon: 1 pid, 2 networks (1 name, 2 fps, 3 utilization %), 3 stream
    queues, 4 devices (1 id, 2 utilization %, 3 architecture); fields that are
    zero are left out of the file.
    """
    load, models = None, []
    for field, value in proto_fields(buf):
        if field == 2:
            net = dict(proto_fields(value))
            models.append([net.get(1, b'').decode(errors='replace'),
                           round(net.get(3, 0.0), 1), round(net.get(2, 0.0), 1)])
        elif field == 4:
            dev = dict(proto_fields(value))
            load = max(load or 0.0, round(dev.get(2, 0.0), 1))
    return {'load': load, 'models': models}


def hailo_stats(sups, proc='/proc', max_age_s=10.0):
    """Return the Hailo's NPU load from the monitor files in its container, or None."""
    load, models = None, []
    for sup in sups:
        folder = f'{proc}/{sup}/root/tmp/hmon_files'
        try:
            names = os.listdir(folder)
        except OSError:
            continue
        for name in names:
            path = os.path.join(folder, name)
            try:
                if time.time() - os.stat(path).st_mtime > max_age_s:
                    continue      # a process that has ended leaves its file behind
                with open(path, 'rb') as f:
                    one = hailo_monitor(f.read())
            except (OSError, ValueError, IndexError, struct.error):
                continue          # caught mid-write: next time
            if one['load'] is not None:
                load = max(load or 0.0, one['load'])
            models.extend(one['models'])
    if load is None and not models:
        return None
    return {'load': load, 'models': models}


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
            sups = other_supervisors()
            out['programs'] = self.programs(now, sups)
            out['hailo'] = hailo_stats(sups)
        return out

    def programs(self, now, sups):
        """[(container, program, state, cpu % of one core or None)]."""
        table = process_table()
        rows = []
        for name, st, pid in supervisor_programs():
            rows.append(['robot', name, st, pid])
        for sup in sups:
            nsmap = ns_pid_map(table, sup)
            for name, st, pid in supervisor_programs(f'/proc/{sup}/root/run/supervisor.sock'):
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
