#!/usr/bin/env python3
"""qBArm documentation server: serves the Markdown pages in docs/pages as a small web app on the network.

    python3 qb_docs_server.py [--port 8080] [--pages ../pages]

  /                    the viewer (static/index.html): renders Markdown (marked) and UML diagrams (mermaid) in the
                       browser; both libraries are served from static/, so it works without internet access
  /api/pages           page list: [{"name", "title"}], ordered by file name
  /pages/<name>.md     a page's Markdown source
  /api/status          live state of the cell (services, cell process group, arm, claw, GPU server); cached 5 s

Standard library only. Runs as the systemd service qb-arm-docs.service (installed by install.sh).
"""
import argparse
import json
import os
import re
import subprocess
import threading
import time
import urllib.request
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, 'static')
TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
         '.css': 'text/css; charset=utf-8', '.md': 'text/markdown; charset=utf-8', '.svg': 'image/svg+xml',
         '.png': 'image/png', '.json': 'application/json'}

# What /api/status checks (all cheap; the result is cached)
SERVICES = ['ros2-discovery', 'ros2-microros-agent', 'qb-arm-docs']
AP_CONNECTION = 'qbarm-claw'   # NetworkManager access point for the claw
HOSTS = {'Lite6 controller': '192.168.1.23', 'Claw ESP32 (qbarm-claw)': '10.42.0.10'}
GPU_HEALTH = 'http://hbh-ai.local:8770/health'
CELL_PGID = os.path.join(os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'), 'qb_arm_cell', 'pgid')


def page_title(path):
    with open(path, encoding='utf-8') as f:
        for line in f:
            if line.startswith('# '):
                return line[2:].strip()
    return os.path.splitext(os.path.basename(path))[0]


def run(cmd, timeout=3):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def ping(host):
    return subprocess.call(['ping', '-c', '1', '-W', '1', host], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) == 0


def cell_state():
    try:
        pgid = int(open(CELL_PGID).read().strip())
    except (OSError, ValueError):
        return {'running': False}
    procs = run(['pgrep', '-g', str(pgid)]).split()
    if not procs:
        return {'running': False}
    mode = run(['ps', '-o', 'args=', '-p', str(pgid)])
    mode = re.search(r'qb_arm (\w+)\.launch\.py', mode)
    return {'running': True, 'mode': mode.group(1) if mode else '?', 'pgid': pgid, 'processes': len(procs)}


def gpu_state():
    try:
        with urllib.request.urlopen(GPU_HEALTH, timeout=2) as r:
            return json.load(r)
    except Exception as e:  # noqa: BLE001 - any failure means "not reachable"
        return {'ok': False, 'error': str(e)}


class Status:
    def __init__(self, ttl=5.0):
        self.ttl, self.stamp, self.value, self.lock = ttl, 0.0, None, threading.Lock()

    def get(self):
        with self.lock:
            if time.time() - self.stamp > self.ttl:
                self.value = {
                    'time': time.strftime('%Y-%m-%d %H:%M:%S'),
                    'services': {s: run(['systemctl', 'is-active', s]) or 'unknown' for s in SERVICES},
                    'access_point': AP_CONNECTION in run(['nmcli', '-t', '-f', 'NAME', 'con', 'show', '--active']).split(),
                    'cell': cell_state(),
                    'hosts': {name: ping(ip) for name, ip in HOSTS.items()},
                    'gpu_server': gpu_state(),
                }
                self.stamp = time.time()
            return self.value


class Handler(SimpleHTTPRequestHandler):
    pages_dir = None
    status = Status()

    def log_message(self, fmt, *args):  # quiet: one line per error only
        if not str(args[1] if len(args) > 1 else '').startswith(('2', '3')):
            super().log_message(fmt, *args)

    def send(self, body, content_type, status=HTTPStatus.OK):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        self.wfile.write(data)

    def send_file(self, path):
        if not os.path.isfile(path):
            return self.send('Not found', 'text/plain', HTTPStatus.NOT_FOUND)
        with open(path, 'rb') as f:
            self.send(f.read(), TYPES.get(os.path.splitext(path)[1], 'application/octet-stream'))

    def do_GET(self):
        path = self.path.split('?', 1)[0].split('#', 1)[0]
        if path in ('/', '/index.html'):
            return self.send_file(os.path.join(STATIC, 'index.html'))
        if path == '/api/pages':
            names = sorted(n for n in os.listdir(self.pages_dir) if n.endswith('.md'))
            pages = [{'name': n[:-3], 'title': page_title(os.path.join(self.pages_dir, n))} for n in names]
            return self.send(json.dumps(pages), TYPES['.json'])
        if path == '/api/status':
            return self.send(json.dumps(self.status.get()), TYPES['.json'])
        for prefix, root in (('/pages/', self.pages_dir), ('/static/', STATIC)):
            if path.startswith(prefix):
                name = os.path.basename(path[len(prefix):])   # no sub-directories, no '..'
                return self.send_file(os.path.join(root, name))
        self.send('Not found', 'text/plain', HTTPStatus.NOT_FOUND)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--pages', default=os.path.join(HERE, '..', 'pages'))
    args = parser.parse_args()
    Handler.pages_dir = os.path.realpath(args.pages)
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f'qBArm docs on http://{args.bind}:{args.port} (pages: {Handler.pages_dir})', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
