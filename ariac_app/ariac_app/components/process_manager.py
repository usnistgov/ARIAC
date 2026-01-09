import os
import asyncio
import signal
import subprocess
import psutil
import uuid
import shlex
import threading
import sys

from pathlib import Path
    
class ProcessManager:
    def __init__(self, cmd: str, logs_to_file: bool = False):
        self.log_file = None
        if logs_to_file:
            self.log_file = Path(f"/tmp/{uuid.uuid4()}.txt")
            self.file = open(self.log_file, 'w', buffering=1, encoding='utf-8')
            self._reader_thread: threading.Thread | None = None
            self._stop_event = threading.Event()

        self.process = self.start_process(cmd)

    def start_process(self, cmd: str):
        stdout_target = self.file if self.log_file is not None else None
        stderr_target = subprocess.STDOUT
        env = os.environ.copy()
        removed = []

        if 'QT_PLUGIN_PATH' in env and 'cv2' in env['QT_PLUGIN_PATH']:
            env.pop('QT_PLUGIN_PATH', None)
            removed.append('QT_PLUGIN_PATH')
        if 'QT_QPA_PLATFORM_PLUGIN_PATH' in env and 'cv2' in env['QT_QPA_PLATFORM_PLUGIN_PATH']:
            env.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
            removed.append('QT_QPA_PLATFORM_PLUGIN_PATH')

        if removed:
            print(f"ProcessManager: sanitized env, removed: {', '.join(removed)}")

        if self.log_file is not None:
            proc = subprocess.Popen(
                shlex.split(cmd),
                stdout=subprocess.PIPE,
                stderr=stderr_target,
                env=env,
                text=True,
                bufsize=1,
                # preexec_fn=os.setsid,
                start_new_session=True,
            )

            def _reader():
                assert proc.stdout is not None
                for line in proc.stdout:
                    try:
                        self.file.write(line)
                        self.file.flush()
                    except Exception:
                        pass
                    try:
                        sys.stdout.write(line)
                        sys.stdout.flush()
                    except Exception:
                        pass
                proc.stdout.close()

            self._reader_thread = threading.Thread(target=_reader, daemon=True)
            self._reader_thread.start()
            return proc

        return subprocess.Popen(
            shlex.split(cmd),
            stdout=stdout_target,
            stderr=stderr_target,
            env=env,
            preexec_fn=os.setsid,
        )

    @property
    def is_running(self) -> bool:
        return self.process.poll() is None

    async def close(self):
        if self.log_file is not None:
            try:
                if self._reader_thread is not None:
                    self._reader_thread.join(timeout=2.0)
            except Exception:
                pass
            try:
                self.file.close()
            except Exception:
                pass
        if self.is_running:
            try:
                children = psutil.Process(self.process.pid).children(recursive=True)
            except (psutil.NoSuchProcess, Exception):
                children = []

            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except Exception:
                pass

            gone, alive = psutil.wait_procs(children, timeout=1.0)
            for p in alive:
                try:
                    p.kill()
                except Exception:
                    pass
            
            try:
                await asyncio.to_thread(self.process.wait, 1.0)
            except subprocess.TimeoutExpired:
                try:
                    self.process.kill()
                except Exception:
                    pass
                await asyncio.to_thread(self.process.wait)