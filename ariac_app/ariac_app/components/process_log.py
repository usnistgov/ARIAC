import os
import asyncio
import signal
import subprocess
import psutil
from enum import IntEnum
from nicegui import ui
import re
from collections import deque

ANSI_ESCAPE_PATTERN = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

class LogLevel(IntEnum):
    ROS_DEBUG = 1
    GZ_DBG = 2
    ROS_INFO = 3
    GZ_MSG = 4
    OTHER = 5
    ROS_WARN = 6
    GZ_WRN = 7
    ROS_ERROR = 8
    GZ_ERR = 9
    
class ProcessLog:
    def __init__(self, cmd: str):
        self.process = self.start_process(cmd)

        self.log_area: ui.log

        self.all_lines: list[tuple[str, LogLevel]] = []
        self.line_buffer = deque()
        self.push_interval = 1/100

        self.search_text = ""
        self.selected_level = LogLevel.ROS_INFO

        self.read_task = None
        self.push_task = None

        self.colors = {
            LogLevel.OTHER: 'text-gray-800',
            LogLevel.GZ_DBG: 'text-blue-600',
            LogLevel.ROS_DEBUG: 'text-blue-600',
            LogLevel.GZ_MSG: 'text-green-500',
            LogLevel.ROS_INFO: 'text-gray-800',
            LogLevel.GZ_WRN: 'text-yellow-600',
            LogLevel.ROS_WARN: 'text-yellow-600',
            LogLevel.GZ_ERR: 'text-red-600',
            LogLevel.ROS_ERROR: 'text-red-600',
        }

    def start_process(self, cmd: str):
        return subprocess.Popen(
            cmd.split(" "),
            # stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=os.environ.copy(),
            preexec_fn=os.setsid,
        )

    def content(self):
        # search input
        with ui.column().classes('w-full items-center'):
            ui.input(
                placeholder="Search logs...",
                on_change=self.on_search_change
            ).classes("w-1/2").bind_value(self, "search_text")

            # horizontal toggle for log levels
            ui.toggle( 
                options={
                    LogLevel.ROS_DEBUG: 'Debug',
                    LogLevel.ROS_INFO: 'Info',
                    LogLevel.ROS_WARN: 'Warn',
                    LogLevel.ROS_ERROR: 'Error'
                },
                value=None,
                on_change=self.on_level_change,
            )

            self.log_area = ui.log().classes("bg-grey-1 w-full h-96 hover:resize-y").props(f'id={self.process.pid}')

        # start tasks
        self.read_task = asyncio.create_task(self.read_process_output())
        self.push_task = asyncio.create_task(self.throttled_push_loop())

    @property
    def is_running(self) -> bool:
        return self.process.poll() is None

    async def close(self):
        # Cancel the log reading and pushing tasks if they are still running
        if self.read_task:
            self.read_task.cancel()
        if self.push_task:
            self.push_task.cancel()

        if self.is_running:
            children = psutil.Process(self.process.pid).children(recursive=True)
            
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)

            tasks = [asyncio.create_task(asyncio.to_thread(self.process.wait))]

            pids = [self.process.pid]
            for process in children:
                pids.append(process.pid)
                if process.is_running():
                    tasks.append(asyncio.create_task(asyncio.to_thread(process.wait)))

            try:
                await asyncio.wait_for(asyncio.gather(*tasks), timeout=10.0)
            except asyncio.TimeoutError:
                for task in tasks:
                    try:
                        task.cancel()
                    except asyncio.CancelledError:
                        pass
                for pid in pids:
                    try:
                        proc = psutil.Process(pid)
                        proc.kill()
                    except:
                        pass

    # ----------------- filtering -----------------
    def on_level_change(self, event):
        self.selected_level = event.value
        self.apply_filters()

    def on_search_change(self, _):
        self.apply_filters()

    def apply_filters(self):
        self.log_area.clear()
        self.line_buffer.clear()
        for line, lvl in self.all_lines:
            if lvl >= self.selected_level and self.search_text.lower() in line.lower():
                self.log_area.push(
                    line,
                    classes=f'text-wrap font-mono text-sm {self.colors[lvl]}'
                )
        
        self.scroll_to_bottom()

    # ----------------- helpers -----------------
    def get_log_level(self, line: str) -> LogLevel:
        if "Dbg" in line:
            return LogLevel.GZ_DBG
        elif "DEBUG" in line:
            return LogLevel.ROS_DEBUG
        elif "Msg" in line:
            return LogLevel.GZ_MSG
        elif "INFO" in line:
            return LogLevel.ROS_INFO
        elif "Wrn" in line:
            return LogLevel.GZ_WRN
        elif "WARN" in line:
            return LogLevel.ROS_WARN
        elif "Err" in line:
            return LogLevel.GZ_ERR
        elif "ERROR" in line:
            return LogLevel.ROS_ERROR
    
        return LogLevel.OTHER

    def strip_ansi(self, text: str) -> str:
        return ANSI_ESCAPE_PATTERN.sub('', text)
    
    def scroll_to_bottom(self):        
        ui.run_javascript(f'var el = document.getElementById("{self.process.pid}"); if (el) el.scrollTop = el.scrollHeight;')

    # ----------------- reading / pushing -----------------
    async def read_process_output(self):
        if self.process.stdout is None:
            return

        while self.is_running:
            line: bytes = await asyncio.to_thread(self.process.stdout.readline)
            
            if not line:
                break

            decoded_line = self.strip_ansi(line.decode(errors='replace').rstrip())

            if "NameManager::issueNewName" in decoded_line:
                continue

            if "Contact system publishing on" in decoded_line:
                continue

            if "Binding loop detected for property" in decoded_line:
                continue

            if "Trying to serialize component" in decoded_line:
                continue

            if "Custom moment of inertia calculator for meshes" in decoded_line:
                continue

            lvl = self.get_log_level(decoded_line)

            self.all_lines.append((decoded_line, lvl))
            
            if lvl >= self.selected_level and self.search_text.lower() in decoded_line.lower():
                self.line_buffer.append(decoded_line)
            
        self.process.wait()

    async def throttled_push_loop(self):
        while self.is_running:
            if self.line_buffer:
                line = self.line_buffer.popleft()
                lvl = self.get_log_level(line)
                self.log_area.push(
                    line,
                    classes=f'text-wrap font-mono text-sm {self.colors[lvl]}'
                )
            await asyncio.sleep(self.push_interval)
