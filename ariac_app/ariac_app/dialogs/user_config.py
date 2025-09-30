import math
import os

from nicegui import events, ui

from typing import Literal

from string import ascii_letters, digits

from ariac_app.structures import UserInfo
from ariac_app.dialogs.file_picker import FilePicker

from ariac_setup.structures import (
    Sensor,
    SensorType,
    BreakBeam,
    DistanceSensor,
    Camera,
    Lidar,
    Scan
)

from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Pose, Point, Quaternion


class UserConfigBuilder(ui.dialog):
    def __init__(self, info: UserInfo):
        super().__init__()
        self.info = info
        
        with self.classes('w-full'), ui.card().classes('w-full'):
            ui.label('Enter Team Information').classes('w-full text-base')

            self.team_name = ui.input('Team Name').classes('w-full').bind_value(self.info, "name")

            ui.separator()

            ui.label('Enter Inspection Conveyor Settings').classes('w-full text-base')

            with ui.row().classes('w-full items-center'):
                ui.number(label='Conveyor Speed (m/s)').classes('w-40').bind_value(self.info, "conveyor_speed")
                ui.slider(min=0.01, max=0.1, step=0.005).classes('w-64').bind_value(self.info, "conveyor_speed")

            with ui.row().classes('w-full items-center'):
                ui.number(label='Cell Feed Rate').classes('w-40').bind_value(self.info, "cell_feed_rate")
                ui.slider(min=0.01, max=0.2, step=0.005).classes('w-64').bind_value(self.info, "cell_feed_rate")

            ui.separator()

            ui.label('Enter Sensors').classes('w-full text-base')

            with ui.row().classes('w-full items-center justify-center'):
                self.add_button = ui.button('Add', on_click=self.add_sensor)
                self.add_button.disable()

                self.sensor_to_add = ui.select(
                    [str(s).replace('_', '').capitalize() for s in SensorType], 
                    on_change=lambda e: self.add_button.enable()
                ).classes('w-32')

            self.sensor_grid = ui.aggrid(
                {
                    'columnDefs': [
                        {'headerName': 'Type', 'field': 'type'},
                        {'headerName': 'Grade', 'field': 'grade'},
                        {'headerName': 'Name', 'field': 'name'},
                        {'headerName': 'Cost', 'field': 'cost'}
                    ],
                    'rowData': [],
                    'rowSelection': 'single'
                }
            )

            self.sensor_grid.on('rowSelected', self.handle_selection)

            with ui.row().classes('w-full justify-center'):
                self.edit_button = ui.button('Edit', on_click=self.edit_sensor)
                self.remove_button = ui.button('Remove', on_click=self.remove_sensor)
                self.edit_button.disable()
                self.remove_button.disable()

            ui.button('Save', on_click=self._save_file)

            if self.info.sensors:
                self.update_grid()


    def update_grid(self):
        self.edit_button.disable()
        self.remove_button.disable()
        
        self.sensor_grid.options['rowData'] = [
            {
                'type': str(s.sensor_type).replace('_', '').capitalize(),
                'grade': str(s.grade),
                'name': s.name,
                'cost': s.cost
            }
            for s in self.info.sensors
        ]

        self.sensor_grid.update()


    async def add_sensor(self, original_sensor: Sensor | None = None):        
        sensor: Sensor | None
        if original_sensor is None:
            match(self.sensor_to_add.value):
                case "Breakbeam":
                    sensor = await AddBreakBeam() #type: ignore
                case "Distance":
                    sensor = await AddDistanceSensor() #type: ignore
                case "Camera":
                    sensor = await AddCamera() #type: ignore
                case "Lidar":
                    sensor = await AddLidar() #type: ignore
                case _:
                    raise ValueError("Invalid sensor in add_sensor")
        else:
            match(original_sensor.sensor_type):
                case SensorType.BREAKBEAM:
                    sensor = await AddBreakBeam(original_sensor) #type: ignore
                case SensorType.DISTANCE:
                    sensor = await AddDistanceSensor(original_sensor) #type: ignore
                case SensorType.CAMERA:
                    sensor = await AddCamera(original_sensor) #type: ignore
                case SensorType.LIDAR:
                    sensor = await AddLidar(original_sensor) #type: ignore
                case _:
                    raise ValueError("Invalid sensor in add_sensor")


        if original_sensor is not None and sensor is None:
            self.info.sensors.append(original_sensor) # Add back original sensor if edit fails
            ui.notify("Edit sensor cancelled")
        elif sensor is None:
            ui.notify("Unable to add sensor", type='warning')
        elif sensor.name in [s.name for s in self.info.sensors]:
            if original_sensor is not None:
                self.info.sensors.append(original_sensor) # Add back original sensor if edit fails
                ui.notify("Sensor name already exists. Cancelling edit", type="warning")
            else:
                ui.notify("Sensor name already exists", type='negative')
        else:
            self.info.sensors.append(sensor)  

        self.update_grid()
    
    async def edit_sensor(self):
        row = await self.sensor_grid.get_selected_row()

        if row is None:
            ui.notify("Unable to get selcted row")
            return
        
        sensor = next((s for s in self.info.sensors if s.name == row['name']), None)

        if sensor is None:
            ui.notify("Sensor name does not exist in list")
            return
        
        self.info.sensors.remove(sensor)
        await self.add_sensor(sensor)
    
    async def remove_sensor(self):
        row = await self.sensor_grid.get_selected_row()

        if row is None:
            ui.notify("Unable to get selcted row")
            return
        
        sensor = next((s for s in self.info.sensors if s.name == row['name']), None)

        if sensor is not None:
            self.info.sensors.remove(sensor)
            self.update_grid()

    def handle_selection(self, e: events.GenericEventArguments) -> None:
        if e.args['selected']:
            self.edit_button.enable()
            self.remove_button.enable()
        else:
            self.edit_button.disable()
            self.remove_button.disable()

    async def _save_file(self):
        if not self.info.name:
            ui.notify("Name must have a value", type="warning")
            return
        
        path = await FilePicker("/team_ws" if os.path.exists("/team_ws") else "~", selection_type="directory")

        if path is None:
            ui.notify("No folder selected")
            return

        yaml_path = os.path.join(path, f"{self.info.name}.yaml")
        
        with open(yaml_path, "w") as f:
            f.write(self.info.contents())
            ui.notify(f"User config written to file: {yaml_path}", type="info")

        self.submit((self.info, yaml_path))


class AddSensor(ui.dialog):
    def __init__(self, sensor_type: str, sensor: Sensor | None = None):
        super().__init__()

        self.add_button: ui.button | None = None

        if sensor is not None:
            self.name = sensor.name
            self.update_rate = sensor.update_rate

            self.x = sensor.pose.position.x
            self.y = sensor.pose.position.y
            self.z = sensor.pose.position.z

            q = [sensor.pose.orientation.x, sensor.pose.orientation.y, sensor.pose.orientation.z, sensor.pose.orientation.w]

            self.roll, self.pitch, self.yaw = euler_from_quaternion(q)
            
        else:
            self.name = ""
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw, = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

        with self.classes('w-5/6'), ui.card():
            ui.label(f'Enter {sensor_type} Info').classes('w-full text-base')

            ui.input('Sensor Name', on_change=self.name_change, validation=self.name_validation).classes('w-full').bind_value(self, 'name')

            self.custom_sensor_elements()

            ui.separator()

            ui.label('Enter Pose').classes('w-full text-base')

            with ui.row().classes('w-full justify_center'):
                ui.number('x', step=0.05).classes('w-24').bind_value(self, 'x')
                ui.number('y', step=0.05).classes('w-24').bind_value(self, 'y')
                ui.number('z', step=0.05).classes('w-24').bind_value(self, 'z')

            with ui.row().classes('w-full justify_center'): 
                ui.number('roll', step=0.05).classes('w-24').bind_value(self, 'roll')
                ui.number('pitch', step=0.05).classes('w-24').bind_value(self, 'pitch')
                ui.number('yaw', step=0.05).classes('w-24').bind_value(self, 'yaw')

            ui.separator()
            
            with ui.row().classes('w-full justify-end'):
                self.add_button = ui.button('Add' if sensor is None else 'Edit', on_click=self._handle_ok)

                if self.name == '':
                    self.add_button.disable()
    
    @property
    def pose(self):
        qx, qy, qz, qw = quaternion_from_euler(self.roll, self.pitch, self.yaw)
    
        return Pose(position=Point(x=self.x, y=self.y, z=self.z),orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))

    def name_validation(self, name: str):
        if not name:
            return 'Name is required'
        
        for c in name:
            if c not in (ascii_letters + digits + "_"):
                return f"Invalid character {c}"
        
        return None
    
    def name_change(self, e: events.ValueChangeEventArguments):
        if self.add_button is None:
            return 
        
        if self.name_validation(e.value) is None:
            self.add_button.enable()
        else:
            self.add_button.disable()

    def custom_sensor_elements(self):
        raise NotImplementedError

    def _handle_ok(self):
        raise NotImplementedError

class AddBreakBeam(AddSensor):
    def __init__(self, sensor: BreakBeam | None = None):
        if sensor is None:
            self.update_rate = 10
        else:
            self.update_rate = sensor.update_rate

        super().__init__("Breakbeam", sensor)
    
    def custom_sensor_elements(self):
        with ui.row().classes('w-full items-center justify-start'):
            ui.label('Update Rate (Hz)')

            ui.toggle([10,30]).bind_value(self, 'update_rate')

    def _handle_ok(self):
        self.submit(
            BreakBeam (
                name=self.name, 
                sensor_type=SensorType.BREAKBEAM, 
                pose=self.pose, 
                update_rate=self.update_rate,
            )
        )

class AddDistanceSensor(AddSensor):
    def __init__(self, sensor: DistanceSensor | None = None):
        if sensor is None:
            self.update_rate = 10
        else:
            self.update_rate = sensor.update_rate

        super().__init__("Distance Sensor", sensor)
    
    def custom_sensor_elements(self):
        with ui.row().classes('w-full items-center justify-start'):
            ui.label('Update Rate (Hz)')

            ui.toggle([10,30]).bind_value(self, 'update_rate')

    def _handle_ok(self):
        self.submit(
            DistanceSensor (
                name=self.name, 
                sensor_type=SensorType.DISTANCE, 
                pose=self.pose, 
                update_rate=self.update_rate,
            )
        )

class AddCamera(AddSensor):
    def __init__(self, sensor: Camera | None = None):
        self.resolution: Literal['720p', '1080p']

        if sensor is None:
            self.resolution = "720p"
            self.fov = 90
        else:
            self.resolution = sensor.resolution
            self.fov = round(math.degrees(sensor.fov), 1)

        super().__init__("Camera", sensor)
    
    def custom_sensor_elements(self):
        with ui.row().classes('w-full items-center justify-start'):
            ui.label('Resolution')
            ui.toggle(["720p", "1080p"]).bind_value(self, 'resolution')

        with ui.row().classes('w-full items-center'):
            ui.number(label='Field of View (°)').classes('w-40').bind_value(self, 'fov')
            ui.slider(min=1, max=180, step=1).classes('w-40').bind_value(self, 'fov')

    def _handle_ok(self):
        self.submit(
            Camera(
                name=self.name, 
                sensor_type=SensorType.CAMERA,
                pose=self.pose, 
                update_rate=30,
                resolution=self.resolution,
                fov=math.radians(self.fov)
            )
        )

class AddLidar(AddSensor):
    def __init__(self, sensor: Lidar | None = None):
        if sensor is None:
            self.update_rate = 10
            self.h_fov = 30
            self.h_samples = 10
            self.v_fov = 30
            self.v_samples = 10
        else:
            self.update_rate = sensor.update_rate
            self.h_fov = round(math.degrees(abs(sensor.horizontal.max_angle) * 2), 1)
            self.h_samples = sensor.horizontal.samples
            self.v_fov = round(math.degrees(abs(sensor.vertical.max_angle) * 2), 1)
            self.v_samples = sensor.vertical.samples
        
        super().__init__("Lidar", sensor)
    
    def custom_sensor_elements(self):
        with ui.row().classes('w-full items-center justify-start'):
            ui.label('Update Rate (Hz)')
            ui.toggle([10,20]).bind_value(self, 'update_rate')

        ui.separator()

        with ui.row().classes('w-full items-center justify-start'):
            ui.label("Horizontal Scan Properties").classes("text-small")
        
        with ui.row().classes('w-full items-center'):
            fov = ui.number(label='Field of View (°)').classes('w-40').bind_value(self, 'h_fov')
            slider = ui.slider(min=1, max=180, step=1).classes('w-40').bind_value(self, 'h_fov')
            fov.bind_value_from(slider)
            slider.bind_value_from(fov)

        with ui.row().classes('w-full items-center'):
            self.horizontal_samples = ui.number(label='Samples').classes('w-full').bind_value(self, 'h_samples')

        ui.separator()

        with ui.row().classes('w-full items-center justify-start'):
            ui.label("Vertical Scan Properties").classes("text-small")
        
        with ui.row().classes('w-full items-center'):
            fov = ui.number(label='Field of View (°)').classes('w-40').bind_value(self, 'v_fov')
            slider = ui.slider(min=1, max=180, step=1).classes('w-40').bind_value(self, 'v_fov')
            fov.bind_value_from(slider)
            slider.bind_value_from(fov)

        with ui.row().classes('w-full items-center'):
            self.horizontal_samples = ui.number(label='Samples').classes('w-full').bind_value(self, 'v_samples')

    def _handle_ok(self):
        lidar = Lidar(
            name=self.name, 
            sensor_type=SensorType.LIDAR, 
            pose=self.pose,
            update_rate=self.update_rate,
            horizontal=Scan(
                max_angle=math.radians(self.h_fov/2),
                min_angle=-math.radians(self.h_fov/2),
                samples=self.h_samples
            ),
            vertical=Scan(
                max_angle=math.radians(self.v_fov/2),
                min_angle=-math.radians(self.v_fov/2),
                samples=self.v_samples
            )
        )
        
        if not lidar.valid_location():
            ui.notify("Lidar not within bounding box", type="warning")
            return

        self.submit(lidar)
