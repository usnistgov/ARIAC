# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from ariac_gui.trial_generator import GUI_CLASS


def main(args=None):
    rclpy.init(args=args)
    print("started_node")
    main_gui = GUI_CLASS()
    main_gui.mainloop()
    print("finished_node")
    rclpy.shutdown()


if __name__ == '__main__':
    main()