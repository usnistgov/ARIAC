
.. _TUTORIAL_6:

======================================
Tutorial 6: Moving AGVs
======================================

.. note::
  Prerequisites: Complete tutorial 1.


In this tutorial you will learn how to:
  - Receive orders, 
  - Store orders internally,
  - Display orders on the standard output.




Receive Orders
---------------------------------

Orders are published to the topic ``/ariac/orders`` will be stored in a Python class ``Order``. 


Import Modules
^^^^^^^^^^^^^^
Modules needed in this tutorial are imported in the ``competition_interface.py`` file as seen in :numref:`import-modules`.

.. code-block:: python
    :caption: Module Imports
    :name: import-modules
    
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ariac_msgs.msg import Order as OrderMsg
    from ariac_msgs.msg import KittingPart as KittingPartMsg
    from ariac_msgs.msg import AssemblyPart as AssemblyPartMsg
    from ariac_msgs.msg import AssemblyTask as AssemblyTaskMsg
    from ariac_msgs.msg import KittingTask as KittingTaskMsg
    from ariac_msgs.msg import CombinedTask as CombinedTaskMsg
    from ariac_msgs.msg import PartPose as PartPoseMsg
    from geometry_msgs.msg import Pose



Subscriber
^^^^^^^^^^

To read orders published on the topic ``/ariac/orders``, create a subscriber in the ``competition_interface.py`` file as seen in :numref:`subscriber`.

.. code-block:: python
    :caption: Subscriber to the  ``/ariac/orders`` Topic
    :name: subscriber
    
    class CompetitionInterface(Node):

      ...

      def __init__(self):
          super().__init__('competition_interface')

          ...

        # Subscriber to the order topic
        self.orders_sub = self.create_subscription(OrderMsg, '/ariac/orders', self.orders_cb, 10)


Order Callback
^^^^^^^^^^^^^^^

.. code-block:: python
    :caption: Subscriber Callback
    
    def orders_cb(self, msg: Order):
        order = Order(msg)
        self.orders.append(order)
        if self.parse_incoming_order:
            self.get_logger().info(self.parse_order(order))
