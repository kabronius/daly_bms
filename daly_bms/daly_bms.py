import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from dalybms import DalyBMS as DalyBMSDriver

class DalyBMS(Node):
    def __init__(self):
        super().__init__("daly_bms")
        self._driver: DalyBMSDriver = DalyBMSDriver()
        self._battery_status = BatteryState()   
        self._battery_status.header.frame_id = 'daly_bms'  
        self._battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN   
        self._battery_status.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

    def ros_read_params(self):
        self.declare_parameter("~serial_port", "/dev/ttyS0")
        if self.get_parameter("~serial_port").value == None:
            self.get_logger().warning("No serial port provided, using default: /dev/ttyS0")
            self._port = "/dev/ttyS0"
        else:
            self._port = self.get_parameter("~serial_port").value

    def ros_setup(self):
        self._battery_status_pub = self.create_publisher(BatteryState, "~/data", 10)
        timer_period: float = 1  # seconds
        self._reading_timer = self.create_timer(timer_period, self.read)
        self._publishing_timer = self.create_timer(timer_period, self.publish)

    def setup(self):
        self.ros_read_params()
        self._driver.connect(self._port)
        self.ros_setup()

    def read(self):
        try:
            soc_data = self._driver.get_soc()
            mosfet_data = self._driver.get_mosfet_status()
            cells_data = self._driver.get_cell_voltages()
            temp_data = self._driver.get_temperatures()
            status_data = self._driver.get_status()
        except:
            self.get_logger().warning("Skipping current read cycle: Driver failed to return data")
            return
        if soc_data    == False or \
           mosfet_data == False or \
           cells_data  == False or \
           temp_data   == False or \
           status_data == False:
            self.get_logger().warning("Skipping current read cycle: Driver failed to return data")
            return
        
        if status_data["cells"] != 0:
            self._battery_status.present = True
        else:
            self._battery_status.present = False
        self._battery_status.percentage = soc_data["soc_percent"]
        self._battery_status.voltage = soc_data["total_voltage"]
        self._battery_status.current = soc_data["current"]
        # TODO: Add PS status for full battery
        if mosfet_data["mode"] == "discharging":
            self._battery_status.power_supply_status =  BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif mosfet_data["mode"] == "charging":
            self._battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif mosfet_data["mode"] == "stationary":
            self._battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            self._battery_status.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            
        self._battery_status.charge = mosfet_data["capacity_ah"]
        self._battery_status.cell_voltage = list(cells_data.values())
        self._battery_status.cell_temperature = list(temp_data.values())

    def publish(self):
        self._battery_status.header.stamp = self.get_clock().now().to_msg()
        self._battery_status_pub.publish(self._battery_status)
