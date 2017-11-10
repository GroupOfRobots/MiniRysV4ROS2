import rclpy
from rys_interfaces import msg as RysMsgs
from rys_interfaces import srv as RysSrvs
from std_msgs import msg as RosMsgs

class RysRemoteNode(rclpy.Node):
	"""docstring for RysRemoteNode"""

	def __init__(self, robotName, nodeName, callbacks, messageRates):
		super().__init__(nodeName, namespace = robotName)

		self.enabled = False

		self.throttle = 0.0
		self.rotation = 0.0
		self.precision = 1

		# Create ROS publishers
		self.publisherEnableMotors = self.create_publisher(RosMsgs.Bool, '/control/enable_motors')
		self.publisherEnableBalancing = self.create_publisher(RosMsgs.Bool, '/control/enable_balancing')
		self.publisherSteering = self.create_publisher(RysMsgs.Steering, '/control/steering')
		self.publisherIMUCalibrate = self.create_publisher(RosMsgs.Empty, '/control/imu/calibrate')

		# Create ROS service clients
		self.clientSetRegulatorSettings = self.create_client(RysSrvs.SetRegulatorSettings, '/control/regulator_settings/set')
		self.clientGetRegulatorSettings = self.create_client(RysSrvs.GetRegulatorSettings, '/control/regulator_settings/get')

		# Create ROS subscribers
		self.subscriptionBattery = self.create_subscription(RysMsgs.BatteryStatus, '/sensor/battery', callbacks['battery'], qos_profile = rclpy.qos.qos_profile_default)
		self.subscriptionImu = self.create_subscription(RysMsgs.ImuRollRotation, '/sensor/imu', callbacks['imu'], qos_profile = rclpy.qos.qos_profile_sensor_data)
		self.subscriptionRanges = self.create_subscription(RysMsgs.Ranges, '/sensor/ranges', callbacks['ranges'], qos_profile = rclpy.qos.qos_profile_sensor_data)
		self.subscriptionTemperature = self.create_subscription(RosMsgs.Float32, '/sensor/temperature', callbacks['temperature'], qos_profile = rclpy.qos.qos_profile_default)

		self.enableTimer = self.create_timer(1.0 / messageRates['enableMotors'], self.enableTimerCallback)
		self.steeringTimer = self.create_timer(1.0 / messageRates['steering'], self.steeringTimerCallback)

	def destroyTimers(self):
		self.destroy_timer(self.enableTimer)
		self.destroy_timer(self.steeringTimer)

	'''Private methods'''

	def enableTimerCallback(self):
		message = RosMsgs.Bool()
		message.data = self.enabled
		self.publisherEnableMotors.publish(message)

	def steeringTimerCallback(self):
		if self.enabled is False:
			return

		message = RysMsgs.Steering()
		message.throttle = self.throttle
		message.rotation = self.rotation
		message.precision = self.precision
		self.publisherSteering.publish(message)

	'''Public methods'''

	def setEnabled(self, enabled):
		self.enabled = enabled
		self.enableTimerCallback()

	def setSteering(self, throttle, rotation, precision):
		self.throttle = float(throttle)
		self.rotation = float(rotation)
		self.precision = int(precision)

	def setBalancingEnabled(self, balancingEnabled):
		message = RosMsgs.Bool()
		message.data = balancingEnabled
		self.publisherEnableBalancing.publish(message)

	def requestSetRegulatorSettings(self, parameters):
		request = RysSrvs.SetRegulatorSettings.Request()

		request.speed_filter_factor = parameters['speedFilterFactor']
		request.angle_filter_factor = parameters['rollFilterFactor']
		request.lqr_enabled = parameters['lqrEnabled']
		request.pid_speed_regulator_enabled = parameters['pidSpeedRegulatorEnabled']
		request.pid_speed_kp = parameters['pidSpeedKp']
		request.pid_speed_ki = parameters['pidSpeedKi']
		request.pid_speed_kd = parameters['pidSpeedKd']
		request.pid_angle_kp = parameters['pidAngleKp']
		request.pid_angle_ki = parameters['pidAngleKi']
		request.pid_angle_kd = parameters['pidAngleKd']
		request.lqr_linear_velocity_k = parameters['lqrLinearVelocityK']
		request.lqr_angular_velocity_k = parameters['lqrAngularVelocityK']
		request.lqr_angle_k = parameters['lqrAngleK']

		self.clientSetRegulatorSettings.call(request)

	def requestGetRegulatorSettings(self):
		request = RysSrvs.GetRegulatorSettings.Request()
		self.clientGetRegulatorSettings.call(request)

	def getRequestResponses(self):
		setResponse = self.clientSetRegulatorSettings.response
		getResponse = self.clientGetRegulatorSettings.response

		self.clientSetRegulatorSettings.response = None
		self.clientGetRegulatorSettings.response = None

		return (setResponse, getResponse)

	def calibrateImu(self):
		message = RosMsgs.Empty()
		self.publisherIMUCalibrate.publish(message)
