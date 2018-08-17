import rclpy
from rclpy.node import Node
import time
from rys_interfaces import msg as RysMsgs
from rys_interfaces import srv as RysSrvs
from std_msgs import msg as StdMsgs
from nav_msgs import msg as NavigationMsgs
from sensor_msgs import msg as SensorMsgs

class RysRemoteNode(Node):
	"""docstring for RysRemoteNode"""

	def __init__(self, robotName, nodeName, callbacks, messageRates):
		super().__init__(nodeName, namespace = robotName)

		self.enabled = False

		self.throttle = 0.0
		self.rotation = 0.0
		self.precision = 1
		self.balancing = False

		# Create ROS publishers
		self.publisherEnableMotors = self.create_publisher(StdMsgs.Bool, '/' + robotName + '/control/enable_motors')
		# self.publisherEnableBalancing = self.create_publisher(StdMsgs.Bool, '/' + robotName + '/control/enable_balancing')
		self.publisherSteering = self.create_publisher(RysMsgs.Steering, '/' + robotName + '/control/steering')

		# Create ROS service clients
		self.clientSetRegulatorSettings = self.create_client(RysSrvs.SetRegulatorSettings, '/' + robotName + '/control/regulator_settings/set')
		self.clientGetRegulatorSettings = self.create_client(RysSrvs.GetRegulatorSettings, '/' + robotName + '/control/regulator_settings/get')

		self.clientSetRegulatorSettingsResponse = None
		self.clientGetRegulatorSettingsResponse = None

		# Create ROS subscribers
		batteryTopicName = '/' + robotName + '/sensor/battery'
		imuTopicName = '/' + robotName + '/sensor/imuInfrequent'
		rangesTopicName = '/' + robotName + '/sensor/ranges'
		temperatureTopicName = '/' + robotName + '/sensor/temperature'
		odometryTopicName = '/' + robotName + '/control/odometry'
		regulationTopicName = '/' + robotName + '/control/regulation'
		# qosProfileDefault = rclpy.qos.qos_profile_default
		qosProfileSensors = rclpy.qos.qos_profile_sensor_data
		qosProfileOdometry = rclpy.qos.QoSProfile(
			history = rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
			depth = 200,
			reliability = rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
			durability = rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
		)
		self.subscriptionBattery = self.create_subscription(RysMsgs.BatteryStatus, batteryTopicName, callbacks['battery'], qos_profile = qosProfileSensors)
		self.subscriptionImu = self.create_subscription(SensorMsgs.Imu, imuTopicName, callbacks['imu'], qos_profile = qosProfileSensors)
		self.subscriptionRanges = self.create_subscription(RysMsgs.Ranges, rangesTopicName, callbacks['ranges'], qos_profile = qosProfileSensors)
		self.subscriptionTemperature = self.create_subscription(RysMsgs.TemperatureStatus, temperatureTopicName, callbacks['temperature'], qos_profile = qosProfileSensors)
		self.subscriptionOdometry = self.create_subscription(NavigationMsgs.Odometry, odometryTopicName, callbacks['odometry'], qos_profile = qosProfileOdometry)
		self.subscriptionRegulation = self.create_subscription(RysMsgs.RegulationCallback, regulationTopicName, callbacks['regulation'], qos_profile = qosProfileSensors)

		self.enableTimer = self.create_timer(1.0 / messageRates['enableMotors'], self.enableTimerCallback)
		self.steeringTimer = self.create_timer(1.0 / messageRates['steering'], self.steeringTimerCallback)

	def destroyTimers(self):
		self.destroy_timer(self.enableTimer)
		self.destroy_timer(self.steeringTimer)

	'''Private methods'''

	def enableTimerCallback(self):
		message = StdMsgs.Bool()
		message.data = self.enabled
		self.publisherEnableMotors.publish(message)

	def steeringTimerCallback(self):
		if self.enabled is False:
			return

		message = RysMsgs.Steering()
		timeNow = time.time()
		message.header.stamp.sec = int(timeNow)
		message.header.stamp.nanosec = int((timeNow - message.header.stamp.sec) * 1000 * 1000 * 1000)
		message.header.frame_id = 'joystick'
		message.throttle = self.throttle
		message.rotation = self.rotation
		message.precision = self.precision
		message.balancing = self.balancing
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
		self.balancing = bool(balancingEnabled)
		# message = StdMsgs.Bool()
		# message.data = balancingEnabled
		# self.publisherEnableBalancing.publish(message)

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

		self.clientSetRegulatorSettingsResponse = self.clientSetRegulatorSettings.call(request)
		# self.clientSetRegulatorSettings.call(request)

	def requestGetRegulatorSettings(self):
		request = RysSrvs.GetRegulatorSettings.Request()
		self.clientGetRegulatorSettingsResponse = self.clientGetRegulatorSettings.call(request)
		# self.clientGetRegulatorSettings.call(request)

	def getRequestResponses(self):
		setResponse = self.clientSetRegulatorSettingsResponse
		getResponse = self.clientGetRegulatorSettingsResponse
		# setResponse = self.clientSetRegulatorSettings.response
		# getResponse = self.clientGetRegulatorSettings.response

		self.clientSetRegulatorSettingsResponse = None
		self.clientGetRegulatorSettingsResponse = None
		# self.clientSetRegulatorSettings.response = None
		# self.clientGetRegulatorSettings.response = None

		return (setResponse, getResponse)
