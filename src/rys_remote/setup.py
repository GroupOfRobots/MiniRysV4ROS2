from setuptools import setup

package_name = 'rys_remote'

setup(
	name=package_name,
	version='0.0.0',
	packages = [
		'',
		'Gamepad',
		'ROS',
		'UI',
		'UI.Layouts',
	],
	package_dir = {
		'': 'src',
		'Gamepad': 'src/Gamepad',
		'ROS': 'src/ROS',
		'UI': 'src/UI',
		'UI.Layouts': 'src/UI/Layouts',
	},
	install_requires=[
		# 'launch',
		'setuptools',
		'PyQt5',
		'pygame',
	],
	author='MJBogusz',
	author_email='mjbogusz.email.address@domain.name.com',
	maintainer='MJBogusz',
	maintainer_email='mjbogusz.email.address@domain.name.com',
	keywords=['ROS'],
	classifiers=[
		'Programming Language :: Python',
	],
	description=(
		'Node to remotely control and display data from MiniRys robot.'
	),
	license='',
	entry_points={
		'console_scripts': [
			'main = rys_remote:main',
		],
	},
)
