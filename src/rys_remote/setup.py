from setuptools import setup

package_name = 'rys_remote'

setup(
	name=package_name,
	version='0.0.0',
	packages = [
		'',
		'Gamepad',
		'Mapper',
		'ROS',
		'UI',
		'UI.Layouts',
	],
	package_dir = {
		'': 'src',
		'Gamepad': 'src/Gamepad',
		'Mapper': 'src/Mapper',
		'ROS': 'src/ROS',
		'UI': 'src/UI',
		'UI.Layouts': 'src/UI/Layouts',
	},
	install_requires = [
		# 'launch',
		'setuptools',
		'PyQt5',
		'pygame',
	],
	data_files = [(
		'share/' + package_name,
		['package.xml']
	)],
	author='MJBogusz',
	author_email='mjbogusz.email.address@domain.name.com',
	maintainer='MJBogusz',
	maintainer_email='mjbogusz.email.address@domain.name.com',
	keywords=['ROS'],
	classifiers = [
		'Programming Language :: Python',
	],
	description = (
		'Node to remotely control and display data from MiniRys robot.'
	),
	license = '',
	entry_points = {
		'console_scripts': [
			'main = rys_remote:main',
		],
	},
)
