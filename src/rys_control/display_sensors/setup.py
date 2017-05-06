from setuptools import setup, find_packages

setup(
	name='rys_display_sensors',
	version='0.0.0',
	packages=find_packages(),
	install_requires=[
		'launch',
		'setuptools',
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
		'Node to display sensor readings from MiniRys robot.'
	),
	license='',
	entry_points={
		'console_scripts': [
			'rys_display_sensors = src.rys_display_sensors:main',
		],
	},
)
