from setuptools import setup

setup(
	name='rys_remote',
	version='0.0.0',
	packages=[],
	py_modules=[
		'src.rys_remote'
	],
	install_requires=[
		'launch',
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
			'rys_remote = src.rys_remote:main',
		],
	},
)
