from setuptools import find_packages, setup

package_name = 'classify_agent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/agent.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wujian',
    maintainer_email='wujian@todo.todo',
    description='Natural-language task agent for the Piper classification demo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = classify_agent.agent_node:main',
        ],
    },
)
