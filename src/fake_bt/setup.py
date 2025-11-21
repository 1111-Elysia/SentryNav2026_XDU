from setuptools import setup

package_name = 'fake_bt'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=['fake_bt_manager'],  # scripts/fake_bt_manager.py
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='青春版行为树管理器',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'fake_bt_manager.py = fake_bt_manager:main',
        ],
    },
)
