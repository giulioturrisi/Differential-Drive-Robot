from setuptools import setup

package_name = 'controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['run_controllers = controllers.run_controllers:main',
                            'run_casadi = controllers.run_casadi_nmpc:main',
                            'run_acados = controllers.run_acados_nmpc:main',
                            'run_ilqr = controllers.run_ilqr:main',
                            'run_approx_linearization = controllers.run_approx_linearization:main',
                            'run_io_linearization = controllers.run_io_linearization:main',
                            'run_nonlinear_lyapunov = controllers.run_nonlinear_lyapunov:main',
                            'run_dynamic_linearization = controllers.run_dynamic_linearization:main',
        ],
    },
)
