from setuptools import setup

def readme():
    with open('README.md') as f:
        return f.read()

setup(name='python_i2c_mpu9250',
    version='0.1',
    description='Library for reading 10dof imu',
    long_description=readme(),
    url='https://github.com/modular-CAT/python_i2c_mpu9250',
    author='Daniel Smith',
    author_email='modular-CAT@users.noreply.github.com',
    license='MIT',
    packages=['python_i2c_mpu9250'],
    install_requires=[],
    zip_safe=False)
