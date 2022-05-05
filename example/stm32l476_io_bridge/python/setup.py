 
from setuptools import setup, find_packages

setup(name='iobridge',
      version='0.1',
      description='STM32 IOBridge API',
      url='http://github.com/70p4z/iobridge',
      author='Olivier TOMAZ',
      author_email='topaz@free.fr',
      license='Apache',
      packages=find_packages(),
      install_requires=[
          'pyserial',
      ],
      zip_safe=False)
