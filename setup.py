#!/usr/bin/env python3
 
from setuptools import setup, find_packages
 
setup(name='SCSCtrl',
      version='0.0.0',
      description='TTLServoCtrl',
      author='waveshare',
      author_email='support@waveshare.com',
      maintainer='waveshare',
      maintainer_email='support@waveshare.com',
      url=' http://waveshare.com/',
      packages=find_packages(),
      install_requires=[
        'pyserial',
        'imutils',
      ],
      long_description=".",
      license="Public domain",
      platforms=["any"],
     )