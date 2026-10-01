from setuptools import setup
from glob import glob

setup(name='crc_solution', version='0.1.0', packages=['crc_solution'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/crc_solution']),
                  ('share/crc_solution', ['package.xml']),
                  ('share/crc_solution/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['driver = crc_solution.node:main']})
