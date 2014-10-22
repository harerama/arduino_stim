from setuptools import setup

def readme():
    with open('README') as f:
        return f.read()

setup(name='arduino_stim',
      version='0.1',
      description='tDCS using Arduino device',
      url='http://github.com/storborg/funniest',
      author='Gderou',
      author_email='gderou@gmail.com',
      license='AGPLv3',
      packages=['arduino_stim'],
      zip_safe=False,
      install_requires=[
          'pyserial',
          'pySide'
      ],
      include_package_data=True,
      classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'License :: OSI Approved :: GNU Affero General Public License v3 or later (AGPLv3+)',
        'Programming Language :: Python :: 3.4',
      ],)
