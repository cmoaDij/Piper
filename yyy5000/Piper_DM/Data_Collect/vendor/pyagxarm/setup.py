from setuptools import setup, find_packages
from pathlib import Path

about = {}
exec(Path("pyAgxArm/version.py").read_text(encoding="utf-8"), about)

setup(
    name='pyAgxArm',
    version=about["__version__"],
    setup_requires=['setuptools>=40.0'],
    url='https://github.com/agilexrobotics/pyAgxArm',
    license='LGPL-3.0-only',
    packages=find_packages(
        include=['pyAgxArm', 'pyAgxArm.*'],
    ),
    include_package_data=True,
    package_data={
        '*': ['*.pyi'],
        'pyAgxArm': [
            'py.typed',
        ],
    },
    install_requires=[
        'python-can>=3.3.4',
        'typing-extensions>=3.7.4.3',
    ],
    author='Agilex Robotics Co., Ltd.',
    author_email='',
    description='Python SDK for Agilex robotic arms',
    platforms=['Linux', 'Windows', 'Darwin'],
    classifiers=[
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Operating System :: POSIX :: Linux',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: MacOS :: MacOS X',
    ],
    python_requires='>=3.6',
    project_urls={
        'Homepage': 'https://github.com/agilexrobotics/pyAgxArm',
        'ChangeLog': 'https://github.com/agilexrobotics/pyAgxArm/blob/master/CHANGELOG.md',
    },
)
