from setuptools import setup

setup(
    name='axi',
    version='0.1',
    description='Library for working with the AxiDraw v3 pen plotter. Forked from Michael Fogleman',
    author='El Kaplan',
    packages=['axi'],
    package_data={'axi': ['*.ini']},
    entry_points={
        'console_scripts': [
            'axi = axi.main:main'
        ]
    },
    license='MIT',
    classifiers=(
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Natural Language :: English',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: Implementation :: CPython',
    ),
)
