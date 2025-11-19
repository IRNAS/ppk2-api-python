from setuptools import setup, find_packages

def read_requirements():
    with open('requirements.txt') as f:
        return [line.strip() for line in f if line and not line.startswith('#')]

setup(
    name='smu-ppk2-gui',
    version='1.0.0',
    description='Intuitive GUI for SMU Emulation using Nordic PPK2',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='RolandWa',
    url='github.com/RolandWa/ppk2-api-python/tools/smu-ppk2-gui',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=read_requirements(),
    python_requires='>=3.8',
    entry_points={
        'console_scripts': [
            'smu-gui = gui_main:main'
        ]
    },
    include_package_data=True,
    package_data={'': ['schemas/*.svg', 'schemas/*.jpg', 'components/*.yaml']},
)