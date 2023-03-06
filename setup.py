import setuptools

with open('README.md', encoding='utf-8') as readme_file:
    readme = readme_file.read()

# replace relative urls to example files with absolute urls to the main git repo
repo_code_url = "https://github.com/da-vidi21/HP_DA_PT19"
# long_description = long_description.replace("](examples/", "]({}/examples/".format(repo_code_url))

setuptools.setup(
    name='hp_da_pt19',
    packages=['hp_da_pt19'],
    version='0.0.5',
    license='MIT',
    description='Package contaning python wrapper for supporting pitch,roll,yaw, swarms, and more',
    long_description=readme,
    long_description_content_type='text/markdown',
    author='Anonymous',
    author_email='vidit21srivastava@gmail.com',
    url='https://github.com/da-vidi21/HP_DA_PT19',
    keywords=['drone', 'sdk', 'python'],
    install_requires=[
        'matplotlib==3.6.2',
        'numpy==1.23.5',
        'opencv_contrib_python==4.5.5.64',
        'opencv-python',
        'av',
        'pillow'
    ],
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'Topic :: Software Development :: Build Tools',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
)
