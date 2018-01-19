# README #

Caffe Prediction to LED

### What is this repository for? ###

* This is for our guidedog preject. For HRI, Debug and demo videos.
* Version 1.0

### How do I get set up? ###

| LED | Pi2 |
| :-: | :-: |
| +5v | 5V |
| GND | GND|
| Di | GPIO18 |

rpi gpio pin map ([LINK](http://raspi.tv/wp-content/uploads/2014/07/Raspberry-Pi-GPIO-pinouts.png)) 

* sudo apt-get install build-essential python-dev git scons swig
* git clone https://github.com/jgarff/rpi_ws281x.git
* cd rpi\_ws281x
* scons
* cd python
* sudo python setup.py install
* cd example
* sudo python standtest.py

### Who do I talk to? ###

* Monica Lin
