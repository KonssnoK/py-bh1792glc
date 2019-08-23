# pybh1792glc
python bh1792glc driver for raspberry

# Requirements
pip install smbus

# Functionalities
- Synchronized measurement
- Single measurement
- Unsynchronized measurement

# Example
- See test.py

# Enable i2c.0
I2C-0 is disabled by default. To enable it, you'll need to manually edit the configuration file.
Edit /boot/config.txt, and add the following line.

dtparam=i2c_vc=on