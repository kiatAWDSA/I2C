# I2C
A modified version of the alternative Arduino I2C library by DSSCircuits: https://github.com/DSSCircuits/I2C-Master-Library

Added functions:
- ping function: sends the first I2C message (i.e. address and R/W bit), but does not follow up with additional messages. Some devices such as Honeywell HIH8000 require this to trigger a measurement.
- error codes: Most functions now return an error code (defined in the header file) if I2C communication was unsuccessful. 
- allow user to enable/disable internal pull-up resistors in begin()
