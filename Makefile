TARGET = autopilot/autopilot.ino
BOARD = arduino:avr:nano
PORT = /dev/ttyUSB0

build:
	@-arduino --verify --board $(BOARD) $(TARGET)

upload:
	@-arduino --upload --board $(BOARD) --port $(PORT) $(TARGET)

.PHONY: build upload