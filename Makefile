COMPONENT=vmC
include $(MAKERULES)
SENSORBOARD ?= im2sb
CFLAGS += -DTOSH_DATA_LENGTH=100