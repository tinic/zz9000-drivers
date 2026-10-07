CC ?= cc
CFLAGS ?= -O2 -g -Wall -Wextra -Werror -std=c11

BUILD_DIR := build
TARGETS := $(BUILD_DIR)/mcast_test $(BUILD_DIR)/rx_test $(BUILD_DIR)/tx_test $(BUILD_DIR)/ext_test

.PHONY: test clean

test: $(TARGETS)
	$(BUILD_DIR)/mcast_test
	$(BUILD_DIR)/rx_test
	$(BUILD_DIR)/tx_test
	$(BUILD_DIR)/ext_test

$(BUILD_DIR)/mcast_test: mcast_test.c ../mcast.c ../mcast.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I.. -o $@ mcast_test.c ../mcast.c

$(BUILD_DIR)/rx_test: rx_test.c ../rx.c ../rx.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I.. -o $@ rx_test.c ../rx.c

$(BUILD_DIR)/tx_test: tx_test.c ../tx.c ../tx.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I.. -o $@ tx_test.c ../tx.c

$(BUILD_DIR)/ext_test: ext_test.c ../ext.c ../ext.h
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -I.. -o $@ ext_test.c ../ext.c

clean:
	rm -rf $(BUILD_DIR)
