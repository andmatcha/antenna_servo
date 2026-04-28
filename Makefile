PIO ?= pio

PROJECT_DIR := .
ENV := nucleo_f303k8
DEBUG_ENV := nucleo_f303k8_debug

.PHONY: help build debug upload upload-debug clean clean-debug list info monitor restructure

help:
	@printf '%s\n' \
		'make build          Build release firmware' \
		'make debug          Build debug firmware with printf logs' \
		'make upload         Upload release firmware' \
		'make upload-debug   Upload debug firmware with printf logs' \
		'make monitor        Show ITM/SWO printf log output'

build:
	$(PIO) run --project-dir $(PROJECT_DIR) -e $(ENV)

debug:
	$(PIO) run --project-dir $(PROJECT_DIR) -e $(DEBUG_ENV)

upload:
	$(PIO) run --project-dir $(PROJECT_DIR) -e $(ENV) -t upload

upload-debug:
	$(PIO) run --project-dir $(PROJECT_DIR) -e $(DEBUG_ENV) -t upload

clean:
	$(PIO) run --project-dir $(PROJECT_DIR) -e $(ENV) -t clean

clean-debug:
	$(PIO) run --project-dir $(PROJECT_DIR) -e $(DEBUG_ENV) -t clean

list:
	$(PIO) device list

info:
	st-info --probe

monitor:
	./f3_swd_monitor.zsh

restructure:
	./restructure.sh
