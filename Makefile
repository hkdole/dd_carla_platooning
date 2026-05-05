WORKSPACE_ROOT := /home/ubuntu/workspace
PROJECT_ROOT   := /home/ubuntu/workspace/carla_platooning

OMNETPP_ROOT    := /home/ubuntu/workspace/omnetpp-6.0.3
INET_ROOT       := /home/ubuntu/workspace/omnetpp-6.0.3/v2v/inet4.5
VEINS_ROOT      := /home/ubuntu/workspace/veins
PLEXE_ROOT      := /home/ubuntu/workspace/plexe
CARLANET_ROOT   := /home/ubuntu/workspace/carlanetpp
PYCARLANET_ROOT := /home/ubuntu/workspace/pycarlanet

SRC_DIR     := $(PROJECT_ROOT)/src
EXAMPLE_DIR := $(PROJECT_ROOT)/examples/joinAtBack

.PHONY: paths build clean run

all: build

paths:
	@echo "PROJECT_ROOT=$(PROJECT_ROOT)"
	@echo "OMNETPP_ROOT=$(OMNETPP_ROOT)"
	@echo "INET_ROOT=$(INET_ROOT)"
	@echo "VEINS_ROOT=$(VEINS_ROOT)"
	@echo "PLEXE_ROOT=$(PLEXE_ROOT)"
	@echo "CARLANET_ROOT=$(CARLANET_ROOT)"
	@echo "PYCARLANET_ROOT=$(PYCARLANET_ROOT)"

build:
	$(MAKE) -C $(SRC_DIR)

clean:
	$(MAKE) -C $(SRC_DIR) clean

run:
	cd $(EXAMPLE_DIR) && $(OMNETPP_ROOT)/bin/opp_run -m -u Cmdenv \
	  -n ..:$(PYCARLANET_ROOT):$(SRC_DIR):$(CARLANET_ROOT)/simulations:$(CARLANET_ROOT)/src:$(INET_ROOT)/examples:$(INET_ROOT)/showcases:$(INET_ROOT)/src:$(INET_ROOT)/tests/validation:$(INET_ROOT)/tests/networks:$(PLEXE_ROOT)/examples/platooning:$(PLEXE_ROOT)/src/plexe:$(PLEXE_ROOT)/src:$(VEINS_ROOT)/examples:$(VEINS_ROOT)/src/veins:$(VEINS_ROOT)/src \
	  -l $(PROJECT_ROOT)/out/gcc-debug/src/libcarla_dbg.so \
	  -l $(CARLANET_ROOT)/src/carlanet \
	  -l $(INET_ROOT)/src/INET \
	  -l $(PLEXE_ROOT)/src/plexe \
	  -l $(VEINS_ROOT)/src/veins \
	  omnetpp.ini
